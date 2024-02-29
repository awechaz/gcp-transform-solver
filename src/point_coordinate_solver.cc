/*
Solves for world coordinates of annotated points given solved cameras.

Allows us to quickly determine the locations of points given an atlas without redoing the whole bundle adjustment. 
*/

// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2023 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.

#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

// for undistortion
// #include <Eigen/Core>
// #include <Eigen/LU>
#include <ceres/jet.h>

#include "my_colmap_cameras.h"

#define NUM_EXTRINSIC_CAMERA_PARAMETERS 7 // + 1 if we count the camera index (intrinsic parameters)
#define NUM_INTRINSIC_CAMERA_PARAMETERS 10

// Read a Bundle Adjustment in the Large dataset.
class BALProblem {
 public:
  ~BALProblem() {
    delete[] point_index_;
    delete[] photo_index_; // extrinsic camera parameters
    delete[] camera_index_; // intrinsic camera parameters
    delete[] observations_; // image-plane coordinates
    delete[] parameters_; // typically intrinsic & extrinsic camera parameters and point coordinates, but in this case will be just a coordinate transform
  }

  int num_observations() const { return num_observations_; }
  int num_cameras() const { return num_cameras_; }
  int num_photos() const { return num_photos_; }
  int num_points() const { return num_points_; }
  const double* observations() const { return observations_; }
  double* mutable_photos() { return parameters_; }
  double* mutable_cameras() { 
    return parameters_ + NUM_EXTRINSIC_CAMERA_PARAMETERS * num_photos_; 
  }
  double* mutable_points() { 
    return parameters_ + NUM_EXTRINSIC_CAMERA_PARAMETERS * num_photos_ \
                       + NUM_INTRINSIC_CAMERA_PARAMETERS * num_cameras_;
  }
  int camera_index_for_observation(int i) { return camera_index_[photo_index_[i]]; }
  int photo_index_for_observation(int i) { return photo_index_[i]; }

  double* mutable_photo_for_observation(int i) {
    return mutable_photos() + photo_index_[i] * NUM_EXTRINSIC_CAMERA_PARAMETERS; // skip the camera index (intrinsic parameters)
  }
  double* mutable_camera_for_observation(int i) {
    return mutable_cameras() + camera_index_for_observation(i) * NUM_INTRINSIC_CAMERA_PARAMETERS;
  }
  double* mutable_point_for_observation(int i) {
    return mutable_points() + point_index_[i] * 3;
  }

  bool LoadFile(const char* filename) {
    FILE* fptr = fopen(filename, "r");
    if (fptr == nullptr) {
      return false;
    };

    FscanfOrDie(fptr, "%d", &num_photos_);
    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);

    point_index_ = new int[num_observations_];
    photo_index_ = new int[num_observations_];
    camera_index_ = new int[num_photos_];
    observations_ = new double[2 * num_observations_];

    num_parameters_ = NUM_EXTRINSIC_CAMERA_PARAMETERS * num_photos_ \
                      + NUM_INTRINSIC_CAMERA_PARAMETERS * num_cameras_ \
                      + 3 * num_points_ \
                      + 6; // rodriguez (3) and translation (3) vectors parametrizing GCP coordinate transform (inverse)
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_observations_; ++i) {
      FscanfOrDie(fptr, "%d", photo_index_ + i);
      FscanfOrDie(fptr, "%d", point_index_ + i);
      for (int j = 0; j < 2; ++j) {
        FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
      }
    }

    // read extrinsic camera parameters
    for (int i = 0; i < num_photos_; ++i) {
      FscanfOrDie(fptr, "%d", camera_index_ + i);
      for (int j = 0; j < NUM_EXTRINSIC_CAMERA_PARAMETERS; ++j) {
        FscanfOrDie(fptr, "%lf", mutable_photos() + i * NUM_EXTRINSIC_CAMERA_PARAMETERS + j);
      }
    }

    // read intrinsic camera parameters
    for (int i = 0; i < num_cameras_; ++i) {
      for (int j = 0; j < NUM_INTRINSIC_CAMERA_PARAMETERS; ++j) {
        FscanfOrDie(fptr, "%lf", mutable_cameras() + i * NUM_INTRINSIC_CAMERA_PARAMETERS + j);
      }
    }

    // read points
    for (int i = 0; i < num_points_; ++i) {
      for (int j = 0; j < 3; ++j) {
        FscanfOrDie(fptr, "%lf", mutable_points() + i * 3 + j);
      }
    }

    // un-normalize observations
    for (int i = 0; i < num_observations_; ++i) {
      double* camera = parameters_ + NUM_EXTRINSIC_CAMERA_PARAMETERS * num_photos_ + camera_index_[photo_index_[i]] * NUM_INTRINSIC_CAMERA_PARAMETERS;
      int width = camera[0];
      int height = camera[1];
      observations_[2*i] = (width / 2.0) + width/2.0 * observations_[2*i];
      observations_[2*i+1] = (height / 2.0) + width/2.0 * observations_[2*i+1];
    }

    return true;
  }

  void WriteFile(const std::string& filename) const {
    FILE* fptr = fopen(filename.c_str(), "w");

    if (fptr == nullptr) {
      LOG(FATAL) << "Error: unable to open file " << filename;
      return;
    };

    fprintf(fptr, "%d %d %d %d\n", num_photos_, num_cameras_, num_points_, num_observations_);

    // normalize observations
    for (int i = 0; i < num_observations_; ++i) {
      double* camera = parameters_ + NUM_EXTRINSIC_CAMERA_PARAMETERS * num_photos_ + camera_index_[photo_index_[i]] * NUM_INTRINSIC_CAMERA_PARAMETERS;
      int width = camera[0];
      int height = camera[1];
      observations_[2*i] = (observations_[2*i] - (width / 2.0)) / (width / 2.0);
      observations_[2*i+1] = (observations_[2*i+1] - (height / 2.0)) / (width / 2.0);
    }

    for (int i = 0; i < num_observations_; ++i) {
      fprintf(fptr, "%d %d", photo_index_[i], point_index_[i]);
      for (int j = 0; j < 2; ++j) {
        fprintf(fptr, " %g", observations_[2 * i + j]);
      }
      fprintf(fptr, "\n");
    }

    double* photos = parameters_;
    for (int i = 0; i < num_photos(); ++i) {
      fprintf(fptr, "%d ", camera_index_[i]);
      for (int j = 0; j < NUM_EXTRINSIC_CAMERA_PARAMETERS; ++j) {
        fprintf(fptr, "%g ", photos[i * NUM_EXTRINSIC_CAMERA_PARAMETERS + j]);
      }
      fprintf(fptr, "\n");
    }

    double* cameras = photos + num_photos_ * NUM_EXTRINSIC_CAMERA_PARAMETERS;
    for (int i = 0; i < num_cameras(); ++i) {
      for (int j = 0; j < NUM_INTRINSIC_CAMERA_PARAMETERS; ++j) {
        fprintf(fptr, "%g ", cameras[i * NUM_INTRINSIC_CAMERA_PARAMETERS + j]);
      }
      fprintf(fptr, "\n");
    }

    double* points = cameras + num_cameras_ * NUM_INTRINSIC_CAMERA_PARAMETERS;
    for (int i = 0; i < num_points(); ++i) {
      for (int j = 0; j < 3; ++j) {
        fprintf(fptr, "%g ", points[i * 3 + j]);
      }
      fprintf(fptr, "\n");
    }

    fclose(fptr);
  }

 private:
  template <typename T>
  void FscanfOrDie(FILE* fptr, const char* format, T* value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1) {
      LOG(FATAL) << "Invalid UW data file.";
    }
  }

  int num_cameras_;
  int num_photos_;
  int num_points_;
  int num_observations_;
  int num_parameters_;

  int* point_index_;
  int* photo_index_;
  int* camera_index_;
  double* observations_;
  double* parameters_;
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 10 parameters. 4 for rotation, 3 for
// translation, 1 for focal length and 2 for radial distortion. The
// principal point is not modeled (i.e. it is assumed be located at
// the image center).
struct ReprojectionError {
  // (u, v): the position of the observation with respect to the image center point.
  ReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(
    const T* const photo,
    const T* const camera,
    const T* const point,
    T* residuals
  ) const {

    T predicted_x, predicted_y;
    OpenCvCameraModelImgFromWorld(photo, camera, point, &predicted_x, &predicted_y);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x; 
    residuals[1] = predicted_y - observed_y; 

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<
            ReprojectionError, 2, NUM_EXTRINSIC_CAMERA_PARAMETERS, NUM_INTRINSIC_CAMERA_PARAMETERS, 3>(
                new ReprojectionError(observed_x, observed_y)));
  }

  double observed_x;
  double observed_y;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  if (argc != 3) {
    std::cerr << "usage: point_coordinate_solver <point_coordinate_problem> <point_coordinate_solution>\n";
    return 1;
  }

  BALProblem bal_problem;
  if (!bal_problem.LoadFile(argv[1])) {
    std::cerr << "ERROR: unable to open file " << argv[1] << "\n";
    return 1;
  }

  std::string output_filename = argv[2];

  const double* observations = bal_problem.observations();

  // initialize points at origin?
  // double* points = bal_problem.mutable_points();
  // for (int i = 0; i < bal_problem.num_points(); ++i) {
  //   for (int j = 0; j < 3; ++j) {
  //     points[i] = 0.;
  //   }
  // }

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  std::vector< ceres::ResidualBlockId > residualBlocksToEvaluateAfterSolve;
  std::vector<int> usedPhotos;
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.

    ceres::CostFunction* cost_function = ReprojectionError::Create(
        observations[2 * i + 0], observations[2 * i + 1]);
    //problem.AddResidualBlock(
    ceres::ResidualBlockId residualBlockId = problem.AddResidualBlock(
      cost_function,
      nullptr /* squared loss */,
      bal_problem.mutable_photo_for_observation(i),
      bal_problem.mutable_camera_for_observation(i),
      bal_problem.mutable_point_for_observation(i)
    );
    residualBlocksToEvaluateAfterSolve.push_back( residualBlockId );

    // sure, this is done over and over again when observations share photos and cameras but whatever:
    problem.SetParameterBlockConstant(bal_problem.mutable_camera_for_observation(i)); 
    problem.SetParameterBlockConstant(bal_problem.mutable_photo_for_observation(i)); 
    //problem.SetParameterBlockConstant(bal_problem.mutable_point_for_observation(i)); 
  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  bal_problem.WriteFile(output_filename);

  // extract residuals
  std::cout << "Residuals after solve:\n";
  double totalCost = 0.0;
  ceres::Problem::EvaluateOptions postSolveOptions;
  for( auto i=0; i<residualBlocksToEvaluateAfterSolve.size() ; i++ ){
    postSolveOptions.residual_blocks = {residualBlocksToEvaluateAfterSolve[i]};
    std::vector<double> evaluatedResiduals;
    problem.Evaluate(postSolveOptions, &totalCost, &evaluatedResiduals, nullptr, nullptr);
    std::cout << i << ": ";
    for( auto k=0; k<evaluatedResiduals.size() ; k++ ){
        std::cout << evaluatedResiduals[k] << " ";
    }
    std::cout << "\n";
  }
  std::cout << "Total cost: " << totalCost << "\n";

  std::cout << "\nSolved point coordinates are:\n";
  for (int i=0; i < bal_problem.num_points(); i++){
    for (int j=0; j < 3; j++){
      std::cout << bal_problem.mutable_points()[3*i + j] << " ";
    }
    std::cout << "\n";
  }

  return 0;
}

// template <typename T> 
// void OpenCvCameraModelIterativeUndistortion(const T* params,
//                                                          T* u,
//                                                          T* v) {
//   // Parameters for Newton iteration using numerical differentiation with
//   // central differences, 100 iterations should be enough even for complex
//   // camera models with higher order terms.
//   const size_t kNumIterations = 100;
//   const double kMaxStepNorm = 1e-10;
//   const double kRelStepSize = 1e-6;

//   Eigen::Matrix2d J;
//   const Eigen::Vector2d x0(*u, *v);
//   Eigen::Vector2d x(*u, *v);
//   Eigen::Vector2d dx;
//   Eigen::Vector2d dx_0b;
//   Eigen::Vector2d dx_0f;
//   Eigen::Vector2d dx_1b;
//   Eigen::Vector2d dx_1f;

//   for (size_t i = 0; i < kNumIterations; ++i) {
//     const double step0 = std::max(std::numeric_limits<double>::epsilon(),
//                                   std::abs(kRelStepSize * x(0)));
//     const double step1 = std::max(std::numeric_limits<double>::epsilon(),
//                                   std::abs(kRelStepSize * x(1)));
//     OpenCvCameraModelDistortion(params, x(0), x(1), &dx(0), &dx(1));
//     OpenCvCameraModelDistortion(params, x(0) - step0, x(1), &dx_0b(0), &dx_0b(1));
//     OpenCvCameraModelDistortion(params, x(0) + step0, x(1), &dx_0f(0), &dx_0f(1));
//     OpenCvCameraModelDistortion(params, x(0), x(1) - step1, &dx_1b(0), &dx_1b(1));
//     OpenCvCameraModelDistortion(params, x(0), x(1) + step1, &dx_1f(0), &dx_1f(1));
//     J(0, 0) = 1 + (dx_0f(0) - dx_0b(0)) / (2 * step0);
//     J(0, 1) = (dx_1f(0) - dx_1b(0)) / (2 * step1);
//     J(1, 0) = (dx_0f(1) - dx_0b(1)) / (2 * step0);
//     J(1, 1) = 1 + (dx_1f(1) - dx_1b(1)) / (2 * step1);
//     const Eigen::Vector2d step_x = J.partialPivLu().solve(x + dx - x0);
//     x -= step_x;
//     if (step_x.squaredNorm() < kMaxStepNorm) {
//       break;
//     }
//   }

//   *u = x(0);
//   *v = x(1);
// }

// template <typename T> 
// void OpenCvCameraModelCamFromImg(
//   // from COLMAP (OpenCVCameraModel)
//   const T* params, const T x, const T y, T* u, T* v, T* w) 
// {
//   const T f1 = params[0];
//   const T f2 = params[1];
//   const T c1 = params[2];
//   const T c2 = params[3];

//   // Lift points to normalized plane
//   *u = (x - c1) / f1;
//   *v = (y - c2) / f2;
//   *w = 1;

//   OpenCvCameraModelIterativeUndistortion(&params[4], u, v);
// }