/*
Tests the GCP coordinate transform by confirming that points at known world locations 
are all mapped to near where they have been observed.
*/

#include <cstdio>
#include <iostream>

#include <iostream>
#include <fstream>
#include <vector>

#include "my_colmap_cameras.h"

#include <Eigen/Dense>

// // Function to multiply a matrix by a vector using Eigen
// void matrixVectorMultiply(const Eigen::MatrixXd& matrix, const double* vector, double* result, int vectorSize) {
//     // Ensure the matrix and vector are compatible for multiplication
//     if (matrix.cols() != vectorSize) {
//         std::cerr << "Matrix and vector dimensions are not compatible for multiplication." << std::endl;
//         exit(EXIT_FAILURE);
//     }

//     // Perform matrix-vector multiplication
//     Eigen::Map<const Eigen::VectorXd> vectorMap(vector, vectorSize);
//     Eigen::Map<Eigen::VectorXd> resultMap(result, vectorSize);
//     resultMap = matrix * vectorMap;
// }

// int load_xyz(char *filename, std::vector<std::vector<double>>& data){

//     // Open the file
//     std::ifstream file(filename);
//     if (!file.is_open()) {
//         std::cerr << "Unable to open file: " << filename << std::endl;
//         return 1;
//     }

//     // Read the data from the file into the vector
//     double value;
//     while (file >> value) {
//         std::vector<double> row;
//         row.push_back(value);

//         for (int i = 0; i < 2; ++i) {
//             if (!(file >> value)) {
//                 std::cerr << "Error: Unexpected end of file or invalid format." << std::endl;
//                 return 1;
//             }
//             row.push_back(value);
//         }

//         data.push_back(row);
//     }

//     // Close the file
//     file.close();

//     return 0;
// }

#define NUM_POINTS 7

int main(int argc, char** argv) {
    /*
    Transforms a point in GCP space into its image coordinate, via:
    - a GCP transform to COLMAP space
    - a COLMAP camera model projection
    */

    // Using hand-annotated observations in image DSCF0199.JPG:
    double photo[7] = {0.9809090224941557, -0.01542673018418325, -0.1915471628214433, 0.029819289066640907, 1.932797670712349, 0.03520878695439343, 2.7861206866606865}; 
    double camera[10] = {6000, 4000, 11659.214941992816, 11699.68320504099, 3000, 2000, -0.06475037283746247, 0.4252632399628402, -0.001169087366460162, -0.00033853124191879634};

    double gcps[NUM_POINTS][3] = {
        47.0, 25.0, 0.0,    // tr_court_corner_i
        28.0, 8.0, 0.0,     // tr_ftline_at_laneline_oo
        28.0, -8.0, 0.0,    // br_ftline_at_laneline_oo
        -28.0, 8.0, 0.0,    // tl_ftline_at_laneline_oo
        -28.0, -8.0, 0.0,   // bl_ftline_at_laneline_oo
        0.0, 25.0, 0.0,     // t_divline_at_sideline
        0.0, -25.0, 0.0    // b_divline_at_sideline
    };

    double observations[NUM_POINTS][2] = { \
        0.8849713802337646, -0.226788600285848,     // tr_court_corner_i
        0.7173482179641724, -0.09943048159281413,   // tr_ftline_at_laneline_oo
        0.9607706069946289, 0.009149273236592611,   // br_ftline_at_laneline_oo
        -0.6649512052536011, 0.04917287826538086,   // tl_ftline_at_laneline_oo
        -0.5497916340827942, 0.19282921155293783,   // bl_ftline_at_laneline_oo
        -0.08861970901489258, -0.1340324878692627,  // t_divline_at_sideline
        0.5128010511398315, 0.2548806667327881      // b_divline_at_sidelin
    };

    // Load GCP transform
    // double gcp_transform[7];
    // FILE* fptr = fopen(argv[1], "r");
    // if (fptr == nullptr) {
    //   return 1;
    // };
    // for (int i=0; i<7; i++){
    //     fscanf(fptr, "%lf", gcp_transform + i);
    // }
    // std::cout << "gcp_transform:\n";
    // for (int i=0; i<7; i++){
    //     std::cout << gcp_transform[i] << " ";
    // }
    // std::cout << "\n\n";

    double s;
    double R[3][3];
    double t[3];
    FILE* fptr = fopen(argv[1], "r");
    if (fptr == nullptr) {
      return 1;
    };
    fscanf(fptr, "%lf", &s);  // Read scale
    for (int i=0; i < 3; i++){ // Read Rotation matrix
        for (int j=0; j < 3; j++){ 
            // (TODO: We are only using the full rotation matrix for now until we figure out the right function to apply e.g. Rodriges vector)
            fscanf(fptr, "%lf", &R[i][j]);
        }
    }
    for (int i=0; i < 3; i++){ // Read translation vector
        fscanf(fptr, "%lf", &t[i]); 
    }

    // Confirm we read everything right
    // std::cout << "R:\n";
    // for (int i=0; i < 3; i++){
    //     for (int j=0; j < 3; j++){
    //         std::cout << R[i][j] << " ";
    //     }
    //     std::cout << "\n";
    // }
    // std::cout << "\n";


    double x, y;
    for (int k=0; k < NUM_POINTS; k++){

        // transform from our coordinates to COLMAP coordinates

        // rotation
        double point[3] = {0., 0., 0.};  // will store the result of rotating the coordinate system around the Rodrigues vector.
        // TODO: this isn't working with the Rodriges vector output, figure out why not
        // ceres::AngleAxisRotatePoint(&gcp_transform[1], &gcps[k][0], point); // The Rodrigues angle-axis rotation vector is camera[3-5].

        // double s = gcp_transform[0];
        // for (int k=0; k<3; k++){
        //     // point[k] = s * point[k];
        //      point[k] = s * point[k] + gcp_transform[4+k]; // scaling and translation
        // }

        // Rotation
        double* point_gcp = gcps[k];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                point[i] += R[i][j] * point_gcp[j];
            }
        }

        // scaling
        for (int i=0; i<3; i++){
            point[i] = s * point[i];
        }

        // translation
        for (int i=0; i<3; i++){
            point[i] = point[i] + t[i]; 
            //point[k] = point[k] + gcp_transform[4+k]; 
        }

        // print GCP coordinates
        for (int i=0; i < 3; i++){
            std::cout << gcps[k][i] << " ";
        }
        std::cout << "--> ";

        // print COLMAP coordinates
        for (int i=0; i < 3; i++){
            std::cout << point[i] << " ";
        }
        std::cout << "\n";


        // apply COLMAP camera model to the point in COLMAP coordinates
        OpenCvCameraModelImgFromWorld(photo, camera, point, &x, &y);

        // double xn, yn;
        // xn = (x - (camera[0] / 2.0) ) / (camera[0] / 2.0);
        // yn = (y - (camera[1] / 2.0) ) / (camera[0] / 2.0); // yes [0] in the denominator
        // std::cout << "estimated (xn,yn) = (" << xn << ", " << yn << ")\n";
        // std::cout << "true (xn,yn)      = (" << observations[k][0] << ", " << observations[k][1] << ")\n\n";
        // double err_xn = xn - observations[k][0];
        // double err_yn = yn - observations[k][1];
        // std::cout << "residual (pixels):  (" << err_xn << ", " << err_yn << ")\n";
        double obs_x = (camera[0] / 2.0) + (camera[0] / 2.0) * observations[k][0];
        double obs_y = (camera[1] / 2.0) + (camera[0] / 2.0) * observations[k][1];
        double err_x = x - obs_x;
        double err_y = y - obs_y;
        double err = sqrt(err_y * err_y + err_x * err_x);
        std::cout << "(x,y)         = (" << x << ", " << y << ")\n";
        std::cout << "(x_hat,y_hat) = (" << obs_x << ", " << obs_y << ")\n";
        //std::cout << "residual =  (" << err_x << ", " << err_y << ")\n";
        std::cout << "pixel error = " << err << "\n\n";
    }

    // Print errors

    return 0;

}