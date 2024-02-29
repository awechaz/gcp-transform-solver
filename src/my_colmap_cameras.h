#include "ceres/ceres.h"
#include "ceres/rotation.h"

template <typename T> 
void OpenCvCameraModelDistortion(
    const T* extra_params, const T u, const T v, T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T p1 = extra_params[2];
  const T p2 = extra_params[3];

  const T u2 = u * u;
  const T uv = u * v;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T radial = k1 * r2 + k2 * r2 * r2;
  *du = u * radial + T(2) * p1 * uv + p2 * (r2 + T(2) * u2);
  *dv = v * radial + T(2) * p2 * uv + p1 * (r2 + T(2) * v2);
}



template <typename T> 
void OpenCvCameraModelImgFromCam(const T* params, T u, T v, T w, T* x, T* y) {
  // from COLMAP (OpenCVCameraModel)
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  u /= w;
  v /= w;

  // Distortion
  T du, dv;
  OpenCvCameraModelDistortion(&params[4], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f1 * *x + c1;
  *y = f2 * *y + c2;
}



template <typename T>
void OpenCvCameraModelImgFromWorld(const T* const photo, const T* const camera, const T* const point, T* x, T* y) { 
    // camera[0,1,2,3] is are the rotation of the camera as a quaternion.
    //
    // We use QuaternionRotatePoint as it does not assume that the
    // quaternion is normalized, since one of the ways to run the
    // bundle adjuster is to let Ceres optimize all 4 quaternion
    // parameters without a local parameterization.
    T p[3];
    ceres::QuaternionRotatePoint(photo, point, p);

    p[0] += photo[4];
    p[1] += photo[5];
    p[2] += photo[6];

    OpenCvCameraModelImgFromCam(camera + 2, p[0], p[1], p[2], x, y);
}