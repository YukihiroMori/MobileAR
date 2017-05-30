//
// Computation of plane and motion parameters
//
#ifndef _PLANE_RECONSTRUCTION_H_
#define _PLANE_RECONSTRUCTION_H_

#include <Eigen/Dense>

using namespace Eigen;

typedef Matrix<double,9,9> Matrix9d;
typedef Matrix<double,9,4> Matrix94d;
typedef Matrix<double,9,1> Vector9d;
typedef DiagonalMatrix<double,3> DiagMatrix3d;

namespace {
   // small function
   double my_cbrt(double a)
   {
      double x, d;

      // initial value from pow
      x = pow(a, 1.0/3.0);

      // Newton iteration
      do
      {
         d = (x*x* x - a) / (3.0*x*x);
         x = x - d;
      }
      while (fabs(d) > 1.0e-16);

    return x;
   }
}

namespace PlaneReconstruction {
   // constants
   const double Convergence_EPS = 1e-6;
   const int    Max_Iteration =  100;
   const double Large_Number = 1e99;
   const double Default_f0 = 600;

   const Matrix9d ZeroMat9 = Matrix9d::Zero();
   const Matrix3d ZeroMat3 = Matrix3d::Zero();
   const Matrix94d ZeroMat94 = Matrix94d::Zero();
   const Vector3d ZeroVec3 = Vector3d::Zero();
   const Vector4d ZeroVec4 = Vector4d::Zero();
   const Vector9d ZeroVec9 = Vector9d::Zero();
   const Matrix3d I3 = Matrix3d::Identity();
   const DiagonalMatrix<double,3> Pk(1.0, 1.0, 0.0);

   // compute plane and motion parameters
   bool plane_reconstruction(const Matrix3d& H,
                             double fl0,
                             double fl1,
                             Vector3d& np,
                             Vector3d& nm,
                             double *dist,
                             Vector3d& tp,
                             Vector3d& tm,
                             Matrix3d& Rp,
                             Matrix3d& Rm,
                             double f0 = Default_f0);

   // choose solution by parameters
   bool choose_solution_by_parameters(const Vector3d& np,
                                      const Vector3d& nm,
                                      double dist,
                                      const Matrix3d& Rp,
                                      const Matrix3d& Rm,
                                      bool cam1,
                                      bool cam2,
                                      bool check[]);

   // choose solution by points
   // Triangulation
   bool check_solution(Vector2d pos0[],
                       Vector2d pos1[],
                       int Num,
                       const Vector3d& tr,
                       const Matrix3d& R,
                       double fl0,
                       double fl1,
                       Vector3d Xp[],
                       double f0 = Default_f0);

   // choose solution by points
   bool choose_solution_by_points(Vector2d pos0[],
                                  Vector2d pos1[],
                                  int Num,
                                  const Vector3d& tp,
                                  const Matrix3d& Rp,
                                  const Vector3d& tm,
                                  const Matrix3d& Rm,
                                  double fl0,
                                  double fl1,
                                  bool check[],
                                  double f0 = Default_f0);
}

#endif // _PLANE_RECONSTRUCTION_H_
