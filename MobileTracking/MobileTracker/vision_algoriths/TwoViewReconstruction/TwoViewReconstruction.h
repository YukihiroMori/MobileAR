//
// Two View Reconstruction
//
#ifndef _TWO_VIEW_RECONSTRUCTION_H_
#define _TWO_VIEW_RECONSTRUCTION_H_

#include <Eigen/Dense>
#include "Triangulation.h"

using namespace Eigen;

typedef Matrix<double,9,3> Matrix93d;
typedef Matrix<double,7,7> Matrix7d;
typedef Matrix<double,7,1> Vector7d;

// unname namespace for small functions
namespace {
   // local small functions
   Matrix3d convert_mat3(const Vector9d& th)
   {
      Matrix3d F;
      F << th(0), th(1), th(2), th(3), th(4), th(5), th(6), th(7), th(8);
      return F;
   };

   Vector9d convert_vec9(const Matrix3d& F)
   {
      Vector9d theta;
      theta << F(0,0), F(0,1), F(0,2), F(1,0), F(1,1), F(1,2), F(2,0), F(2,1), F(2,2);
      return theta;
   };

   double scalar_triple_product(const Vector3d& a, const Vector3d& b, const Vector3d& c)
   {
      return a.dot(b.cross(c));
   };

   Matrix3d cross_product(const Vector3d& t, const Matrix3d& E)
   {
      Matrix3d Tx;

      Tx << 0.0, -t(2), t(1), t(2), 0.0, -t(0), -t(1), t(0), 0.0;
      return Tx * E;
   };
}

// Namespace TwoViewReconstruction
namespace TwoViewReconstruction {
   
   // constants for defualt values
   const double Default_f0 = 600.0;
   const int Max_Iteration = 30;
   const double Convergence_EPS = 1e-5;
   const double Large_Number = 1e99;
   const double Almost0 = 1e-8;

   // constant Vectors and Matrices
   const Vector3d ZeroVec3 = Vector3d::Zero();
   const Matrix3d I3 = Matrix3d::Identity();
   const Vector3d Vec_k(0.0, 0.0, 1.0);

   // focal length computation
   bool focal_length_computation(const Matrix3d& F,
                                 double* nf0,
                                 double* nf1,
                                 double f0 = Default_f0);
   
   // focal length computation (vector version)
   bool focal_length_computation(const Vector9d& theta,
                                 double* nf0,
                                 double* nf1,
                                 double f0 = Default_f0);
   
   // Motion parameter computation
   bool motion_parameter_computation(const Matrix3d& F,
                                     double nf0,
                                     double nf1,
                                     Vector2d pos0[],
                                     Vector2d pos1[],
                                     int Num,
                                     Matrix3d& R,
                                     Vector3d& t,
                                     double def_f0 = Default_f0);

   // Motion parameter computation: vector version
   bool motion_parameter_computation(const Vector9d& theta,
                                     double nf0,
                                     double nf1,
                                     Vector2d pos0[],
                                     Vector2d pos1[],
                                     int Num,
                                     Matrix3d& R,
                                     Vector3d& t,
                                     double def_f0 = Default_f0);

   // 3-D reconstruction from two views
   bool reconstruction(Vector2d pos0[],
                       Vector2d pos1[],
                       int Num,
                       const Vector9d& theta,
                       const Matrix3d& R,
                       const Vector3d& t,
                       double f,
                       double fp,
                       Vector3d X[],
                       int Max_Iter = Max_Iteration,
                       double Conv_EPS = Convergence_EPS,
                       double f0 = Default_f0);

   // 3-D reconstruction from two views (F matrix)
   bool reconstruction(Vector2d pos0[],
                       Vector2d pos1[],
                       int Num,
                       const Matrix3d& F,
                       const Matrix3d& R,
                       const Vector3d& t,
                       double f,
                       double fp,
                       Vector3d X[],
                       int Max_Iter = Max_Iteration,
                       double Conv_EPS = Convergence_EPS,
                       double f0 = Default_f0);

}

#endif // _TWO_VIEW_RECONSTRUCTION_H_
