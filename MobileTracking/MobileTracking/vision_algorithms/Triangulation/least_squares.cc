//
// Triangulation using least squares
//

#include "Triangulation.hpp"

using namespace Eigen;

// unnamed namespace for local functions
namespace {
// make Matrix T
inline Matrix43d
make_matrix_T(const Matrix34d& P0, const Matrix34d& P1,
              const Vector2d& p0, const Vector2d& p1,
              double f0)
{
   Matrix43d    T;

   T <<
      f0*P0(0,0)-p0(0)*P0(2,0), f0*P0(0,1)-p0(0)*P0(2,1), f0*P0(0,2)-p0(0)*P0(2,2), 
      f0*P0(1,0)-p0(1)*P0(2,0), f0*P0(1,1)-p0(1)*P0(2,1), f0*P0(1,2)-p0(1)*P0(2,2), 
      f0*P1(0,0)-p1(0)*P1(2,0), f0*P1(0,1)-p1(0)*P1(2,1), f0*P1(0,2)-p1(0)*P1(2,2), 
      f0*P1(1,0)-p1(1)*P1(2,0), f0*P1(1,1)-p1(1)*P1(2,1), f0*P1(1,2)-p1(1)*P1(2,2);

   return T;
}

// make vector p
inline Vector4d
make_vector_p(const Matrix34d& P0, const Matrix34d& P1,
              const Vector2d& p0, const Vector2d& p1,
              double f0)
{
   Vector4d     p;

   p <<
      f0*P0(0,3) - p0(0)*P0(2,3),
      f0*P0(1,3) - p0(1)*P0(2,3),
      f0*P1(0,3) - p1(0)*P1(2,3),
      f0*P1(1,3) - p1(1)*P1(2,3);

   return p;
}

}
// end of unnamed namespace

// Triangulation using least squares from two camera matrices
Vector3d
Triangulation::least_squares(
   const Vector2d& pos0,
   const Vector2d& pos1,
   const Matrix34d& P0,
   const Matrix34d& P1,
   double    f0)
{
   Matrix43d    T;
   Vector4d     p;
   Vector3d     mTtp;
   Matrix3d     TtT;
   Vector3d     X;

   // constructer of LU
   FullPivLU<Matrix3d> LU;

   T = make_matrix_T(P0, P1, pos0, pos1, f0);
   p = make_vector_p(P0, P1, pos0, pos1, f0);
   TtT = T.transpose() * T;
   mTtp = - T.transpose() * p;

   // compute least squares solution by LU decomposition
   LU.compute(TtT);
   X = LU.solve(mTtp);

   return X;
}

// Triangulation using least squares from two camera matrices
bool
Triangulation::least_squares(
   std::vector<Vector2d> pos0,
   std::vector<Vector2d> pos1,
   int      Num,
   const Matrix34d& P0,
   const Matrix34d& P1,
   std::vector<Vector3d> X,
   double    f0)
{
   for (int al = 0; al < Num; al++)
      X[al] = least_squares(pos0[al], pos1[al], P0, P1, f0);

   return true;
}
