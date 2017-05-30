//
// Common functions for planar triangulation
//

#include "PlanarTriangulation.h"

using namespace Eigen;

// small functions 
// set xi1
Vector9d
PlanarTriangulation::set_xi1(const Vector2d& pos0, const Vector2d& pos1, double f0)
{
   Vector9d xi1;
   xi1 << 0.0, 0.0, 0.0, -f0*pos0(0), -f0*pos0(1), -sqr(f0),
      pos0(0)*pos1(1), pos0(1)*pos1(1), f0*pos1(1);
   return xi1;
}

// set xi2
Vector9d
PlanarTriangulation::set_xi2(const Vector2d& pos0, const Vector2d& pos1, double f0)
{
   Vector9d xi2;
   xi2 << f0*pos0(0), f0*pos0(1), sqr(f0), 0.0, 0.0, 0.0,
      -pos0(0)*pos1(0), -pos0(1)*pos1(0), -f0*pos1(0);
   return xi2;
}

// set xi3
Vector9d
PlanarTriangulation::set_xi3(const Vector2d& pos0, const Vector2d& pos1, double f0)
{
   Vector9d xi3;
   xi3 <<
      -pos0(0)*pos1(1), -pos0(1)*pos1(1), -f0*pos1(1),
      pos0(0)*pos1(0), pos0(1)*pos1(0), f0*pos1(0),
      0.0, 0.0, 0.0;
   return xi3;
}

// set xi1 from vec4
Vector9d
PlanarTriangulation::set_xi1(const Vector4d& pos, double f0)
{
   Vector9d xi1;
   xi1 << 0.0, 0.0, 0.0, -f0*pos(0), -f0*pos(1), -sqr(f0),
      pos(0)*pos(2), pos(1)*pos(3), f0*pos(3);
   return xi1;
}

// set xi2 from vec4
Vector9d
PlanarTriangulation::set_xi2(const Vector4d& pos, double f0)
{
   Vector9d xi2;
   xi2 << f0*pos(0), f0*pos(1), sqr(f0), 0.0, 0.0, 0.0,
      -pos(0)*pos(2), -pos(1)*pos(2), -f0*pos(2);
   return xi2;
}

// set xi3 from vec4
Vector9d
PlanarTriangulation::set_xi3(const Vector4d& pos, double f0)
{
   Vector9d xi3;
   xi3 <<
      -pos(0)*pos(3), -pos(1)*pos(3), -f0*pos(3),
      pos(0)*pos(2), pos(1)*pos(2), f0*pos(2),
      0.0, 0.0, 0.0;
   return xi3;
}

// set Ta1
void
PlanarTriangulation::set_Ta1(Matrix94d& Ta1, const Vector2d& pos0, const Vector2d& pos1, double f0)
{
   Ta1(3,0) = Ta1(4,1) = -f0;
   Ta1(6,0) = Ta1(7,1) = pos1(1);
   Ta1(6,3) = pos0(0);
   Ta1(7,3) = pos0(1);
   Ta1(8,3) = f0;
}

// set Ta2
void
PlanarTriangulation::set_Ta2(Matrix94d& Ta2, const Vector2d& pos0, const Vector2d& pos1, double f0)
{
   Ta2(0,0) = Ta2(1,1) = f0;
   Ta2(6,0) = Ta2(7,1) = -pos1(0);
   Ta2(6,2) = -pos0(0);
   Ta2(7,2) = -pos0(1);
   Ta2(8,2) = -f0;
}

// set Ta3
void
PlanarTriangulation::set_Ta3(Matrix94d& Ta3, const Vector2d& pos0, const Vector2d& pos1, double f0)
{
   Ta3(0,0) = Ta3(1,1) = -pos1(1);
   Ta3(0,3) = -pos0(0);
   Ta3(1,3) = -pos0(1);
   Ta3(2,3) = -f0;
   Ta3(3,0) = Ta3(4,1) = pos1(0);
   Ta3(3,2) = pos0(0);
   Ta3(4,2) = pos0(1);
   Ta3(5,2) = f0;
}

// set Ta1 from vec4
void
PlanarTriangulation::set_Ta1(Matrix94d& Ta1, const Vector4d& pos, double f0)
{
   Ta1(3,0) = Ta1(4,1) = -f0;
   Ta1(6,0) = Ta1(7,1) = pos(3);
   Ta1(6,3) = pos(0);
   Ta1(7,3) = pos(1);
   Ta1(8,3) = f0;
}

// set Ta2 from vec4
void
PlanarTriangulation::set_Ta2(Matrix94d& Ta2, const Vector4d& pos, double f0)
{
   Ta2(0,0) = Ta2(1,1) = f0;
   Ta2(6,0) = Ta2(7,1) = -pos(2);
   Ta2(6,2) = -pos(0);
   Ta2(7,2) = -pos(1);
   Ta2(8,2) = -f0;
}

// set Ta3 from vec4
void
PlanarTriangulation::set_Ta3(Matrix94d& Ta3, const Vector4d& pos, double f0)
{
   Ta3(0,0) = Ta3(1,1) = -pos(3);
   Ta3(0,3) = -pos(0);
   Ta3(1,3) = -pos(1);
   Ta3(2,3) = -f0;
   Ta3(3,0) = Ta3(4,1) = pos(2);
   Ta3(3,2) = pos(0);
   Ta3(4,2) = pos(1);
   Ta3(5,2) = f0;
}

// Rank constrained Generalized Inverse
template<class MatT>
MatT gen_inv_w_rank(const MatT& M, int r)
{
   if (M.cols() != M.rows()) abort(); // not squared

   int n = M.cols(); // set size
   if (r > n)  abort();

   // Eigen Solver
   SelfAdjointEigenSolver<MatT> ES(M);

   if (ES.info() != Success) abort(); // failed

   // Compute generalized inverse with rank r
   MatT GenI = MatT::Zero();
   for (int i = n - r; i < n; i++)
   {
      GenI += ES.eigenvectors().col(i) * ES.eigenvectors().col(i).transpose()
         / ES.eigenvalues()(i);
   }

   return GenI;
}

// Rank trancated genelralized inverse
Matrix9d
PlanarTriangulation::trancated_generalized_inverse(
   const Matrix9d& M,         // Matrix:   Input
   int rank             // rank:     Input
)
{
   // Eigen Solver for Self Adjoint Matrix
   SelfAdjointEigenSolver<Matrix9d> GES(M);

   Matrix9d Mi = ZeroMat9;
   for (int i = 9 - rank; i < 9; i++)
      Mi += GES.eigenvectors().col(i) * GES.eigenvectors().col(i).transpose()
         / GES.eigenvalues()(i);

   return Mi;
}

// Rank trancated genelralized inverse
Matrix3d
PlanarTriangulation::trancated_generalized_inverse(
   const Matrix3d& M,         // Matrix:   Input
   int rank             // rank:     Input
)
{
   // Eigen Solver for Self Adjoint Matrix
   SelfAdjointEigenSolver<Matrix3d> GES(M);

   Matrix3d Mi = ZeroMat3;
   
   for (int i = 3 - rank; i < 3; i++)
      Mi += GES.eigenvectors().col(i) * GES.eigenvectors().col(i).transpose()
         / GES.eigenvalues()(i);

   return Mi;
}
