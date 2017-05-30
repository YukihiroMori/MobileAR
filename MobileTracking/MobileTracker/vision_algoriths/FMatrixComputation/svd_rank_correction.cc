//
// Rank Correction by SVD
//

#include "FMatrixComputation.h"

using namespace Eigen;

// Rank correction of fundamental matrix by SVD
// Matrix form
Matrix3d
FMatrixComputation::svd_rank_correction(
   const Matrix3d &F
)
{
   double       s1, s2, t;
   Matrix3d     Fn;
   
   // SVD by Jacobi method
   JacobiSVD<Matrix3d> SVD(F, ComputeFullU | ComputeFullV);

   // Singular Values
   s1 = SVD.singularValues()(0);
   s2 = SVD.singularValues()(1);
   t = sqrt(sqr(s1) + sqr(s2));

   // Diagonal Matrix
   Vector3d S(s1/t, s2/t, 0);

   // Rank Corrected Matrix
   Fn = SVD.matrixU() * S.asDiagonal() * SVD.matrixV().transpose();

   return Fn;
}

// Rank correction of fundamental matrix by SVD
// Vector form
Vector9d
FMatrixComputation::svd_rank_correction(
   const Vector9d &theta
)
{
   double       s1, s2, t;
   Vector9d     ntheta;
   Matrix3d     F, Fn;

   // convert vector to matrix
   F = convert_mat3(theta);

   Fn = svd_rank_correction(F);

   // convert matrix to vector
   ntheta = convert_vec9(Fn);

   return ntheta;
}
