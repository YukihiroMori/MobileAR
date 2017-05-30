//
// Motion parameter computation
//

#include "TwoViewReconstruction.hpp"

using namespace Eigen;

// Motion parameter computation
bool
TwoViewReconstruction::motion_parameter_computation(const Matrix3d& F,
                                                    double nf0,
                                                    double nf1,
                                                    std::vector<Vector2d> pos0,
                                                    std::vector<Vector2d> pos1,
                                                    int Num,
                                                    Matrix3d& R,
                                                    Vector3d& t,
                                                    double def_f0
)
{
   Matrix3d     E, K;
   Vector3d     x0, x1;
   double       J;

   // Diagonal Matrices
   DiagonalMatrix<double,3> Df(1.0/def_f0, 1.0/def_f0, 1.0/nf0);
   DiagonalMatrix<double,3> Db(1.0/def_f0, 1.0/def_f0, 1.0/nf1);
   
   // Set Essential Matrix
   E = Df * F * Db;

   // Compute the translation from the essential matrix
   SelfAdjointEigenSolver<Matrix3d> ES(E*E.transpose());
   t = ES.eigenvectors().col(0);

   // check sign of translation
   J = 0.0;
   for (int al = 0; al < Num; al++)
   {
      x0 << pos0[al](0) / nf0, pos0[al](1) / nf0, 1.0;
      x1 << pos1[al](0) / nf1, pos1[al](1) / nf1, 1.0;
      J += scalar_triple_product(t, x0, E*x1);
   }
   if (J < 0) t = -t;

   // Compute matrix K
   K = - cross_product(t, E);

   // SVD of matrix K
   JacobiSVD<Matrix3d> SVD(K, ComputeFullU | ComputeFullV);
   Matrix3d UVt = SVD.matrixU() * SVD.matrixV().transpose();
   DiagonalMatrix<double,3> DD(1.0, 1.0, UVt.determinant());

   // compute the rotation R
   R = SVD.matrixU() * DD * SVD.matrixV().transpose();

   return true;
}

// Motion parameter computation: vector version
bool
TwoViewReconstruction::motion_parameter_computation(const Vector9d& theta,
                                                    double nf0,
                                                    double nf1,
                                                    std::vector<Vector2d> pos0,
                                                    std::vector<Vector2d> pos1,
                                                    int Num,
                                                    Matrix3d& R,
                                                    Vector3d& t,
                                                    double def_f0
)
{
   Matrix3d     F = convert_mat3(theta);

   return motion_parameter_computation(F, nf0, nf1, pos0, pos1, Num,
                                       R, t, def_f0);
}
