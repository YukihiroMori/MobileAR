//
// Focal length computation from Fundamental Matrix
//

#include "TwoViewReconstruction.hpp"

using namespace Eigen;

// focal length computation
bool
TwoViewReconstruction::focal_length_computation(const Matrix3d& F,
                                                double* nf0,
                                                double* nf1,
                                                double f0
)
{
   Vector3d     e0, e1;
   double       xi, eta;
   Vector3d     Fk, Ftk, e1k, e0k;
   double       kFk, Fk2nrm, Ftk2nrm, e1k2nrm, e0k2nrm, kFFtFk;

   // Construct of Eigen Solver
   SelfAdjointEigenSolver<Matrix3d> ES;

   // Compute epipole e0
   ES.compute(F*F.transpose());
   e0 = ES.eigenvectors().col(0);

   // Compute epipole e1
   ES.compute(F.transpose()*F);
   e1 = ES.eigenvectors().col(0);

   // compute xi and eta
   Fk = F * Vec_k;
   Ftk = F.transpose() * Vec_k;
   kFk = Vec_k.dot(Fk);
   Fk2nrm = Fk.squaredNorm();
   Ftk2nrm = Ftk.squaredNorm();
   e1k2nrm = (e1.cross(Vec_k)).squaredNorm();
   e0k2nrm = (e0.cross(Vec_k)).squaredNorm();
   kFFtFk = Vec_k.dot(F*F.transpose()*Fk);

   xi  = (Fk2nrm - kFFtFk*e1k2nrm/kFk) / (e1k2nrm*Ftk2nrm - sqr(kFk));
   eta = (Ftk2nrm - kFFtFk*e0k2nrm/kFk) / (e0k2nrm*Fk2nrm - sqr(kFk));

   // imaginary focal length check
   if (xi < -1.0 || eta < -1.0)
      return false;
   
   // compute focal lengths
   *nf0 = f0 / sqrt(1.0 + xi);
   *nf1 = f0 / sqrt(1.0 + eta);

   return true;
}

// focal length computation (vector version)
bool
TwoViewReconstruction::focal_length_computation(const Vector9d& theta,
                                                double* nf0,
                                                double* nf1,
                                                double f0
)
{
   Matrix3d     F = convert_mat3(theta);

   return focal_length_computation(F, nf0, nf1, f0);
}
