//
// Reconstruction of supporting plane of circle
//

#include <iostream>
#include "EllipseAnalysis.h"

using namespace Eigen;

// reconstruct the supporting plane of a circle
bool
EllipseAnalysis::supporting_plane_reconstruction(
   const Matrix3d& Q,   // Ellipse parameter:                INPUT
   double radius,       // Radius of the circle:             INPUT
   Vector3d& normal,    // Normal of the circle:             OUTPUT
   double *dist,        // Distance to the supporting plane: OUTPUT
   double f,            // Focal length:                     INPUT
   double f0            // Default focal length:             INPUT w. default
)
{
   Matrix3d     Qb;
   DiagonalMatrix<double,3> D(1.0/f0, 1.0/f0, 1.0/f);
   Vector3d     u1, u2, u3;
   double       lmd1, lmd2, lmd3;
   double       detQ;

   // compute \bar{Q}
   Qb = D * Q * D;
   detQ = Qb.determinant();

   if (detQ < 0.0)
      Qb /= cbrt(-detQ);
   else
      Qb /= -cbrt(detQ);
      
   // Eigenvalues of \bar{Q}
   SelfAdjointEigenSolver<Matrix3d> ES(Qb);
   if (ES.info() != Success)
      return false;

   // set eigenvalues and vectors as lmd2 >= lmd1 > 0 > lmd3
   lmd3 = ES.eigenvalues()(0); u3 = ES.eigenvectors().col(0);
   lmd1 = ES.eigenvalues()(1); u1 = ES.eigenvectors().col(1);
   lmd2 = ES.eigenvalues()(2); u2 = ES.eigenvectors().col(2);
   
   // normal of the supporting plane
//   normal = sqrt(lmd2 - lmd1) * u2 + sqrt(lmd1 - lmd3) * u3;
   // The following vector is also the normal of the plane
   normal = sqrt(lmd2 - lmd1) * u2 - sqrt(lmd1 - lmd3) * u3;
   normal.normalize();

   // distance from the supporting plane
   *dist = sqrt(lmd1*lmd1*lmd1) * radius;

   return true;
}
