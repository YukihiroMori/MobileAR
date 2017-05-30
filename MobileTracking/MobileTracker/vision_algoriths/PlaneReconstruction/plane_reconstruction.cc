//
// Compute plane and motion parameters from homography
//

#include "PlaneReconstruction.h"

typedef DiagonalMatrix<double,3> DiagMatrix3d;

bool
PlaneReconstruction::plane_reconstruction(
   const Matrix3d& H,
   double fl0,
   double fl1,
   Vector3d& np,
   Vector3d& nm,
   double *dist,
   Vector3d& tp,
   Vector3d& tm,
   Matrix3d& Rp,
   Matrix3d& Rm,
   double f0)
{
   DiagMatrix3d Pfp(f0, f0, fl1);
   DiagMatrix3d Pfi(1.0/f0, 1.0/f0, 1.0/fl0);
   Matrix3d Hh;
   Vector3d v1, v2, v3;
   double sg1, sg2, sg3, tt12, tt23;

   // remove f0 and normalize H
   Hh = Pfp * H * Pfi;
   Hh /= my_cbrt(Hh.determinant());

   // SVD
   JacobiSVD<Matrix3d> SVD(Hh, ComputeFullU | ComputeFullV);

   sg1 = SVD.singularValues()(0);
   sg2 = SVD.singularValues()(1);
   sg3 = SVD.singularValues()(2);
   v1 = SVD.matrixV().col(0);
   v3 = SVD.matrixV().col(2);

   // normal vector of the plane
   tt12 = sqrt(sg1*sg1 - sg2*sg2);
   tt23 = sqrt(sg2*sg2 - sg3*sg3);

   // plus
   np = tt12*v1 + tt23*v3;
   np.normalize();
   // minus
   nm = tt12*v1 - tt23*v3;
   nm.normalize();

   // distance to the plane
   *dist = sg2 / (sg1 - sg3);

   // translation: plus
   tp = -sg3*tt12*v1 + sg1*tt23*v3;
   tp.normalize();
   // minus
   tm = -sg3*tt12*v1 - sg1*tt23*v3;
   tm.normalize();

   // rotation: plus
   Rp = (I3 + sg2*sg2 * np * tp.transpose() / *dist) * Hh.transpose() / sg2;
   // minus
   Rm = (I3 + sg2*sg2 * nm * tm.transpose() / *dist) * Hh.transpose() / sg2;
}
