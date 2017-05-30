//
// Common functions for affine self-calibration
//

#include "AffineSelfCalibration.h"

using namespace Eigen;

// small functions

// extract 3 columns from matrix
MatrixXd
AffineSelfCalibration::left3cols(const MatrixXd& U0)
{
   MatrixXd U(U0.rows(),3);

   for (int r = 0; r < U0.rows(); r++)
      U.row(r) = U0.block<1,3>(r,0);

   return U;
}

// extract upper left 3x3 matrix from matrix
inline Matrix3d
AffineSelfCalibration::upperleft3x3(const MatrixXd& Sg0)
{
   return Sg0.block<3,3>(0,0);
}

// compute the center of the gravity
Vector2d
AffineSelfCalibration::center_of_gravity(
   const MatrixXd pt0[],
   int PtNum,
   int cam)
{
   Vector2d     g;
   double       gx, gy;

   gx = gy = 0.0;
   for (int pt = 0; pt < PtNum; pt++)
   {
      gx += pt0[pt].col(cam)(0);
      gy += pt0[pt].col(cam)(1);
   }

   g << gx / (double)PtNum, gy / (double)PtNum;

   return g;
}

void
AffineSelfCalibration::center_of_gravity(
   const MatrixXd pt0[],
   int PtNum,
   int cam,
   double *tx,
   double *ty)
{
   double       gx, gy;

   gx = gy = 0.0;
   for (int pt = 0; pt < PtNum; pt++)
   {
      gx += pt0[pt].col(cam)(0);
      gy += pt0[pt].col(cam)(1);
   }

   *tx = gx / (double)PtNum;
   *ty = gy / (double)PtNum;
}

// translate the origin to the center of the gravity
void
AffineSelfCalibration::translation(
   const MatrixXd pt0[],
   MatrixXd ptt[],
   int PtNum,
   int cam,
   const Vector2d& gp)
{
   for (int pt = 0; pt < PtNum; pt++)
      ptt[pt].col(cam) = pt0[pt].col(cam) - gp;
}

void
AffineSelfCalibration::translation(
   const MatrixXd pt0[],
   MatrixXd ptt[],
   int PtNum,
   int cam,
   double tx,
   double ty)
{
   for (int pt = 0; pt < PtNum; pt++)
   {
      ptt[pt].col(cam)(0) = pt0[pt].col(cam)(0) - tx;
      ptt[pt].col(cam)(1) = pt0[pt].col(cam)(1) - ty;
   }
}

// make observationt matrix W
MatrixXd
AffineSelfCalibration::observation_matrix(
   const MatrixXd x[],
   int CamNum,
   int PtNum)
{
   MatrixXd W(2*CamNum, PtNum);

   for (int cam = 0; cam < CamNum; cam++)
   {
      int row1, row2;
      
      row1 = cam * 2;
      row2 = cam * 2 + 1;
      for (int pt = 0; pt < PtNum; pt++)
      {
         W(row1, pt) = x[pt].col(cam)(0);
         W(row2, pt) = x[pt].col(cam)(1);
      }
   }
   
   return W;
}

// Singular value decomposition of observation_matrix
void
AffineSelfCalibration::svd_decomp(const MatrixXd& W,
                                  MatrixXd& U,
                                  MatrixXd& Sg,
                                  MatrixXd& V)
{
   MatrixXd     Up, Sgp, Vp;
   
   // Singular value decomposition
   JacobiSVD<MatrixXd> SVD(W, ComputeThinU | ComputeThinV);

   // Matrix U
   Up = SVD.matrixU();
   U = left3cols(Up);

   // Matrix Sigma
   Sgp = SVD.singularValues().asDiagonal();
   Sg = upperleft3x3(Sgp);

   // Matrix V
   Vp = SVD.matrixV();
   V = left3cols(Vp);
}

// extract matrix U and set vectors u1 and u2
void
AffineSelfCalibration::extract_uvecs(const MatrixXd& U,
                                     int CamNum,
                                     Vector3d u1[],
                                     Vector3d u2[])
{
   for (int kp = 0; kp < CamNum; kp++)
   {
      u1[kp] = U.row(2*kp).transpose();
      u2[kp] = U.row(2*kp+1).transpose();
   }
}
