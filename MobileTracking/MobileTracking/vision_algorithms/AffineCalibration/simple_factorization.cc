//
// Simple factorization method for affine camera
//
#include "AffineSelfCalibration.h"

// pure function of simple factorization method
bool
AffineSelfCalibration::pure_factorization(
   const MatrixXd& W,
   MatrixXd& M,
   MatrixXd& S)
{
   MatrixXd U, Sg, V;
   
   // call common function
   svd_decomp(W, U, Sg, V);

   // Motion matrix
   M = U;

   // Set shape matrix S
   S = Sg * V.transpose();

   return true;
}

// function of simple factorization method
bool
AffineSelfCalibration::simple_factorization(
   const MatrixXd pt0[],
   int  CamNum,
   int  PtNum,
   MatrixXd& M,
   MatrixXd& S)
{
   MatrixXd     ptnew[PtNum];
   MatrixXd     W, U, Sg, V;

   // resize
   for (int i = 0; i < PtNum; i++)
      ptnew[i].resize(2,CamNum);

   // translate coordinates
   for (int kp = 0; kp < CamNum; kp++)
   {
      double tx, ty;

      // the center of the gravity
      center_of_gravity(pt0, PtNum, kp, &tx, &ty);

      // translation with resize
      translation(pt0, ptnew, PtNum, kp, tx, ty);
   }

   // make observation matrix W
   W = observation_matrix(ptnew, CamNum, PtNum);

   // call common function
   svd_decomp(W, U, Sg, V);

   // Motion matrix
   M = U;

   // Set shape matrix S
   S = Sg * V.transpose();

   return true;
}
