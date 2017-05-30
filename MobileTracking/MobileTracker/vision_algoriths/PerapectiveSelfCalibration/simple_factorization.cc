//
// Simple factorization method for perspective camera
//
#include "PerspectiveSelfCalibration.h"

// pure function of simple factorization method
bool
PerspectiveSelfCalibration::pure_factorization(
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
PerspectiveSelfCalibration::simple_factorization(
   const MatrixXd pt0[],
   int  CamNum,
   int  PtNum,
   Matrix34d Prj[],
   Vector4d X[],
   double f0)
{
   MatrixXd     ptn[PtNum];
   MatrixXd     W, U, Sg, V, M, S;

   // copy 2-vector to 3-vector
   for (int al = 0; al < PtNum; al++)
   {
      ptn[al].resize(3,CamNum);
      for (int kp = 0; kp < CamNum; kp++)
      {
         ptn[al].col(kp)(0) = pt0[al].col(kp)(0);
         ptn[al].col(kp)(1) = pt0[al].col(kp)(1);
         ptn[al].col(kp)(2) = 1.0; // z_ak = 1
      }
   }
   std::cerr << "*** Enhance 2-vector to 3-vector done!" << std::endl;
   
   // make observation matrix W
   W = observation_matrix(ptn, CamNum, PtNum, f0);
   std::cerr << "W = " << W.rows() << "x" << W.cols() << "\n"
             << W << std::endl;

   // call common function
   svd_decomp(W, U, Sg, V);

   // Motion matrix
   M = U;
   std::cerr << "M = " << M.rows() << "x" << M.cols() << "\n"
             << M << std::endl;

   // Set shape matrix S
   S = Sg * V.transpose();
   std::cerr << "S = " << S.rows() << "x" << S.cols() << "\n"
             << S << std::endl;

   // extract each projection matrix and coordinates
   for (int k = 0; k < CamNum; k++)
      Prj[k] = M.block<3,4>(k*3,0);

      // extract each reconstructed point
   for (int p = 0; p < PtNum; p++)
      X[p] = S.block<4,1>(0,p);

   return true;
}
