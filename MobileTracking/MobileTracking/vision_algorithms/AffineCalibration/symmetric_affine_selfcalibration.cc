//
// Symmetric affine camera self calibration
//

#include "AffineSelfCalibration.h"

// Symmetric affine camera factorization method
bool
AffineSelfCalibration::symmetric_affine_selfcalibration(
   const MatrixXd pt0[],
   int  CamNum,
   int  PtNum,
   MatrixXd& M,
   MatrixXd& S)
{
   MatrixXd     ptnew[PtNum];
   double       A[CamNum], C[CamNum];
   MatrixXd     W, U, Sg, V;
   Vector3d     u1[CamNum], u2[CamNum];
   Matrix9d     B0;
   Matrix6d     B;
   Vector6d     tau;
   Matrix3d     T, Aff;

   // resize
   for (int i = 0; i < PtNum; i++)
      ptnew[i].resize(2,CamNum);

   // translate coordinates
   for (int kp = 0; kp < CamNum; kp++)
   {
      double tx, ty;

      // the center of the gravity
      center_of_gravity(pt0, PtNum, kp, &tx, &ty);

      // set coefficients
      A[kp] = tx * ty;
      C[kp] = sqr(tx) - sqr(ty);

      // translation with resize
      translation(pt0, ptnew, PtNum, kp, tx, ty);
   }

   // make observation matrix W
   W = observation_matrix(ptnew, CamNum, PtNum);

   // Singular value decomposition
   svd_decomp(W, U, Sg, V);

   // extract matrix U and set vectors u1 and u2
   extract_uvecs(U, CamNum, u1, u2);

   // compute matrix B
   for (int i = 0; i < 3; i++)
   {
      for (int j = 0; j < 3; j++)
      {
         int ij = i*3 + j;
         for (int k = 0; k < 3; k++)
         {
            for (int l = 0; l < 3; l++)
            {
               int kl = k*3 + l;

               B0(ij,kl) = 0.0;
               for (int kp = 0; kp < CamNum; kp++)
               {
                  double tt1, tt2, tt3;
                  tt1 = u1[kp](i)*u1[kp](j)*u1[kp](k)*u1[kp](l)
                     +  u2[kp](i)*u2[kp](j)*u2[kp](k)*u2[kp](l)
                     -  u1[kp](i)*u1[kp](j)*u2[kp](k)*u2[kp](l)
                     -  u2[kp](i)*u2[kp](j)*u1[kp](k)*u1[kp](l);
                  
                  tt2 = u1[kp](i)*u2[kp](j)*u1[kp](k)*u2[kp](l)
                     +  u2[kp](i)*u1[kp](j)*u1[kp](k)*u2[kp](l)
                     +  u1[kp](i)*u2[kp](j)*u2[kp](k)*u1[kp](l)
                     +  u2[kp](i)*u1[kp](j)*u2[kp](k)*u1[kp](l);
                  
                  tt3 = u1[kp](i)*u1[kp](j)*u1[kp](k)*u2[kp](l)
                     +  u1[kp](i)*u1[kp](j)*u2[kp](k)*u1[kp](l)
                     +  u1[kp](i)*u2[kp](j)*u1[kp](k)*u1[kp](l)
                     +  u2[kp](i)*u1[kp](j)*u1[kp](k)*u1[kp](l)
                     -  u1[kp](i)*u2[kp](j)*u2[kp](k)*u2[kp](l)
                     -  u2[kp](i)*u1[kp](j)*u2[kp](k)*u2[kp](l)
                     -  u2[kp](i)*u2[kp](j)*u1[kp](k)*u2[kp](l)
                     -  u2[kp](i)*u2[kp](j)*u2[kp](k)*u1[kp](l);
                     
                  B0(ij,kl) += sqr(A[kp])*tt1 + sqr(C[kp])*tt2/4.0
                     - A[kp]*C[kp]*tt3/2.0;
               }
            }
         }
      }
   }

   // set matrix B
   B = set_matrix_B(B0);

   // eigen values and eigen vectors
   SelfAdjointEigenSolver<Matrix6d> ES(B);

   // eigen vector for the smallest eigen value
   tau = ES.eigenvectors().col(0);
   T = convert_to_T(tau);

   // check sign
   if (T.determinant() < 0)  T = -T;
      
   // Cholesky decomposition
   LLT<Matrix3d> CholD(T);

   // Affine matrix
   Aff = CholD.matrixL();

   // Motion matrix
   M = U * Aff;

   // Shape matrix
   S = Aff.inverse() * Sg * V.transpose();

   return true;
}
