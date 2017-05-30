//
// Orthographic camera self calibration
//

#include "AffineSelfCalibration.h"

// Orthographic factorization
bool
AffineSelfCalibration::orthographic_selfcalibration(
   const MatrixXd pt0[],
   int  CamNum,
   int  PtNum,
   MatrixXd& M,
   MatrixXd& S)
{
   MatrixXd     ptnew[PtNum];
   MatrixXd     W, U, Sg, V;
   Vector3d     u1[CamNum], u2[CamNum];
   Matrix9d     B0;
   Matrix6d     B;
   Vector6d     tau, vb;
   Matrix3d     T, Aff;

   // resize
   for (int i = 0; i < PtNum; i++)
      ptnew[i].resize(2,CamNum);

   // translate coordinates
   for (int kp = 0; kp < CamNum; kp++)
   {
      double    tx, ty;
      
      // the center of the gravity
      center_of_gravity(pt0, PtNum, kp, &tx, &ty);

      // translation
      translation(pt0, ptnew, PtNum, kp, tx, ty);
   }

   // make observation matrix W
   W = observation_matrix(ptnew, CamNum, PtNum);

   // Singular value decomposition
   svd_decomp(W, U, Sg, V);

   // extract matrix U and set vectors u1 and u2
   extract_uvecs(U, CamNum, u1, u2);

   // compute matrix B0
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
                  B0(ij,kl) +=
                       u1[kp](i)*u1[kp](j)*u1[kp](k)*u1[kp](l)
                     + u2[kp](i)*u2[kp](j)*u2[kp](k)*u2[kp](l)
                     + (u1[kp](i)*u2[kp](j) + u2[kp](i)*u1[kp](j))
                     * (u1[kp](k)*u2[kp](l) + u2[kp](k)*u1[kp](l))/4.0;
               }
            }
         }
      }
   }

   // set 6x6 matrix
   B = set_matrix_B(B0);

   // solve B * tau = b
   vb << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
   FullPivLU<Matrix6d> lu(B);
   tau = lu.solve(vb);

   // set matrix T from tau
   T = convert_to_T(tau);

   // check sign
   if (T.determinant() < 0)  T = -T;
      
   // Affine matrix by Cholesky decomposition
   LLT<Matrix3d> CholD(T);
   Aff = CholD.matrixL();

   // Motion matrix
   M = U * Aff;

   // Shape matrix
   S = Aff.inverse() * Sg * V.transpose();

   return true;
}
