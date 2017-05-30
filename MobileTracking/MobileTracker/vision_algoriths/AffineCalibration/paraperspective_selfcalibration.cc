//
// Paraperspective camera self calibration
//

#include "AffineSelfCalibration.h"

// Paraperspective factorization
bool
AffineSelfCalibration::paraperspective_selfcalibration(
   const MatrixXd pt0[],
   const double fl0[],
   int  CamNum,
   int  PtNum,
   MatrixXd& M,
   MatrixXd& S)
{
   MatrixXd     ptnew[PtNum];
   double       alpha[CamNum], beta[CamNum], gamma[CamNum];
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
      double    tx, ty;
      
      // the center of the gravity
      center_of_gravity(pt0, PtNum, kp, &tx, &ty);

      // set alphas, betas, and gammas
      alpha[kp] = 1.0 / (1.0 + sqr(tx)/sqr(fl0[kp]));
      beta[kp]  = 1.0 / (1.0 + sqr(ty)/sqr(fl0[kp]));
      gamma[kp] = tx * ty / sqr(fl0[kp]);

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
                  double gmp1, gmm1, al2, bt2, algm, btgm, albt;
                  gmp1 = sqr(gamma[kp]) + 1.0;
                  gmm1 = sqr(gamma[kp]) - 1.0;
                  al2 = sqr(alpha[kp]);
                  bt2 = sqr(beta[kp]);
                  algm = alpha[kp] * gamma[kp];
                  btgm = beta[kp] * gamma[kp];
                  albt = alpha[kp] * beta[kp];

                  B0(ij,kl) +=
                       gmp1*al2*u1[kp](i)*u1[kp](j)*u1[kp](k)*u1[kp](l)
                     + gmp1*bt2*u2[kp](i)*u2[kp](j)*u2[kp](k)*u2[kp](l)
                     + u1[kp](i)*u2[kp](j)*u1[kp](k)*u2[kp](l)
                     + u1[kp](i)*u2[kp](j)*u2[kp](k)*u1[kp](l)
                     + u2[kp](i)*u1[kp](j)*u1[kp](k)*u2[kp](l)
                     + u2[kp](i)*u1[kp](j)*u2[kp](k)*u1[kp](l)
                     - algm*u1[kp](i)*u1[kp](j)*u1[kp](k)*u2[kp](l)
                     - algm*u1[kp](i)*u1[kp](j)*u2[kp](k)*u1[kp](l)
                     - algm*u1[kp](i)*u2[kp](j)*u1[kp](k)*u1[kp](l)
                     - algm*u2[kp](i)*u1[kp](j)*u1[kp](k)*u1[kp](l)
                     - btgm*u2[kp](i)*u2[kp](j)*u1[kp](k)*u2[kp](l)
                     - btgm*u2[kp](i)*u2[kp](j)*u2[kp](k)*u1[kp](l)
                     - btgm*u1[kp](i)*u2[kp](j)*u2[kp](k)*u2[kp](l)
                     - btgm*u2[kp](i)*u1[kp](j)*u2[kp](k)*u2[kp](l)
                     + gmm1*albt*u1[kp](i)*u1[kp](j)*u2[kp](k)*u2[kp](l)
                     + gmm1*albt*u2[kp](i)*u2[kp](j)*u1[kp](k)*u1[kp](l);
               }
            }
         }
      }
   }

   // set 6x6 matrix
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
