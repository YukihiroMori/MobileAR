//
// Primary method of perspective camera self calibration
//

#include "PerspectiveSelfCalibration.h"

// Primary method: faster version
bool
PerspectiveSelfCalibration::faster_primary_method(
   const MatrixXd pt0[],
   int  CamNum,
   int  PtNum,
   Matrix34d Prj[],
   Vector4d X[],
   double rperr,
   double f0,
   int  MaxIter)
{
   MatrixXd     ptn[PtNum];
   MatrixXd     W, U, Sg, V;
   MatrixXd     Uc, Sgc, Vc;
   MatrixXd     u(3*CamNum,4);
   MatrixXd     C(CamNum,4);
   MatrixXd     z_ak(PtNum,CamNum);
   VectorXd     xi;
   double       E, E0, s;
   int          count;
   JacobiSVD<MatrixXd> SVD;

   // copy 2-vector to 3-vector and initialize z_ak to 1.0
   for (int al = 0; al < PtNum; al++)
   {
      ptn[al].resize(3,CamNum);
      for (int kp = 0; kp < CamNum; kp++)
      {
         ptn[al].col(kp)(0) = pt0[al].col(kp)(0) / f0;
         ptn[al].col(kp)(1) = pt0[al].col(kp)(1) / f0;
         ptn[al].col(kp)(2) = z_ak(al,kp) = 1.0; // z_ak = 1
      }
   }

   // iteration to minimize the reprojection errors
   count = 0;
   E = Large_Number;
   do
   {
      E0 = E;
      std::cerr << "***** loop: " << count;
      
      // make observation matrix W
      W = observation_matrix(ptn, z_ak, CamNum, PtNum);

      // normalization of each column vector
      normalize_each_column(W);

      // Singular value decomposition
      SVD.compute(W, ComputeThinU | ComputeThinV);

      // extract first four columns of U
      u = extract_4columns(SVD.matrixU());

      // set projection matrices
      set_projection_matrices(u, Prj, CamNum);

      for (int al = 0; al < PtNum; al++)
      {
         // set matrix C_{\kappa i}^\alpha
         for (int kp = 0; kp < CamNum; kp++)
         {
            Vector3d x_ak = ptn[al].col(kp);
            x_ak.normalize();
            
            for (int i = 0; i < 4; i++)
            {
               Vector3d uik = u.block<3,1>(kp*3,i);
               
               C(kp,i) = x_ak.dot(uik);
            }
         }

         // singular value decompositoin of C
         SVD.compute(C, ComputeThinU | ComputeThinV);
         Uc = SVD.matrixU();

         // eigen vector for the largest eigen value
         xi = Uc.col(0);

         // check sign
         s = 0.0;
         for (int kp = 0; kp < CamNum; kp++)
            s += xi(kp);
         if (s < 0.0)
            xi = -xi;

         // update projective depth
         for (int kp = 0; kp < CamNum; kp++)
         {
            Vector3d x_ak;

            x_ak = ptn[al].col(kp);
            z_ak(al,kp) = xi(kp) / x_ak.norm();
         }
      }

      // compute reconstructed points
      for (int al = 0; al < PtNum; al++)
      {
         VectorXd p_a = W.col(al);

         for (int i = 0; i < 4; i++)
            X[al](i) = p_a.dot(u.col(i));
      }

      // compute reprojection error
      s = 0.0;
      for (int al = 0; al < PtNum; al++)
         for (int kp = 0; kp < CamNum; kp++)
         {
            Vector3d   d, x_ak;

            x_ak = ptn[al].col(kp);
            d = x_ak - Znormalize(Prj[kp]*X[al]);
            s += d.squaredNorm();
         }
      E = f0 * sqrt(s / (double)(CamNum * PtNum));
      std::cerr << ", reprojection error = " << E
                << " (" << s << ")"
                << std::endl;
   }
   while (fabs(E0 - E) >= rperr && ++count < MaxIter);

   if (count >= MaxIter)
   {
      std::cerr << "faster_primary_method do not converge!" << std::endl;
      return false;
   }
            
   return true;
}


// Primary method: original version
bool
PerspectiveSelfCalibration::original_primary_method(
   const MatrixXd pt0[],
   int  CamNum,
   int  PtNum,
   Matrix34d Prj[],
   Vector4d X[],
   double rperr,
   double f0,
   int  MaxIter)
{
   MatrixXd     ptn[PtNum];
   MatrixXd     M, S;
   MatrixXd     W, U, Sg, V;
   MatrixXd     u(3*CamNum,4);
   MatrixXd     A(CamNum,CamNum);
   MatrixXd     z_ak(PtNum,CamNum);
   VectorXd     xi;
   double       E, E0, s;
   int          count;

   // copy 2-vector to 3-vector and initialize z_ak to 1.0
   for (int al = 0; al < PtNum; al++)
   {
      ptn[al].resize(3,CamNum);
      for (int kp = 0; kp < CamNum; kp++)
      {
         ptn[al].col(kp)(0) = pt0[al].col(kp)(0) / f0;
         ptn[al].col(kp)(1) = pt0[al].col(kp)(1) / f0;
         ptn[al].col(kp)(2) = z_ak(al,kp) = 1.0; // z_ak = 1
      }
   }

   // iteration to minimize the reprojection errors
   count = 0;
   E = Large_Number;
   do
   {
      E0 = E;
      std::cerr << "***** loop: " << count;
      
      // make observation matrix W
      W = observation_matrix(ptn, z_ak, CamNum, PtNum);

      // normalization of each column vector
      normalize_each_column(W);

      // Singular value decomposition
      svd_decomp(W, U, Sg, V);
      M = U;
      S = Sg * V.transpose();

      // extract first four columns of U
      u = extract_4columns(U);

      for (int al = 0; al < PtNum; al++)
      {
         for (int kp = 0; kp < CamNum; kp++)
         {
            double x_ak_nrm;
            Vector3d x_ak;
            
            x_ak = ptn[al].col(kp);
            x_ak_nrm = x_ak.norm();
            
            for (int lmd = 0; lmd < CamNum; lmd++)
            {
               Vector3d u_ik, u_il, x_al;
               double x_al_nrm;

               x_al = ptn[al].col(lmd);
               x_al_nrm = x_al.norm();

               s = 0.0;
               for (int i = 0; i < 4; i++)
               {
                  u_ik = u.block<3,1>(kp*3,i);
                  u_il = u.block<3,1>(lmd*3,i);
                  s += x_ak.dot(u_ik) * x_al.dot(u_il);
               }
               A(kp,lmd) = s / (x_ak_nrm * x_al_nrm);
            }
         }

         // compute eigen values and vectors
         SelfAdjointEigenSolver<MatrixXd> ES(A);

         // eigen vector for the largest eigen value
         xi = ES.eigenvectors().col(CamNum-1);

         // check sign
         s = 0.0;
         for (int kp = 0; kp < CamNum; kp++)
            s += xi(kp);
         if (s < 0.0)
            xi = -xi;

         // update projective depth
         for (int kp = 0; kp < CamNum; kp++)
         {
            Vector3d x_ak;

            x_ak = ptn[al].col(kp);
            z_ak(al,kp) = xi(kp) / x_ak.norm();
         }
      }

      // Compute projection matrices and 3-D points
      for (int kp = 0; kp < CamNum; kp++)
         Prj[kp] = M.block<3,4>(kp*3,0);

      // extract each reconstructed point
      for (int al = 0; al < PtNum; al++)
         X[al] = S.block<4,1>(0,al);

      // compute reprojection error
      s = 0.0;
      for (int al = 0; al < PtNum; al++)
         for (int kp = 0; kp < CamNum; kp++)
         {
            Vector3d   d, x_ak;

            x_ak = ptn[al].col(kp);
            d = x_ak - Znormalize(Prj[kp]*X[al]);
            s += d.squaredNorm();
         }
      E = f0 * sqrt(s / (double)(CamNum * PtNum));
      std::cerr << ", reprojection error = " << E
                << " (" << s << ")"
                << std::endl;
   }
   while (fabs(E0 - E) >= rperr && ++count < MaxIter);

   if (count >= MaxIter)
   {
      std::cerr << "original_primary_method do not converge!" << std::endl;
      return false;
   }
            
   return true;
}
