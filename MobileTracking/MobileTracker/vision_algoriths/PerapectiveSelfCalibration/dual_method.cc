//
// Dual method of perspective camera self calibration
//

#include "PerspectiveSelfCalibration.h"

// Dual method: original version
bool
PerspectiveSelfCalibration::original_dual_method(
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
   MatrixXd     v(3*CamNum,4);
   MatrixXd     B(PtNum,PtNum);
   MatrixXd     z_ak(PtNum,CamNum);
   VectorXd     xi;
   double       E, E0, s;
   int          count;
   JacobiSVD<MatrixXd> SVD;

   // copy 2-vector to 3-vector and initialize z_ak to 1.0
   for (int al = 0; al < PtNum; al++)
   {
      ptn[al].resize(3, CamNum);
      for (int kp = 0; kp < CamNum; kp++)
      {
         ptn[al].col(kp)(0) = pt0[al].col(kp)(0) / f0;
         ptn[al].col(kp)(1) = pt0[al].col(kp)(1) / f0;
         ptn[al].col(kp)(2) = z_ak(al,kp) = 1.0; // z_ak = 1
//         
//         double nrm = ptn[al].col(kp).norm();
//         ptn[al].col(kp) /= nrm;
      }
   }

   // make observation matrix W
   W = observation_matrix(ptn, z_ak, CamNum, PtNum);

   // iteration to minimize the reprojection errors
   E = Large_Number;
   count = 0;
   do
   {
      E0 = E;
      std::cerr << "***** loop: " << count;

      // normalization of each column vector
      normalize_each_3rows(W);

      // Singular value decomposition
      SVD.compute(W, ComputeThinU | ComputeThinV);

      // extract first four columns of V
      v = extract_4columns(SVD.matrixV());

      // set projection matrices
      set_projection_matrices(W, v, Prj, CamNum);

      for (int kp = 0; kp < CamNum; kp++)
      {
         for (int al = 0; al < PtNum; al++)
         {
            Vector3d xa;
            Vector4d va;
            
            xa = ptn[al].col(kp);
            xa /= xa.norm();
            va = v.row(al).transpose();
                 
            for (int bt = al; bt < PtNum; bt++)
            {
               Vector3d xb;
               Vector4d vb;

               xb = ptn[bt].col(kp);
               xb /= xb.norm();
               vb = v.row(bt).transpose();

               B(al,bt) = B(bt,al) = va.dot(vb) * xa.dot(xb);
            }
         }

         // compute eigen values and vectors
         SelfAdjointEigenSolver<MatrixXd> ES(B);

         // eigen vector for the largest eigen value
         xi = ES.eigenvectors().col(PtNum-1);

         // check sign
         s = 0.0;
         for (int al = 0; al < PtNum; al++)
            s += xi(al);
         if (s < 0.0)
            xi = -xi;

         // update projective depth
         for (int al = 0; al < PtNum; al++)
         {
            Vector3d xa;

            xa = ptn[al].col(kp);
            z_ak(al,kp) = xi(al) / xa.norm();
         }
      }

      // make observation matrix W
      W = observation_matrix(ptn, z_ak, CamNum, PtNum);

      // extract each reconstructed point
      for (int al = 0; al < PtNum; al++)
         X[al] = v.transpose().col(al);
      
      // set projection matrices
      set_projection_matrices(W, v, Prj, CamNum);
      
      // compute reprojection error
      s = 0.0;
      for (int al = 0; al < PtNum; al++)
         for (int kp = 0; kp < CamNum; kp++)
         {
            Vector3d   d, x_ak;

            x_ak = Znormalize(ptn[al].col(kp));
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
      std::cerr << "original_dual_method do not converge!" << std::endl;
      return false;
   }
            
   return true;
}


// Dual method: faster version
bool
PerspectiveSelfCalibration::faster_dual_method(
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
   MatrixXd     v(3*CamNum,4);
   MatrixXd     C(PtNum,12), C1(PtNum,4), C2(PtNum,4), C3(PtNum,4);
   MatrixXd     z_ak(PtNum,CamNum);
   VectorXd     xi;
   double       E, E0, s;
   int          count;
   JacobiSVD<MatrixXd> SVD;

   // copy 2-vector to 3-vector and initialize z_ak to 1.0
   for (int al = 0; al < PtNum; al++)
   {
      ptn[al].resize(3, CamNum);
      for (int kp = 0; kp < CamNum; kp++)
      {
         ptn[al].col(kp)(0) = pt0[al].col(kp)(0) / f0;
         ptn[al].col(kp)(1) = pt0[al].col(kp)(1) / f0;
         ptn[al].col(kp)(2) = z_ak(al,kp) = 1.0; // z_ak = 1
         
//         double nrm = ptn[al].col(kp).norm();
//         ptn[al].col(kp) /= nrm;
      }
   }

   // make observation matrix W
   W = observation_matrix(ptn, z_ak, CamNum, PtNum);

   // normalization of each column vector
   normalize_each_3rows(W);

   // iteration to minimize the reprojection errors
   E = Large_Number;
   count = 0;
   do
   {
      E0 = E;
      std::cerr << "***** loop: " << count;

      // Singular value decomposition
      SVD.compute(W, ComputeThinU | ComputeThinV);

      // extract first four columns of V
      v = extract_4columns(SVD.matrixV());

      // set projection matrices
      set_projection_matrices(W, v, Prj, CamNum);

      for (int kp = 0; kp < CamNum; kp++)
      {
         for (int al = 0; al < PtNum; al++)
         {
            Vector3d xa = ptn[al].col(kp);
            double xa_nrm = xa.norm();

            for (int i = 0; i < 4; i++)
            {
               C1(al,i) = xa(0) * v(al,i) / xa_nrm;
               C2(al,i) = xa(1) * v(al,i) / xa_nrm;
               C3(al,i) = xa(2) * v(al,i) / xa_nrm;
            }
         }
         C.block(0,0,PtNum,4) = C1;
         C.block(0,4,PtNum,4) = C2;
         C.block(0,8,PtNum,4) = C3;

         // compute eigen values and vectors
         SVD.compute(C, ComputeThinU | ComputeThinV);

         // eigen vector for the largest eigen value
         xi = SVD.matrixU().col(0);

         // check sign
         s = 0.0;
         for (int al = 0; al < PtNum; al++)
            s += xi(al);
         if (s < 0.0)
            xi = -xi;

         // update projective depth
         for (int al = 0; al < PtNum; al++)
         {
            Vector3d xa;

            xa = ptn[al].col(kp);
            z_ak(al,kp) = xi(al) / xa.norm();
         }
      }

      // update observation matrix W
      W = observation_matrix(ptn, z_ak, CamNum, PtNum);

      // normalization of each column vector
      normalize_each_3rows(W);

      // extract each reconstructed point
      for (int al = 0; al < PtNum; al++)
         X[al] = v.transpose().col(al);
      
      // set projection matrices
      set_projection_matrices(W, v, Prj, CamNum);
      
      // compute reprojection error
      s = 0.0;
      for (int al = 0; al < PtNum; al++)
         for (int kp = 0; kp < CamNum; kp++)
         {
            Vector3d   d, x_ak;

            x_ak = Znormalize(ptn[al].col(kp));
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
      std::cerr << "original_dual_method do not converge!" << std::endl;
      return false;
   }
            
   return true;
}
