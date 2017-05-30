//
// Optimal correction for multiple view triangulation
//

#include <iostream>

#include "Triangulation.hpp"
#include "MultipleViewTriangulation.hpp"

// small function 
namespace {
   // convert two projection matices to F matrix
   Vector9d make_Fmat(const Matrix34d& P0, const Matrix34d& P1)
   {
      Vector9d theta;
      Matrix4d T;

      // F(0,0)
      T.row(0) = P0.row(1); T.row(1) = P0.row(2);
      T.row(2) = P1.row(1); T.row(3) = P1.row(2);
      theta(0) = T.determinant();

      // F(0,1): 1st and 2nd rows are equivalent to F(0,0)
      T.row(2) = P1.row(2); T.row(2) = P1.row(0);
      theta(1) = T.determinant();

      // F(0,2): 1st and 2nd rows are equivalent to F(0,0)
      T.row(2) = P1.row(0); T.row(2) = P1.row(1);
      theta(2) = T.determinant();

      // F(1,0)
      T.row(0) = P0.row(2); T.row(1) = P0.row(0);
      T.row(2) = P1.row(1); T.row(3) = P1.row(2);
      theta(3) = T.determinant();

      // F(1,1)
      T.row(0) = P0.row(0); T.row(1) = P0.row(2);
      T.row(2) = P1.row(0); T.row(3) = P1.row(2);
      theta(4) = T.determinant();

      // F(1,2)
      T.row(0) = P0.row(2); T.row(1) = P0.row(0);
      T.row(2) = P1.row(0); T.row(3) = P1.row(1);
      theta(5) = T.determinant();

      // F(2,0)
      T.row(0) = P0.row(0); T.row(1) = P0.row(1);
      T.row(2) = P1.row(1); T.row(3) = P1.row(2);
      theta(6) = T.determinant();

      // F(2,1)
      T.row(2) = P1.row(2); T.row(3) = P1.row(0);
      theta(7) = T.determinant();

      // F(2,2)
      T.row(2) = P1.row(0); T.row(3) = P1.row(1);
      theta(8) = T.determinant();

      return theta;
   }
} // unnamed namespace

// Optimal correction
bool
MultipleViewTriangulation::optimal_correction(
   std::vector<std::vector<Matrix3d>> tfT,
   int CNum,
   std::vector<Vector3d> xk,
   std::vector<Vector3d> xc,
   double *reperr,
   int Max_Iter,
   double Conv_EPS
)
{
   std::vector<Vector3d> xkh(CNum), xkt(CNum);
   double EE, E0;
   std::vector<Matrix39d> P(CNum), Q(CNum), R(CNum);
   Vector3d Pk[3], pt, qt, rt;
   std::vector<Matrix9d> A(CNum), B(CNum), C(CNum), D(CNum), E(CNum);
   std::vector<Vector9d> F(CNum);
//   Vector9d lmd[CNum];

   // Matrix 9(M-2) * 9(M-2)
   int sz = 9*(CNum - 2);
   MatrixXd T;
   MatrixXd Ti(sz, sz);
   VectorXd ff(sz);
   VectorXd llmd(sz);

   int count;

   // Set Pk
   Pk[0] << 1.0, 0.0, 0.0;
   Pk[1] << 0.0, 1.0, 0.0;
   Pk[2] << 0.0, 0.0, 0.0;

   for (int kp = 0; kp < CNum; kp++)
   {
      xkh[kp] = xk[kp];
      xkt[kp] = ZeroVec3;
   }

   E0 = Large_Number;
   count = 0;

   do
   {
      int pq, rs;
      // set Pkpqs, Qkpqs, and Rkpqs
      // Pkpqs = P[kp](s,3*p+q)
      // Qkpqs = Q[kp](s,3*p+q)
      // Rkpqs = R[kp](s,3*p+q)
      for (int kp = 0; kp < CNum; kp++)
      {
         int kpm1, kpm2, kpp1, kpp2;
         kpm1 = kp - 1;
         kpm2 = kp - 2;
         kpp1 = kp + 1;
         kpp2 = kp + 2;
         
	 for (int s = 0; s < 3; s++)
	    for (int p = 0; p < 3; p++)
	       for (int q = 0; q < 3; q++)
	       {
		  pq = 3*p + q;
                  if (kp < CNum - 2)
                     P[kp](s,pq) = calc_Txyz(&tfT[kp][0], Pk[s], xkh[kpp1], xkh[kpp2], p, q);
                  else
                     P[kp](s,pq) = 0.0; 
		  if (kp >= 1 && kp < CNum -1)
		     Q[kp](s,pq) = calc_Txyz(&tfT[kp-1][0], xkh[kpm1], Pk[s], xkh[kpp1], p, q);
                  else
                     Q[kp](s,pq) = 0.0;
		  if (kp >= 2)
		     R[kp](s,pq) = calc_Txyz(&tfT[kp-2][0], xkh[kpm2], xkh[kpm1], Pk[s], p, q);
                  else
		     R[kp](s,pq) = 0.0;
	       }
      }

      // set Ak(pqrs), Bk(pqrs), Ck(pqrs), Dk(pqrs), Ek(pqrs), and Fk(pq)
      for (int kp = 0; kp < CNum - 2; kp++)
      {
	 for (int p = 0; p < 3; p++)
	 {
	    for (int q = 0; q < 3; q++)
	    {
	       pq = 3*p + q;
	       for (int r = 0; r < 3; r++)
	       {
		  for (int s = 0; s < 3; s++)
		  {
		     rs = 3*r + s;
		     
		     if (kp >= 2)
		     {
			A[kp](pq,rs) =
			   calc_Txyz(&tfT[kp][0], R[kp].col(rs), xkh[kp+1], xkh[kp+2], p, q);
		     }
		     if (kp >= 1)
		     {
			B[kp](pq,rs) =
			   calc_Txyz(&tfT[kp][0], Q[kp].col(rs), xkh[kp+1], xkh[kp+2], p, q)
			   + calc_Txyz(&tfT[kp][0], xkh[kp], R[kp+1].col(rs), xkh[kp+2], p, q);
		     }
		     C[kp](pq,rs) = 
			calc_Txyz(&tfT[kp][0], P[kp].col(rs), xkh[kp+1], xkh[kp+2], p, q)
			+ calc_Txyz(&tfT[kp][0], xkh[kp], Q[kp+1].col(rs), xkh[kp+2], p, q)
			+ calc_Txyz(&tfT[kp][0], xkh[kp], xkh[kp+1], R[kp+2].col(rs), p, q);
		     if (kp <= CNum - 4)
		     {
			D[kp](pq,rs) =
			   calc_Txyz(&tfT[kp][0], xkh[kp], P[kp+1].col(rs), xkh[kp+2], p, q)
			   + calc_Txyz(&tfT[kp][0], xkh[kp], xkh[kp+1], Q[kp+2].col(rs), p, q);
		     }
		     if (kp <= CNum - 5)
		     {
			E[kp](pq,rs) =
			   calc_Txyz(&tfT[kp][0], xkh[kp], xkh[kp+1], P[kp+2].col(rs), p, q);
		     }
		  }
	       }
	       F[kp](pq) =
		  calc_Txyz(&tfT[kp][0], xkh[kp], xkh[kp+1], xkh[kp+2], p, q)
		  + calc_Txyz(&tfT[kp][0], xkt[kp], xkh[kp+1], xkh[kp+2], p, q)
		  + calc_Txyz(&tfT[kp][0], xkh[kp], xkt[kp+1], xkh[kp+2], p, q)
		  + calc_Txyz(&tfT[kp][0], xkh[kp], xkh[kp+1], xkt[kp+2], p, q);
	    }
	 }
      }
      
      T = MatrixXd::Zero(sz, sz);
      for (int kp = 0; kp < CNum - 2; kp++)
      {
	 T.block(9*kp,9*kp,9,9) = C[kp];
	 if (kp <= CNum - 4)
         {
	    T.block(9*kp, 9*(kp+1), 9, 9) = D[kp];
         }
	 if (kp <= CNum - 5)
         {
	    T.block(9*kp, 9*(kp+2), 9, 9) = E[kp];
         }
	 if (kp >= 1)
         {
	    T.block(9*kp, 9*(kp-1), 9, 9) = B[kp];
         }
	 if (kp >= 2)
         {
	    T.block(9*kp, 9*(kp-2), 9, 9) = A[kp];
         }
	 ff.segment(9*kp,9) = F[kp];
      }

      // Generalized Inverse
      Ti = generalized_inverse_rank_N(T, 2*CNum - 3);

      // solve linear equations
      llmd = Ti * ff;

      // update xt and xt
      for (int kp = 0; kp < CNum; kp++)
      {
	 xkt[kp] = ZeroVec3;
	 for (int i = 0; i < 3; i++)
	    for (int p = 0; p < 3; p++)
	       for (int q = 0; q < 3; q++)
	       {
                  double ttt1, ttt2, ttt3;
                  
		  pq = 3*p + q;

                  if (9*kp + pq >= sz)
                     ttt1 = 0.0;
                  else
                     ttt1 = P[kp](i,pq) * llmd(9*kp+pq);
                  
                  if (9*(kp-1) + pq >= sz || kp < 1)
                     ttt2 = 0.0;
                  else
                     ttt2 = Q[kp](i,pq) * llmd(9*(kp-1)+pq);

                  if (9*(kp-2) + pq >= sz || kp < 2)
                     ttt3 = 0.0;
                  else
                     ttt3 = R[kp](i,pq) * llmd(9*(kp-2)+pq);
                  
		  xkt[kp](i) += ttt1 + ttt2 + ttt3;		     
	       }
	 xkh[kp] = xk[kp] - xkt[kp];
      }

      // Compute reprojection error
      EE = 0.0;
      for (int kp = 0; kp < CNum; kp++)
	 EE += xkt[kp].squaredNorm();

      if (fabs (EE - E0) < Conv_EPS)
      {
         *reperr = EE;
	 for (int kp = 0; kp < CNum; kp++)
	    xc[kp] = xkh[kp];

	 return true;
      }
      E0 = EE;
   }
   while (++count < Max_Iter);

   return false;
}

// correct all points for every camera.
bool
MultipleViewTriangulation::optimal_correction_all(
   std::vector<Matrix34d> Proj,
   int CamNumAll,
   std::vector<MatrixXd> Xk0,
   std::vector<MatrixXd> Xkc0,
   MatrixXi& idx,
   double *reperr,
   int PtNum,
   int Max_Iter,
   double Conv_EPS,
   double f0
)
{
   int CamNum;
   int cc;
   bool flag;

   // size check
   if (Xk0[0].rows() != 2)
      return false;

   // correction loop for each point
   for (int pnum = 0; pnum < PtNum; pnum++)
   {
      // Count cameras
      CamNum = 0;
      for (int cnum = 0; cnum < CamNumAll; cnum++)
      {
         idx(pnum,cnum) = false;
         if (Xk0[pnum].col(cnum)(0) < 0.0) continue; // x < 0: a point cannot be observe
         idx(pnum,cnum) = true;
         CamNum++;
      }

      // three or more cameras
      if (CamNum >= 3)
      {
         std::vector<Matrix34d> Prj(CamNum);
         std::vector<std::vector<Matrix3d>> tfT(CamNum-2);
         std::vector<Vector3d> xk(CamNum), xkc(CamNum);
         
         for(int i = 0; i < CamNum - 2; i++){
             tfT[i] = std::vector<Matrix3d>(3);
         }
          
         cc = 0;
         for (int cnum = 0; cnum < CamNumAll; cnum++)
         {
            if (idx(pnum,cnum))
            {
               xk[cc] << Xk0[pnum].col(cnum)/f0, 1.0;
               Prj[cc] = Proj[cnum];
               cc++;
            }
         }
      
         // compute trifocal tensors
         for (int cnum = 0; cnum < CamNum - 2; cnum++)
            make_trifocal_tensor(&(Prj[cnum]), &(tfT[cnum][0]));

         // correction
         flag = optimal_correction(tfT, CamNum, xk, xkc, reperr,
                                   Max_Iter, Conv_EPS);

         if (flag) // success
         {
            cc = 0;
            for (int cnum = 0; cnum < CamNumAll; cnum++)
            {
               if (idx(pnum,cnum))
               {
                  Xkc0[pnum].col(cnum) << f0 * xkc[cc](0), f0 * xkc[cc](1);
                  cc++;
               }
               else
                  Xkc0[pnum].col(cnum) = ZeroVec2;
            }
         }
         else // failed
         {
            for (int cnum = 0; cnum < CamNum; cnum++)
               Xkc0[pnum].col(cnum) = ZeroVec2;
         }
      } 
      else if (CamNum == 2)
      {
         std::vector<Matrix34d> Prj(CamNum);
         Vector2d pos[2], np[2];
         Vector9d theta;
         
         cc = 0;
         for (int cnum = 0; cnum < CamNumAll; cnum++)
         {
            if (idx(pnum,cnum))
            {
               Prj[cc] = Proj[cnum];               // copy projection matrix
               pos[cc] = Xk0[pnum].col(cnum);      // copy data
               cc++;
            }
         }

         // Fundamental matrix
         theta = make_Fmat(Prj[0], Prj[1]);
      
         // correction using stereo triangulation
         reperr[pnum] = Triangulation::optimal_correction(pos[0], pos[1], theta,
                                                          np[0], np[1],
                                                          Max_Iter, Conv_EPS, f0);
         // copy correcetd data
         cc = 0;
         for (int cnum = 0; cnum < CamNumAll; cnum++)
         {
            if (idx(pnum,cnum))
            {
               Xkc0[pnum].col(cnum) = np[cc];
               cc++;
            }
            else
            {
               Xkc0[pnum].col(cnum) = ZeroVec2;
            }
         }
      }
   }

   return true;
}
