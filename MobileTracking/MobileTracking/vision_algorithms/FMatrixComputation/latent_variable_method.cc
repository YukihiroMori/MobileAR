//
// Latent Variable Method for computing Fundamental Matrix
//

#include "FMatrixComputation.hpp"

using namespace Eigen;

namespace {

Matrix3d
rot_matrix(const Vector3d& w)
{
   Matrix3d     R;
   double       omega = w.norm();
   Vector3d     l = w / omega;
   double       cs, sn, cs1;

   cs = cos(omega);
   sn = sin(omega);
   cs1 = 1.0 - cs;

   R <<
      cs + sqr(l(0))*cs1, l(0)*l(1)*cs1 - l(2)*sn, l(0)*l(2)*cs1 + l(1)*sn,
      l(0)*l(1)*cs1 + l(2)*sn, cs + sqr(l(1))*cs1, l(1)*l(2)*cs1 - l(0)*sn,
      l(0)*l(2)*cs1 - l(1)*sn, l(1)*l(2)*cs1 + l(0)*sn, cs + sqr(l(2))*cs1;

   return R;
}

Matrix93d
set_Fu(const Matrix3d& F)
{
   Matrix93d    Fu;

   Fu(0,0) =  0.0;    Fu(0,1) =  F(2,0); Fu(0,2) = -F(1,0);
   Fu(1,0) =  0.0;    Fu(1,1) =  F(2,1); Fu(1,2) = -F(1,1);
   Fu(2,0) =  0.0;    Fu(2,1) =  F(2,2); Fu(2,2) = -F(1,2);
   Fu(3,0) = -F(2,0); Fu(3,1) =  0.0;    Fu(3,2) =  F(0,0); 
   Fu(4,0) = -F(2,1); Fu(4,1) =  0.0;    Fu(4,2) =  F(0,1); 
   Fu(5,0) = -F(2,2); Fu(5,1) =  0.0;    Fu(5,2) =  F(0,2);
   Fu(6,0) =  F(1,0); Fu(6,1) = -F(0,0); Fu(6,2) =  0.0;
   Fu(7,0) =  F(1,1); Fu(7,1) = -F(0,1); Fu(7,2) =  0.0;
   Fu(8,0) =  F(1,2); Fu(8,1) = -F(0,2); Fu(8,2) =  0.0;
   
   return Fu;
}

Matrix93d
set_Fv(const Matrix3d& F)
{
   Matrix93d    Fv;

   Fv(0,0) =  0.0;    Fv(0,1) =  F(0,2); Fv(0,2) = -F(0,1);
   Fv(1,0) = -F(0,2); Fv(1,1) =  0.0;    Fv(1,2) =  F(0,0);
   Fv(2,0) =  F(0,1); Fv(2,1) = -F(0,0); Fv(2,2) =  0.0;
   Fv(3,0) =  0.0;    Fv(3,1) =  F(1,2); Fv(3,2) = -F(1,1);
   Fv(4,0) = -F(1,2); Fv(4,1) =  0.0;    Fv(4,2) =  F(1,0);
   Fv(5,0) =  F(1,1); Fv(5,1) = -F(1,0); Fv(5,2) =  0.0;
   Fv(6,0) =  0.0;    Fv(6,1) =  F(2,2); Fv(6,2) = -F(2,1);
   Fv(7,0) = -F(2,2); Fv(7,1) =  0.0;    Fv(7,2) =  F(2,0);
   Fv(8,0) =  F(2,1); Fv(8,1) = -F(2,0); Fv(8,2) =  0.0;

   return Fv;
}

Vector9d
set_theta_phi(double s1, double s2, const Matrix3d& U, const Matrix3d& V)
{
   Vector9d     thp;

   thp <<
      s1*U(0,1)* V(0,1) - s2*U(0,0)*V(0,0),
      s1*U(0,1)* V(1,1) - s2*U(0,0)*V(1,0),
      s1*U(0,1)* V(2,1) - s2*U(0,0)*V(2,0),
      s1*U(1,1)* V(0,1) - s2*U(1,0)*V(0,0),
      s1*U(1,1)* V(1,1) - s2*U(1,0)*V(1,0),
      s1*U(1,1)* V(2,1) - s2*U(1,0)*V(2,0),
      s1*U(2,1)* V(0,1) - s2*U(2,0)*V(0,0),
      s1*U(2,1)* V(1,1) - s2*U(2,0)*V(1,0),
      s1*U(2,1)* V(2,1) - s2*U(2,0)*V(2,0);

   return thp;
}

double
compute_J(const Vector9d& theta, std::vector<Vector9d> Xi, std::vector<Matrix9d> V0, int Num)
{
   double       J;
   double       xith, thV0th;

   J = 0.0;
   for (int al = 0; al < Num; al++)
   {
      xith = theta.dot(Xi[al]);
      thV0th = theta.dot(V0[al] * theta);
      J += sqr(xith) / thV0th;
   }

   return J / (double)Num;
}

Matrix7d
diag(const Matrix7d& H)
{
   Matrix7d     T = FMatrixComputation::ZeroMat7;

   for (int i = 0; i < 7; i++)
      T(i,i) = H(i,i);

   return T;
}

// end of unnamed namespace
}

// Latent Variable Method: matrix form
// In this method, matrix F must be rank 2.
bool
FMatrixComputation::latent_variable_method(
   const Matrix3d &F0,
   Matrix3d &newF,
   std::vector<Vector2d> pos0,
   std::vector<Vector2d> pos1,
   int Num,
   int Max_Iter,
   double Conv_EPS,
   double f0
)
{
   Matrix3d     F, U, V, U1, V1, F1;
   Matrix93d    Fu, Fv;
   Vector9d     th_phi, th, th1;
   std::vector<Vector9d> Xi(Num);
   std::vector<Matrix9d> V0(Num);
   Matrix9d     M, L, X;
   Vector3d     DwJ, DwpJ;
   double       RJRp, RJ2p2;
   Vector3d     RDwJRp, RDwpJRp;
   Matrix3d     DwwJ, DwpwpJ, DwwpJ;
   Matrix7d     H, A;
   Vector7d     b, x;
   double       phi, phi1;
   double       s1, s2;
   Vector3d     tt;
   double       c;
   int          count1, count2;
   double       J, J1;
   double       dF;

   // SVD by Jacobi method
   F = F0;
   JacobiSVD<Matrix3d> SVD(F, ComputeFullU | ComputeFullV);

   // Singular Values
   s1 = SVD.singularValues()(0);
   s2 = SVD.singularValues()(1);
   phi = acos(s1);
   U = SVD.matrixU();
   V = SVD.matrixV();

   // compute Xi and V0 before iteration
   for (int al = 0; al < Num; al++)
   {
      Xi[al] = convert_vec9(pos0[al], pos1[al], f0);
      V0[al] = make_cov_mat(pos0[al], pos1[al], f0);
   }

   // set vector theta from F
   th <<
      F(0,0), F(0,1), F(0,2),
      F(1,0), F(1,1), F(1,2),
      F(2,0), F(2,1), F(2,2);

   // Levenberg-Marquardt loop
   J = compute_J(th, Xi, V0, Num);
   count1 = 0;
   c = 1e-4;
   do
   {
      // set matrices Fu and Fv
      Fu = set_Fu(F);
      Fv = set_Fv(F);

      // set 9-vector theta_phi
      th_phi = set_theta_phi(s1, s2, U, V);

      // set theta
      th <<
         F(0,0), F(0,1), F(0,2),
         F(1,0), F(1,1), F(1,2),
         F(2,0), F(2,1), F(2,2);

      // Ccompute matrices M and N
      M = L = ZeroMat9;
      for (int al = 0; al < Num; al++)
      {
         double tV0t;
         tV0t = th.dot(V0[al]*th);
         M += Xi[al] * Xi[al].transpose() / tV0t;
         L += sqr(th.dot(Xi[al]) / tV0t) * V0[al];
      }
      M /= (double)Num;
      L /= (double)Num;

      X = M - L;

      // 1st derivatives
      DwJ = 2.0 * Fu.transpose() * X * th;
      DwpJ = 2.0 * Fv.transpose() * X * th;
      RJRp = 2.0 * th_phi.dot(X * th);

      // 2nd derivatives
      DwwJ = 2.0 * Fu.transpose() * X * Fu;
      DwpwpJ = 2.0 * Fv.transpose() * X * Fv;
      DwwpJ = 2.0 * Fu.transpose() * X * Fv;
      RJ2p2 = 2.0 * th_phi.dot(X*th_phi);
      RDwJRp = 2.0 * Fu.transpose() * X * th_phi;
      RDwpJRp = 2.0 * Fv.transpose() * X * th_phi;

      // Hessian
      H.block(0,0,3,3) = DwwJ;
      H.block(0,3,3,3) = DwwpJ;
      H.block(3,0,3,3) = DwwpJ.transpose();
      H.block(3,3,3,3) = DwpwpJ;
      H.block(0,6,3,1) = RDwJRp;
      H.block(3,6,3,1) = RDwpJRp;
      H.block(6,0,1,3) = RDwJRp.transpose();
      H.block(6,3,1,3) = RDwpJRp.transpose();
      H(6,6) = RJ2p2;

      count2 = 0;
      do
      {
         // solve simultanous equation
         A = H + c * diag(H);
         b.block(0,0,3,1) = DwJ;
         b.block(3,0,3,1) = DwpJ;
         b(6) = RJRp;
         FullPivLU<Matrix7d> LU(A);
         x = LU.solve(b);

         // update U, V, and phi
         U1 = rot_matrix(x.block(0,0,3,1)) * U;
         V1 = rot_matrix(x.block(3,0,3,1)) * V;
         phi1 = phi + x(6);
         
         // Update F
         tt << cos(phi1), sin(phi1), 0.0;
         F1 = U1 * tt.asDiagonal() * V1.transpose();

         // compute J'
         th1 <<
            F1(0,0), F1(0,1), F1(0,2),
            F1(1,0), F1(1,1), F1(1,2),
            F1(2,0), F1(2,1), F1(2,2);

         // compute J'
         J1 = compute_J(th1, Xi, V0, Num);

         if (J1 - J > Conv_EPS)
            c *= 10.0;
      }
      while (++count2 < Max_Iter);

      if (count2 >= Max_Iter)  return false;

      dF = (F1 - F).norm();
      if (dF < Conv_EPS)
         break;

      // update F, U, V, phi, and c
      F = F1;
      U = U1;
      V = V1;
      phi = phi1;
      J = J1;
      c /= 10.0;
   }
   while (++count1 < Max_Iter);

   if (count1 >= Max_Iter)  return false;
         
   newF = F1;

   return true;
}
