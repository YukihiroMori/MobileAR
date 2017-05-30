//
// Common functions for perspective camera self-calibration
//

#include "PerspectiveSelfCalibration.h"

using namespace Eigen;

// small functions

// extract 4 columns from matrix
MatrixXd
PerspectiveSelfCalibration::left4cols(const MatrixXd& U0)
{
   MatrixXd U(U0.rows(),4);

   for (int r = 0; r < U0.rows(); r++)
      U.row(r) = U0.block<1,4>(r,0);

   return U;
}

// extract upper left 4x4 matrix from matrix
inline Matrix4d
PerspectiveSelfCalibration::upperleft4x4(const MatrixXd& Sg0)
{
   return Sg0.block<4,4>(0,0);
}

// extract 4 columns of U
MatrixXd
PerspectiveSelfCalibration::extract_4columns(const MatrixXd& U)
{
   MatrixXd     u(U.rows(),4);

   for (int i = 0; i < 4; i++)
      u.col(i) = U.col(i);

   return u;
}

// extract 4 rows of U
MatrixXd
PerspectiveSelfCalibration::extract_4rows(const MatrixXd& Vmat)
{
   MatrixXd     v(Vmat.cols(),4);

   for (int i = 0; i < 4; i++)
      v.col(i) = Vmat.row(i).transpose();

   return v;
}

// make observationt matrix W
MatrixXd
PerspectiveSelfCalibration::observation_matrix(
   const MatrixXd x[],
   int CamNum,
   int PtNum,
   double f0)
{
   MatrixXd W(3*CamNum, PtNum);

   for (int cam = 0; cam < CamNum; cam++)
   {
      int row1, row2, row3;
      
      row1 = cam * 3;
      row2 = cam * 3 + 1;
      row3 = cam * 3 + 2;
      for (int pt = 0; pt < PtNum; pt++)
      {
         double z;
         z = x[pt].col(cam)(2);
         W(row1,pt) = z * x[pt].col(cam)(0) / f0;
         W(row2,pt) = z * x[pt].col(cam)(1) / f0;
         W(row3,pt) = z;
      }
   }
   
   return W;
}

// make observationt matrix W
MatrixXd
PerspectiveSelfCalibration::observation_matrix(
   const MatrixXd x[],
   const MatrixXd& z_ak,
   int CamNum,
   int PtNum)
{
   MatrixXd W(3*CamNum, PtNum);

   for (int cam = 0; cam < CamNum; cam++)
   {
      int row1, row2, row3;
      
      row1 = cam * 3;
      row2 = cam * 3 + 1;
      row3 = cam * 3 + 2;
      for (int pt = 0; pt < PtNum; pt++)
      {
         W(row1,pt) = z_ak(pt,cam) * x[pt].col(cam)(0);
         W(row2,pt) = z_ak(pt,cam) * x[pt].col(cam)(1);
         W(row3,pt) = z_ak(pt,cam) * x[pt].col(cam)(2);
      }
   }
   
   return W;
}

// make observationt matrix W
void
PerspectiveSelfCalibration::observation_matrix(
   MatrixXd& W,
   const MatrixXd x[],
   const MatrixXd& z_ak,
   int CamNum,
   int PtNum)
{
   W.resize(3*CamNum, PtNum);

   for (int cam = 0; cam < CamNum; cam++)
   {
      int row1, row2, row3;
      
      row1 = cam * 3;
      row2 = cam * 3 + 1;
      row3 = cam * 3 + 2;
      for (int pt = 0; pt < PtNum; pt++)
      {
         W(row1,pt) = z_ak(pt,cam) * x[pt].col(cam)(0);
         W(row2,pt) = z_ak(pt,cam) * x[pt].col(cam)(1);
         W(row3,pt) = z_ak(pt,cam) * x[pt].col(cam)(2);
      }
   }
}

// normalize each column vector to unit length
void
PerspectiveSelfCalibration::normalize_each_column(MatrixXd& W)
{
   VectorXd     vc(W.rows());
   int  Ncol = W.cols();
   
   for (int c = 0; c < Ncol; c++)
   {
      vc = W.col(c);
      W.col(c) /= vc.norm();
   }
}

// normalize each 3 rows of W to unit length
void
PerspectiveSelfCalibration::normalize_each_3rows(MatrixXd& W)
{
   int  Nrow = W.rows();
   
   for (int r = 0; r < Nrow; r += 3)
   {
      double    nrm;

      nrm = sqrt(  W.row(r).squaredNorm()
                 + W.row(r+1).squaredNorm()
                 + W.row(r+2).squaredNorm());

      W.row(r)   /= nrm;
      W.row(r+1) /= nrm;
      W.row(r+2) /= nrm;
   }
}

// Singular value decomposition of observation_matrix
void
PerspectiveSelfCalibration::orig_svd_decomp(const MatrixXd& W,
                                            MatrixXd& U,
                                            MatrixXd& Sg,
                                            MatrixXd& V)
{
   // Singular value decomposition
   JacobiSVD<MatrixXd> SVD(W, ComputeThinU | ComputeThinV);

   // Matrix U
   U = SVD.matrixU();

   // Matrix Sigma
   Sg = SVD.singularValues().asDiagonal();

   // Matrix V
   V = SVD.matrixV();
}

// Singular value decomposition of observation_matrix
void
PerspectiveSelfCalibration::svd_decomp(const MatrixXd& W,
                                       MatrixXd& U,
                                       MatrixXd& Sg,
                                       MatrixXd& V)
{
   MatrixXd     Up, Sgp, Vp;
   int  M3, N, L;

   // L = min(3M,N)
   M3 = W.rows();
   N = W.cols();
   L = (M3 < N)? M3: N;
   
   // Singular value decomposition
   JacobiSVD<MatrixXd> SVD(W, ComputeThinU | ComputeThinV);

   // Matrix U
   Up = SVD.matrixU();
   U.resize(M3, L);
   U = Up.block(0, 0, M3, L);
//   std::cerr << "Up = " << Up.rows() << "x" << Up.cols() << std::endl;
//   std::cerr << "U = " << U.rows() << "x" << U.cols() << std::endl;

   // Matrix Sigma
   Sgp = SVD.singularValues().asDiagonal();
   Sg.resize(L, L);
   Sg = Sgp.block(0, 0, L, L);
//   std::cerr << "Sgp = " << Sgp.rows() << "x" << Sgp.cols() << std::endl;
//   std::cerr << "Sg = " << Sg.rows() << "x" << Sg.cols() << std::endl;

   // Matrix V
   Vp = SVD.matrixV();
   V.resize(N, L);
   V = Vp.block(0, 0, N, L);
//   std::cerr << "Vp = " << Vp.rows() << "x" << Vp.cols() << std::endl;
//   std::cerr << "V = " << V.rows() << "x" << V.cols() << std::endl;
}

// Singular value decomposition of observation_matrix
void
PerspectiveSelfCalibration::svd_decomp_rank4(const MatrixXd& W,
                                             MatrixXd& U,
                                             MatrixXd& Sg,
                                             MatrixXd& V)
{
   MatrixXd     Up, Sgp, Vp;
   
   // Singular value decomposition
   JacobiSVD<MatrixXd> SVD(W, ComputeThinU | ComputeThinV);

   // Matrix U
   Up = SVD.matrixU();
   U = left4cols(Up);

   // Matrix Sigma
   Sgp = SVD.singularValues().asDiagonal();
   Sg = upperleft4x4(Sgp);

   // Matrix V
   Vp = SVD.matrixV();
   V = left4cols(Vp);
}

// update vectors u_i
void
PerspectiveSelfCalibration::update_u(const MatrixXd& W, MatrixXd& u)
{
   // projection
   for (int i = 0; i < 4; i++)
   {
      VectorXd vt;
      vt = W.transpose() * u.col(i);
      u.col(i) = W * vt;
   }

   // Gramm-schmit orthogonalization
   u.col(0).normalize();
   u.col(1) = u.col(1) - u.col(0).dot(u.col(1)) * u.col(0);
   u.col(1).normalize();
   u.col(2) = u.col(2) - u.col(0).dot(u.col(2)) * u.col(0)
      + u.col(1).dot(u.col(2)) * u.col(1);
   u.col(2).normalize();
   u.col(3) = u.col(3) - u.col(0).dot(u.col(3)) * u.col(0)
      + u.col(1).dot(u.col(3)) * u.col(1)
      + u.col(2).dot(u.col(3)) * u.col(2);
   u.col(3).normalize();
}

// set projection matrices by vectors v_i
void
PerspectiveSelfCalibration::set_projection_matrices(
   const MatrixXd& W,
   const MatrixXd& v,
   Matrix34d Prj[],
   int CamNum)
{
   int Ncol = W.cols();

   for (int kp = 0; kp < CamNum; kp++)
   {
      MatrixXd qk = W.block(kp*3, 0, 3, Ncol);

      Prj[kp] = qk * v;
   }
}
