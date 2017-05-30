//
// Perspective camera self-calibration
//
#ifndef _PERSPECTIVE_SELF_CALIBRATION_H_
#define _PERSPECTIVE_SELF_CALIBRATION_H_

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,9,9> Matrix9d;
typedef Matrix<double,10,10> Matrix10d;
typedef Matrix<double,3,4> Matrix34d;
typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,10,1> Vector10d;

// noname namespace
namespace {
   inline double sqr(double x)
   {
      return x*x;
   }
}

// Namespace PerspectiveSelfCalibration
namespace PerspectiveSelfCalibration {
   
   // constants for defualt values
   const double Default_f0 = 600.0;
   const double ini_f = 600.0;
   const double ini_u0 = 0.0;
   const double ini_v0 = 0.0;
   const int Max_Iteration = 10000;
   const double Reprojection_Error_EPS = 1.0;
   const double Convergence_EPS = 1e-5;
   const double Large_Number = 1e99;
   const double Almost0 = 1e-8;
   const double Rt2 = sqrt(2.0);

   // extract 4 columns from matrix
   inline MatrixXd left4cols(const MatrixXd& U0);

   // extract upper left 4x4 matrix from matrix
   inline Matrix4d upperleft4x4(const MatrixXd& Sg0);

   // extract 4 columns of U
   MatrixXd extract_4columns(const MatrixXd& U);

   // extract 4 rows of U
   MatrixXd extract_4rows(const MatrixXd& V);

   // normalize each column vector to unit length
   void normalize_each_column(MatrixXd& W);

   // normalize each 3 rows of W to unit length
   void normalize_each_3rows(MatrixXd& W);
   
   // make observationt matrix W
   MatrixXd observation_matrix(const MatrixXd x[],
                                    int CamNum, int PtNum,
                                    double f0 = Default_f0);

   // make observationt matrix W
   MatrixXd observation_matrix(const MatrixXd x[],
                               const MatrixXd& z_ak,
                               int CamNum,
                               int PtNum);
   
   // make observationt matrix W
   void observation_matrix(MatrixXd& W,
                           const MatrixXd x[],
                           const MatrixXd& z_ak,
                           int CamNum,
                           int PtNum);
   
   // Singular value decomposition of observation_matrix
   void orig_svd_decomp(const MatrixXd& W, MatrixXd& U, MatrixXd& Sg, MatrixXd& V);

   // Singular value decomposition of observation_matrix
   void svd_decomp(const MatrixXd& W, MatrixXd& U, MatrixXd& Sg, MatrixXd& V);

   // Singular value decomposition of observation_matrix with rank 4 constraint
   void svd_decomp_rank4(const MatrixXd& W, MatrixXd& U, MatrixXd& Sg, MatrixXd& V);
   // extract matrix U and set vectors u1 and u2
   void extract_uvecs(const MatrixXd& U, int CamNum, Vector3d u1[], Vector3d u2[]);

   // update vectors u_i
   void update_u(const MatrixXd& W, MatrixXd& u);
   
   // set projection matrices by vectors u_i
   inline void set_projection_matrices(const MatrixXd& u, Matrix34d Prj[], int CamNum)
   {
      for (int kp = 0; kp < CamNum; kp++)
         Prj[kp] = u.block<3,4>(kp*3,0);
   };

   // set projection matrices by vectors v_i
   void set_projection_matrices(const MatrixXd& W, const MatrixXd& v, Matrix34d Prj[], int CamNum);
   
   // normalize the 3rd element to 1
   inline Vector3d Znormalize(const Vector3d& v)
   {
      return v/v(2);
   };

   // make 10x10 matrix A
   inline Matrix10d set_matrix_A(const MatrixXd& A0)
   {
      Matrix10d A;
   
      A <<
         A0(0,0),      A0(0,5),      A0(0,10),      A0(0,15),  Rt2*A0(0,1), Rt2*A0(0,2), Rt2*A0(0,3), Rt2*A0(0,6), Rt2*A0(0,7), Rt2*A0(0,11),
         A0(5,0),      A0(5,5),      A0(5,10),      A0(5,15),  Rt2*A0(5,1), Rt2*A0(5,2), Rt2*A0(5,3), Rt2*A0(5,6), Rt2*A0(5,7), Rt2*A0(5,11),
         A0(10,0),     A0(10,5),     A0(10,10),     A0(10,15), Rt2*A0(10,1), Rt2*A0(10,2), Rt2*A0(10,3), Rt2*A0(10,6), Rt2*A0(10,7), Rt2*A0(10,11),
         A0(15,0),     A0(15,5),     A0(15,10),     A0(15,15), Rt2*A0(15,1), Rt2*A0(15,2), Rt2*A0(15,3), Rt2*A0(15,6), Rt2*A0(15,7), Rt2*A0(15,11),
         Rt2*A0(1,0),  Rt2*A0(1,5),  Rt2*A0(1,10),  Rt2*A0(1,15),  2.0*A0(1,1), 2.0*A0(1,2), 2.0*A0(1,3), 2.0*A0(1,6), 2.0*A0(1,7), 2.0*A0(1,11), 
         Rt2*A0(2,0),  Rt2*A0(2,5),  Rt2*A0(2,10),  Rt2*A0(2,15),  2.0*A0(2,1), 2.0*A0(2,2), 2.0*A0(2,3), 2.0*A0(2,6), 2.0*A0(2,7), 2.0*A0(2,11), 
         Rt2*A0(3,0),  Rt2*A0(3,5),  Rt2*A0(3,10),  Rt2*A0(3,15),  2.0*A0(3,1), 2.0*A0(3,2), 2.0*A0(3,3), 2.0*A0(3,6), 2.0*A0(3,7), 2.0*A0(3,11), 
         Rt2*A0(6,0),  Rt2*A0(6,5),  Rt2*A0(6,10),  Rt2*A0(6,15),  2.0*A0(6,1), 2.0*A0(6,2), 2.0*A0(6,3), 2.0*A0(6,6), 2.0*A0(6,7), 2.0*A0(6,11), 
         Rt2*A0(7,0),  Rt2*A0(7,5),  Rt2*A0(7,10),  Rt2*A0(7,15),  2.0*A0(7,1), 2.0*A0(7,2), 2.0*A0(7,3), 2.0*A0(7,6), 2.0*A0(7,7), 2.0*A0(7,11), 
         Rt2*A0(11,0), Rt2*A0(11,5), Rt2*A0(11,10), Rt2*A0(11,15), 2.0*A0(11,1), 2.0*A0(11,2), 2.0*A0(11,3), 2.0*A0(11,6), 2.0*A0(11,7), 2.0*A0(11,11);

      return A;
   };

   // convert omega to Omega
   inline Matrix4d convert_to_Omega(const Vector10d& om)
   {
      Matrix4d O;

      O <<
         om(0),     om(4)/Rt2, om(5)/Rt2, om(6)/Rt2,
         om(4)/Rt2, om(1),     om(7)/Rt2, om(8)/Rt2,
         om(5)/Rt2, om(7)/Rt2, om(2),     om(9)/Rt2,
         om(6)/Rt2, om(8)/Rt2, om(9)/Rt2, om(3);

         return O;
   };

   // pure function of simple factorization method
   bool pure_factorization(const MatrixXd& W,
                           MatrixXd& M,
                           MatrixXd& S);
   
   // function of simple factorization method
   bool simple_factorization(const MatrixXd pt0[],
                             int  CamNum,
                             int  PtNum,
                             Matrix34d Prj[],
                             Vector4d X[],
                             double f0 = Default_f0);

   // primary method of perspective camera self calibration
   bool faster_primary_method(const MatrixXd pt0[],
                              int  CamNum,
                              int  PtNum,
                              Matrix34d Prj[],
                              Vector4d  X[],
                              double rperr = Reprojection_Error_EPS,
                              double f0 = Default_f0,
                              int MaxIter = Max_Iteration);
   
   // primary method of perspective camera self calibration
   bool original_primary_method(const MatrixXd pt0[],
                                int  CamNum,
                                int  PtNum,
                                Matrix34d Prj[],
                                Vector4d  X[],
                                double rperr = Reprojection_Error_EPS,
                                double f0 = Default_f0,
                                int MaxIter = Max_Iteration);
   
   // Dual method
   bool faster_dual_method(const MatrixXd pt0[],
                           int  CamNum,
                           int  PtNum,
                           Matrix34d Prj[],
                           Vector4d X[],
                           double rperr = Reprojection_Error_EPS,
                           double f0 = Default_f0,
                           int MaxIter = Max_Iteration);

   // Dual method
   bool original_dual_method(const MatrixXd pt0[],
                             int  CamNum,
                             int  PtNum,
                             Matrix34d Prj[],
                             Vector4d X[],
                             double rperr = Reprojection_Error_EPS,
                             double f0 = Default_f0,
                             int MaxIter = Max_Iteration);
}

#endif // _PERSPECTIVE_SELF_CALIBRATION_H_
