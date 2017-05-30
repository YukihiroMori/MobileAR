//
// Affine camera self-calibration
//
#ifndef _AFFINE_SELF_CALIBRATION_H_
#define _AFFINE_SELF_CALIBRATION_H_

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

typedef Matrix<double,9,9> Matrix9d;
typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,6,1> Vector6d;

// noname namespace
namespace {
   inline double sqr(double x)
   {
      return x*x;
   }
}

// Namespace AffineSelfCalibration
namespace AffineSelfCalibration {
   
   // constants for defualt values
   const double Default_f0 = 600.0;
   const int Max_Iteration = 30;
   const double Convergence_EPS = 1e-5;
   const double Large_Number = 1e99;
   const double Almost0 = 1e-8;
   const double Root2 = sqrt(2.0);

   // extract 3 columns from matrix
   inline MatrixXd left3cols(const MatrixXd& U0);

   // extract upper left 3x3 matrix from matrix
   inline Matrix3d upperleft3x3(const MatrixXd& Sg0);

   // compute the center of the gravity
   Vector2d center_of_gravity(const MatrixXd pt[], int PtNum, int cam);
   void center_of_gravity(const MatrixXd pt[], int PtNum, int cam, double *tx, double *ty);

   // translate the origin to the center of the gravity
   void translation(const MatrixXd pt[], MatrixXd ptt[], int PtNum, int cam, const Vector2d& gp);
   void translation(const MatrixXd pt[], MatrixXd ptt[], int PtNum, int cam, double tx, double ty);

   // make observationt matrix W
   MatrixXd observation_matrix(const MatrixXd x[], int CamNum, int PtNum);

   // Singular value decomposition of observation_matrix
   void svd_decomp(const MatrixXd& W, MatrixXd& U, MatrixXd& Sg, MatrixXd& V);

   // extract matrix U and set vectors u1 and u2
   void extract_uvecs(const MatrixXd& U, int CamNum, Vector3d u1[], Vector3d u2[]);

   // make 6x6 matrix B
   inline Matrix6d set_matrix_B(const Matrix9d& B0)
   {
      Matrix6d B;
   
      B <<
         B0(0,0),       B0(0,4),       B0(0,8),       Root2*B0(0,5),  Root2*B0(0,6),  Root2*B0(0,1),
         B0(4,0),       B0(4,4),       B0(4,8),       Root2*B0(4,5),  Root2*B0(4,6),  Root2*B0(4,1),
         B0(8,0),       B0(8,4),       B0(8,8),       Root2*B0(8,5),  Root2*B0(8,6),  Root2*B0(8,1),
         Root2*B0(5,0), Root2*B0(5,4), Root2*B0(5,8), 2.0*B0(5,5),    2.0*B0(5,6),    2.0*B0(5,1),
         Root2*B0(6,0), Root2*B0(6,4), Root2*B0(6,8), 2.0*B0(6,5),    2.0*B0(6,6),    2.0*B0(6,1),
         Root2*B0(1,0), Root2*B0(1,4), Root2*B0(1,8), 2.0*B0(1,5),    2.0*B0(1,6),    2.0*B0(1,1);
      return B;
   };

   // convert tau to T
   inline Matrix3d convert_to_T(const Vector6d& tau)
   {
      Matrix3d T;

      T <<
         tau(0),       tau(5)/Root2, tau(4)/Root2,
         tau(5)/Root2, tau(1),       tau(3)/Root2,
         tau(4)/Root2, tau(3)/Root2, tau(2);
      return T;
   };

   // pure factorization method
   bool pure_factorization(const MatrixXd& W, MatrixXd& M, MatrixXd& S);

   // function of simple factorization method
   bool simple_factorization(const MatrixXd pt0[],
                             int  CamNum,
                             int  PtNum,
                             MatrixXd& M,
                             MatrixXd& S);

   // Symmetric affine camera factorization method
   bool symmetric_affine_selfcalibration(const MatrixXd pt0[],
                                         int  CamNum,
                                         int  PtNum,
                                         MatrixXd& M,
                                         MatrixXd& S);
   
   // Paraperspective factorization
   bool paraperspective_selfcalibration(const MatrixXd pt0[],
                                        const double fl0[],
                                        int  CamNum,
                                        int  PtNum,
                                        MatrixXd& M,
                                        MatrixXd& S);

   // Paraperspective factorization
   bool weak_perspective_selfcalibration(const MatrixXd pt0[],
                                         int  CamNum,
                                         int  PtNum,
                                         MatrixXd& M,
                                         MatrixXd& S);
   
   // Orthographic factorization
   bool orthographic_selfcalibration(const MatrixXd pt0[],
                                     int  CamNum,
                                     int  PtNum,
                                     MatrixXd& M,
                                     MatrixXd& S);
}

#endif // _AFFINE_SELF_CALIBRATION_H_
