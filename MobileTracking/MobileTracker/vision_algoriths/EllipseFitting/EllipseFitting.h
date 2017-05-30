//
// Ellipse Fitting Methods
//
#ifndef _EllipseFitting_H_
#define _EllipseFitting_H_

#include <Eigen/Dense>

using namespace Eigen;

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,1,6> RowVector6d;
typedef Matrix<double,6,6> Matrix6d;

// unnamed namespace for tiny functions
namespace {
   inline double sqr(double x) { return x*x; }
}
// end of unnamed namespace  

// Namespace EllipseFitting
namespace EllipseFitting {

   // constants for defualt values
   const double Default_f0 = 600.0;
   const int Max_Iteration = 30;
   const double Convergence_EPS = 1e-5;
   const double Large_Number = 1e99;
   const double Max_Sample_Count = 1000;
   const int Ellipse_Sample_Number = 5;
   const double Distance_Threshold = 2;
   const unsigned long MT_Seed = 10;

   // constant Vectors and Matrices
   const Vector6d ZeroVec6 = Vector6d::Zero();
   const Matrix6d ZeroMat6 = Matrix6d::Zero();

   // common functionsconvert functions
   // Convert 2-vector to 6-vector
   Vector6d convert_vec6(Vector2d &p, double f0);
   // Convert 2-vector to 6-vector
   Vector6d convert_vec6_st(Vector2d &ph, Vector2d &pt, double f0);
   // Make covariance matrix
   Matrix6d make_cov_mat(Vector2d &p, double f0);

   // Rank trancated genelralized inverse
   Matrix6d trancated_generalized_inverse(Matrix6d& M, int rank);

   // Sampling unique indices
   void sample_unique_indices(int idx[], int sampleNum, int DataNum);

   // small functions
   Matrix6d Sym(Matrix6d &A);
   Matrix6d Sym2(Matrix6d &A);

   // least squares
   bool least_squares(Vector2d pos[],
                      int Num,
                      Vector6d &theta,
                      double f0 = Default_f0
                      );

   // iterative reweight method
   bool iterative_reweight(Vector2d pos[],
                           int Num,
                           Vector6d &theta,
                           int Max_Iter = Max_Iteration,
                           double Conv_EPS = Convergence_EPS,
                           double f0 = Default_f0
                           );

   // Taubin method
   bool taubin(Vector2d pos[],
               int Num,
               Vector6d &theta,
               double f0 = Default_f0
               );

   // renormalization
   bool renormalization(
                        Vector2d pos[],
                        int Num,
                        Vector6d &theta,
                        int Max_Iter = Max_Iteration,
                        double Conv_EPS = Convergence_EPS,
                        double f0 = Default_f0
                        );

   // hyper least squares
   bool hyper_least_squares(Vector2d pos[],
                            int Num,
                            Vector6d &theta,
                            double f0 = Default_f0
                            );

   // renormalization core routine
   // This is only used from other fitting functions,
   // so the function do not have any default values.
   bool hyper_renormalization_core(Vector6d Xi[],
                                   Matrix6d V0[],
                                   double W[],
                                   int Num,
                                   Vector6d &theta,
                                   int Max_Iter,
                                   double Conv_EPS
                                   );
   // renormalization for users
   bool hyper_renormalization(Vector2d pos[],
                              int Num,
                              Vector6d &theta,
                              int Max_Iter = Max_Iteration,
                              double Conv_EPS = Convergence_EPS,
                              double f0 = Default_f0
                              );
   
   // FNS core routine
   // This is only used from other fitting functions,
   // so the function do not have any default values.
   bool fns_core(Vector6d Xi[],
                 Matrix6d V0[],
                 double W[],
                 int Num,
                 Vector6d &theta,
                 Matrix6d &M,
                 int Max_Iter,
                 double Conv_EPS
                 );
   // FNS for users
   bool fns(Vector2d pos[],
            int Num,
            Vector6d &theta,
            int Max_Iter = Max_Iteration,
            double Conv_EPS = Convergence_EPS,
            double f0 = Default_f0
            );

   // Geometric distance minimization
   bool geometric_distance_minimization(Vector2d pos[],
                                        int      Num,
                                        Vector6d &theta,
                                        int      Max_Iter = Max_Iteration,
                                        double   Conv_EPS = Convergence_EPS,
                                        double   f0 = Default_f0
                                        );

   // Hyperaccurate correction 
   bool hyperaccurate_correction(Vector2d pos[],
                                 int Num,
                                 Vector6d &theta,
                                 int Max_Iter = Max_Iteration,
                                 double Conv_EPS = Convergence_EPS,
                                 double f0 = Default_f0
                                 );

   // Fitsgibbon's method
   bool fitzgibbon(Vector2d pos[],
                   int Num,
                   Vector6d &theta,
                   double f0 = Default_f0
                   );

   // Random sampling method
   bool random_sampling(Vector2d pos[],
                        int Num,
                        Vector6d &theta,
                        unsigned long Seed = MT_Seed,
                        int Max_Iter = Max_Iteration,
                        double Conv_EPS = Convergence_EPS,
                        int Max_Count = Max_Sample_Count,
                        double f0 = Default_f0
                        );

   // RANSAC method
   bool ransac(Vector2d pos[],
               int Num,
               Vector6d &theta,
               unsigned long Seed = MT_Seed,
               double dist = Distance_Threshold,
               int Max_Count = Max_Sample_Count,
               double f0 = Default_f0
               );
}
// end of namepsace EllipseFitting

#endif // _EllipseFitting_H_
