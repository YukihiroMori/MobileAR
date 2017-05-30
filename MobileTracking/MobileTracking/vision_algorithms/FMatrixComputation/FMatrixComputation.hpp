//
// Fundamental Matrix Computation Methods
//
#ifndef _FMatrixComputation_H_
#define _FMatrixComputation_H_

#include <Eigen/Dense>
#include <vector>
#include "Config.h"

using namespace Eigen;

typedef Matrix<double,9,9> Matrix9d;
typedef Matrix<double,9,3> Matrix93d;
typedef Matrix<double,9,1> Vector9d;
typedef Matrix<double,7,7> Matrix7d;
typedef Matrix<double,7,1> Vector7d;
typedef Matrix<double,2,3> Matrix23d;

// unnamed namespace for small function
#ifndef SQR
#define SQR
namespace {
    static double sqr(double x) { return x*x; }
}
#endif

// Namespace FMatrixComputation
namespace FMatrixComputation {
   
   // constants for defualt values
   const double Default_f0 = Default_F0;
   const int Max_Iteration =  3;
   const double Convergence_EPS = 1e-5;
   const double Large_Number = 1e99;
   const int FMat_Sample_Number = 8;
   const double Max_Sample_Count = 1000;
   const double Distance_Threshold = 2;
   const unsigned long MT_Seed = 10;

   // constant Vectors and Matrices
   const Vector9d ZeroVec9 = Vector9d::Zero();
   const Vector2d ZeroVec2 = Vector2d::Zero();
   const Matrix9d ZeroMat9 = Matrix9d::Zero();
   const Matrix7d ZeroMat7 = Matrix7d::Zero();
   const Matrix9d I9 = Matrix9d::Identity();

   // common functionsconvert functions
   // Convert two 2-vectors to 9-vector
   Vector9d convert_vec9(const Vector2d &p0, const Vector2d &p1, double f0);
   // Convert four 2-vectors to 9-vector
   Vector9d convert_vec9_st(const Vector2d &p0h, const Vector2d &p1h,
                            const Vector2d &p0t, const Vector2d &p1t,
                            double f0);
   // Make covariance matrix
   Matrix9d make_cov_mat(const Vector2d &p0, const Vector2d &p1, double f0);
   // Convert 9-vector to 33-matrix
   Matrix3d convert_mat3(const Vector9d &th);
   // Convert 33-matrix to 9-vector
   Vector9d convert_vec9(const Matrix3d &F);
   // convert theta^\dag
   Vector9d theta_dag(const Vector9d &th);

   // Sampling unique indices
   void sample_unique_indices(int idx[], int sampleNum, int DataNum);
   
   // Rank correction of fundamental matrix by SVD: Matrix form
   Matrix3d svd_rank_correction(const Matrix3d &F);

   // Rank correction of fundamental matrix by SVD: Vector form
   Vector9d svd_rank_correction(const Vector9d &theta);

   // Optimal rank correction of fundamental matrix: vector form
   bool optimal_rank_correction(const Vector9d &theta,
                                Vector9d &ntheta,
                                std::vector<Vector2d> pos0,
                                std::vector<Vector2d> pos1,
                                int Num,
                                double Conv_EPS = Convergence_EPS,
                                double f0 = Default_f0);

   // least squares
   bool least_squares(std::vector<Vector2d> pos0,
                      std::vector<Vector2d> pos1,
                      int Num,
                      Vector9d& theta,
                      double f0 = Default_f0);

   // Taubin method (core)
    bool taubin_core(std::vector<Vector9d> Xi,
                     std::vector<Matrix9d> V0,
                    int      Num,
                    Vector9d& theta);

   // Taubin method
   bool taubin(std::vector<Vector2d> pos0,
               std::vector<Vector2d> pos1,
               int      Num,
               Vector9d& theta,
               double    f0 = Default_f0);
   
   // extended FNS method (core)
   bool efns_core(Vector9d &theta,
                  std::vector<Vector9d> Xi,
                  std::vector<Matrix9d> V0,
                  int Num,
                  int Max_Iter,
                  double Conv_EPS);

   // extended FNS method
   bool efns(std::vector<Vector2d> pos0,
             std::vector<Vector2d> pos1,
             int      Num,
             Vector9d &theta,
             int      Max_Iter = Max_Iteration,
             double   Conv_EPS = Convergence_EPS,
             double   f0 = Default_f0);

   // RANSAC method
   bool ransac(std::vector<Vector2d> pos0,
               std::vector<Vector2d> pos1,
               int Num,
               Vector9d &theta,
               unsigned long Seed = MT_Seed,
               double dist = Distance_Threshold,
               int Max_Count = Max_Sample_Count,
               double f0 = Default_f0);

   // Geometric distance minimization
   bool geometric_distance_minimization(std::vector<Vector2d> p0,
                                        std::vector<Vector2d> p1,
                                        int Num,
                                        const Vector9d &theta0,
                                        Vector9d &theta,
                                        int  Max_Iter = Max_Iteration,
                                        double Conv_EPS = Convergence_EPS,
                                        double f0 = Default_f0);

   // Latent Variable Method: matrix form
   // In this method, matrix F must be rank 2.
   bool latent_variable_method(const Matrix3d &F,
                               Matrix3d &newF,
                               std::vector<Vector2d> pos0,
                               std::vector<Vector2d> pos1,
                               int Num,
                               int Max_Iter = Max_Iteration,
                               double Conv_EPS = Convergence_EPS,
                               double f0 = Default_f0);
   
}

#endif // _FMatrixComputation_H_
