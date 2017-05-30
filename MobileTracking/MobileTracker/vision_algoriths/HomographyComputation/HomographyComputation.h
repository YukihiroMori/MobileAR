//
// Homography Computation
//
#ifndef _HOMOGRAPHY_COMPUTATION_H_
#define _HOMOGRAPHY_COMPUTATION_H_

#include <Eigen/Dense>

using namespace Eigen;

typedef Matrix<double,9,9> Matrix9d;
typedef Matrix<double,9,4> Matrix94d;
typedef Matrix<double,9,1> Vector9d;
typedef DiagonalMatrix<double,3> DiagMatrix3d;

// unnamed namespace for small function
namespace {
   double sqr(double x)
   {
      return x*x;
   }

   Matrix3d skew_sym_mat3(const Vector3d& a)
   {
      Matrix3d S;
      S << 0.0, -a[2], a[1], a[2], 0.0, -a[0], -a[1], a[0], 0.0;
      return S;      
   }

   Matrix9d sym_mat9(const Matrix9d& A)
   {
      return (A + A.transpose()) / 2.0;
   }
   
   // convert to mat3
   Matrix3d convert_mat3(const Vector9d& v)
   {
      Matrix3d M;
      M << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];

      return M;
   }
   
   // convert to vec9
   Vector9d convert_vec9(const Matrix3d& m)
   {
      Vector9d V;
      V << m(0,0), m(0,1), m(0,2),
         m(1,0), m(1,1), m(1,2),
         m(2,0), m(2,1), m(2,2);

      return V;
   }
}

// Namespace HomographyComputation

namespace HomographyComputation {
   // constants
   const double Convergence_EPS = 1e-6;
   const int    Max_Iteration =  100;
   const double Large_Number = 1e99;
   const double Default_f0 = 600;

   const double Max_Sample_Count = 1000;
   const int Homography_Sample_Number = 4;
   const double Distance_Threshold = 2;
   const unsigned long MT_Seed = 10;
   
   const Matrix9d ZeroMat9 = Matrix9d::Zero();
   const Matrix3d ZeroMat3 = Matrix3d::Zero();
   const Matrix94d ZeroMat94 = Matrix94d::Zero();
   const Vector4d ZeroVec4 = Vector4d::Zero();
   const Vector9d ZeroVec9 = Vector9d::Zero();
   const Matrix3d I3 = Matrix3d::Identity();
   const DiagonalMatrix<double,3> Pk(1.0, 1.0, 0.0);

   // small functions 
   // set xi1
   Vector9d set_xi1(const Vector2d& pos0, const Vector2d& pos1, double f0);
   // set xi2
   Vector9d set_xi2(const Vector2d& pos0, const Vector2d& pos1, double f0);
   // set xi3
   Vector9d set_xi3(const Vector2d& pos0, const Vector2d& pos1, double f0);
   // set xi1 from vec4
   Vector9d set_xi1(const Vector4d& pos, double f0);
   // set xi2 from vec4
   Vector9d set_xi2(const Vector4d& pos, double f0);
   // set xi3 from vec4
   Vector9d set_xi3(const Vector4d& pos, double f0);
   // set Ta1
   void set_Ta1(Matrix94d& Ta1,
                const Vector2d& pos0, const Vector2d& pos1, double f0);
   // set Ta2
   void set_Ta2(Matrix94d& Ta2,
                const Vector2d& pos0, const Vector2d& pos1, double f0);
   // set Ta3
   void set_Ta3(Matrix94d& Ta3,
                const Vector2d& pos0, const Vector2d& pos1, double f0);
   // set Ta1 from vec4
   void set_Ta1(Matrix94d& Ta1, const Vector4d& pos, double f0);
   // set Ta2 from vec4
   void set_Ta2(Matrix94d& Ta2, const Vector4d& pos, double f0);
   // set Ta3 from vec4
   void set_Ta3(Matrix94d& Ta3, const Vector4d& pos, double f0);

   // Other common functions
   // Rank constrained Generalized Inverse
   template<class MatT> MatT gen_inv_r(const MatT& M, int r);
   
   // Rank constrained Generalized Inverse
   Matrix3d trancated_generalized_inverse(const Matrix3d& M, int rank);

   // Rank constrained Generalized Inverse
   Matrix9d trancated_generalized_inverse(const Matrix9d& M, int rank);

   // Sampling unique indices
   void sample_unique_indices(int idx[], int sampleNum, int DataNum);

   // Homography computation functions
   // Least-squares method
   bool least_squares(Vector2d pos0[],
                      Vector2d pos1[],
                      int Num,
                      Matrix3d& H,
                      double f0 = Default_f0);

   // Iteration with weight update
   bool iterative_reweight(Vector2d pos0[],
                           Vector2d pos1[],
                           int Num,
                           Matrix3d& H,
                           int Max_Iter = Max_Iteration,
                           double Conv_EPS = Convergence_EPS,
                           double f0 = Default_f0);

   // Taubin method
   bool taubin(Vector2d pos0[],
               Vector2d pos1[],
               int Num,
               Matrix3d& H,
               double f0 = Default_f0);

   // Renormalization
   bool renormalization(Vector2d pos0[],
                        Vector2d pos1[],
                        int Num,
                        Matrix3d& H,
                        int Max_Iter = Max_Iteration,
                        double Conv_EPS = Convergence_EPS,
                        double f0 = Default_f0);

   // Renormalization with Hyper Accuracy
   bool hyper_renormalization(Vector2d pos0[],
                              Vector2d pos1[],
                              int Num,
                              Matrix3d& H,
                              int Max_Iter = Max_Iteration,
                              double Conv_EPS = Convergence_EPS,
                              double f0 = Default_f0);

   // Least-Squares with Hyper Accuracy
   bool hyper_least_squares(Vector2d pos0[],
                            Vector2d pos1[],
                            int Num,
                            Matrix3d& H,
                            double f0 = Default_f0);

   // FNS core routine
   bool fns_core(Vector9d xi[][3],
                 Matrix94d Ta[][3],
                 Matrix3d Wa[],
                 int Num,
                 Vector9d& theta,
                 Matrix9d& M,
                 int Max_Iter,
                 double Conv_EPS);

   // FNS method
   bool fns(Vector2d pos0[],
            Vector2d pos1[],
            int Num,
            Matrix3d& H,
            int Max_Iter = Max_Iteration,
            double Conv_EPS = Convergence_EPS,
            double f0 = Default_f0);
      
   // Geometric distance minimization
   bool geometric_distance_minimization(Vector2d pos0[],
                                        Vector2d pos1[],
                                        int Num,
                                        Vector9d &theta,
                                        int Max_Iter = Max_Iteration,
                                        double Conv_EPS = Convergence_EPS,
                                        double f0 = Default_f0);

   // Geometric distance minimization
   bool geometric_distance_minimization(Vector2d pos0[],
                                        Vector2d pos1[],
                                        int Num,
                                        Matrix3d &H,
                                        int Max_Iter = Max_Iteration,
                                        double Conv_EPS = Convergence_EPS,
                                        double f0 = Default_f0);

   // Hyperaccurate correction 
   bool hyperaccurate_correction(Vector2d pos0[],
                                 Vector2d pos1[],
                                 int Num,
                                 Matrix3d& H,
                                 int Max_Iter = Max_Iteration,
                                 double Conv_EPS = Convergence_EPS,
                                 double f0 = Default_f0);

   // RANSAC method
   bool ransac(Vector2d pos0[],
               Vector2d pos1[],
               int Num,
               Matrix3d &H,
               bool inlier[],
               unsigned long Seed = MT_Seed,
               double dist = Distance_Threshold,
               int Max_Count = Max_Sample_Count,
               double f0 = Default_f0);
}

#endif // _HOMOGRAPHY_COMPUTATION_H_
