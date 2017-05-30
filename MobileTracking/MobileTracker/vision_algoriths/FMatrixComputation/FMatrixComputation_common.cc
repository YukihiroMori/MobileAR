//
// Common functions for Fundamental Matrix Computation
//

#include "MT.h"
#include "FMatrixComputation.h"

using namespace Eigen;

// Convert two 2-vectors into 9-vector
Vector9d
FMatrixComputation::convert_vec9(const Vector2d &p0, const Vector2d &p1, double f0)
{
   Vector9d t;

   t <<
      p0(0)*p1(0), p0(0)*p1(1), f0*p0(0),
      p0(1)*p1(0), p0(1)*p1(1), f0*p0(1),
      f0*p1(0), f0*p1(1), f0*f0;

   return t;
}

// Convert four 2-vectors into 9-vector
Vector9d
FMatrixComputation::convert_vec9_st(const Vector2d &p0h, const Vector2d &p1h,
                                    const Vector2d &p0t, const Vector2d &p1t,
                                    double f0)
{
   Vector9d t;

   t <<
      p0h(0)*p1h(0) + p1h(0)*p0t(0) + p0h(0)*p1t(0),
      p0h(0)*p1h(1) + p1h(1)*p0t(0) + p0h(0)*p1t(1),
      f0*(p0h(0) + p0t(0)),
      p0h(1)*p1h(0) + p1h(0)*p0t(1) + p0h(1)*p1t(0),
      p0h(1)*p1h(1) + p1h(1)*p0t(1) + p0h(1)*p1t(1),
      f0*(p0h(1) + p0t(1)),
      f0*(p1h(0) + p1t(0)),
      f0*(p1h(1) + p1t(1)),
      f0*f0;

   return t;
}

// Make covariance matrix from two 2-vectors
Matrix9d
FMatrixComputation::make_cov_mat(const Vector2d &p0, const Vector2d &p1, double f0)
{
   Matrix9d V0 = ZeroMat9;
   double f02 = sqr(f0);

   V0(0,0) = sqr(p0(0)) + sqr(p1(0));
   V0(1,1) = sqr(p0(0)) + sqr(p1(1));
   V0(2,2) = V0(5,5) = V0(6,6) = V0(7,7) = f02;
   V0(3,3) = sqr(p0(1)) + sqr(p1(0));
   V0(4,4) = sqr(p0(1)) + sqr(p1(1));

   V0(0,1) = V0(1,0) = V0(3,4) = V0(4,3) = p1(0) * p1(1);
   V0(0,2) = V0(2,0) = V0(3,5) = V0(5,3) = f0 * p1(0);
   V0(0,3) = V0(3,0) = V0(1,4) = V0(4,1) = p0(0) * p0(1);
   V0(0,6) = V0(6,0) = V0(1,7) = V0(7,1) = f0 * p0(0);
   V0(1,2) = V0(2,1) = V0(4,5) = V0(5,4) = f0 * p1(1);
   V0(3,6) = V0(6,3) = V0(4,7) = V0(7,4) = f0 * p0(1);

   return V0;
}

// convert 9-vector to 33-matrix
Matrix3d
FMatrixComputation::convert_mat3(const Vector9d &th)
{
   Matrix3d F;

   F << th(0), th(1), th(2), th(3), th(4), th(5), th(6), th(7), th(8);

   return F;
}

// convert 33-matrix to 9-vector
Vector9d
FMatrixComputation::convert_vec9(const Matrix3d &F)
{
   Vector9d theta;

   theta << F(0,0), F(0,1), F(0,2),
            F(1,0), F(1,1), F(1,2),
            F(2,0), F(2,1), F(2,2);

   return theta;
}

// convert theta^\dag
Vector9d
FMatrixComputation::theta_dag(const Vector9d &th)
{
   Vector9d     thdag;
   
   thdag <<
      th(4) * th(8) - th(7) * th(5),
      th(5) * th(6) - th(8) * th(3),
      th(3) * th(7) - th(6) * th(4),
      th(7) * th(2) - th(1) * th(8),
      th(8) * th(0) - th(2) * th(6),
      th(6) * th(1) - th(0) * th(7),
      th(1) * th(5) - th(4) * th(2),
      th(2) * th(3) - th(5) * th(0),
      th(0) * th(4) - th(3) * th(1);

   return thdag;
}

// Randomly sample unique indices
// The seed does not set in this function.
namespace {     // unnamed namespace
   bool
   check_unique(int idx[], int Num, int x)
   {
      for (int i = 0; i < Num; i++)
      {
         if (idx[i] == x)
            return false;
      }

      return true;
   }
}

void
FMatrixComputation::sample_unique_indices(
   int idx[],           // index array:                 OUTPUT
   int SampleNum,       // Number of sample indices:    INPUT
   int DataNum          // Number of Data:              INPUT
                                     )
{
   int  x;

   for (int i = 0; i < SampleNum; i++)
   {
      // check
      do
      {
         x =  (int)(DataNum * genrand_real2());
      }
      while (!check_unique(idx, i, x));

      idx[i] = x;
   }
}
