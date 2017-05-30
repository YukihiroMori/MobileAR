//
// Intersections of two ellipses
//

#include <iostream>
#include "EllipseAnalysis.h"

using namespace Eigen;

// unnamed namespace for computations
namespace {
// compute adjugate matrix (there is no function in Eigen)
   Matrix3d
   adjugate(const Matrix3d& A)
   {
      Matrix3d Adag;

      Adag(0,0) = A(1,1)*A(2,2) - A(2,1)*A(1,2);
      Adag(0,1) = A(2,1)*A(0,2) - A(0,1)*A(2,2);
      Adag(0,2) = A(0,1)*A(1,2) - A(1,1)*A(0,2);

      Adag(1,0) = A(2,0)*A(1,2) - A(1,0)*A(2,2);
      Adag(1,1) = A(0,0)*A(2,2) - A(2,0)*A(0,2);
      Adag(1,2) = A(1,0)*A(0,2) - A(0,0)*A(1,2);

      Adag(2,0) = A(1,0)*A(2,1) - A(2,0)*A(1,1);
      Adag(2,1) = A(2,0)*A(0,1) - A(0,0)*A(2,1);
      Adag(2,2) = A(0,0)*A(1,1) - A(1,0)*A(0,1);

      return Adag;
   }
   
// solve cubic equation
   void
   set_coefficients(const Matrix3d& Q1, const Matrix3d& Q2, Vector4d& a)
   {
      Matrix3d T1, T2;

      T1 = adjugate(Q1) * Q2;
      T2 = Q1 * adjugate(Q2);
   
//      a(0) = Q1.determinant();
//      a(1) = T1.trace();
//      a(2) = T2.trace();
//      a(3) = Q2.determinant();
      a(3) = Q1.determinant(); // index shows the order
      a(2) = T1.trace();
      a(1) = T2.trace();
      a(0) = Q2.determinant();
   }

// solve cubic equation
   bool
   solve_cubic_equation(const Vector4d& coeff,
                        double &x1, double &x2, double &x3 )
   {
      double a = coeff(2)/coeff(3);
      double b = coeff(1)/coeff(3);
      double c = coeff(0)/coeff(3);
      double Q, R ;

      // compute Discriminant
      Q = (a*a - 3.0*b)/9.0;
      R = (2.0*a*a*a - 9.0*a*b + 27.0*c)/54.0;

      // CheckCount the number of real roots
      if ((Q*Q*Q - R*R) >= 0.0)
      {
         // Three real roots
         double theta = acos(R / sqrt(Q*Q*Q));
         x1 = -2.0 * sqrt(Q) * cos(theta/3.0) - (a/3.0);
         x2 = -2.0 * sqrt(Q) * cos((theta + 2.0*M_PI)/3.0) - (a/3.0);
         x3 = -2.0 * sqrt(Q) * cos((theta + 4.0*M_PI)/3.0) - (a/3.0);

         // sorting roots (x1 >= x2 >= x3)
         if (x1 < x2 && x2 < x3)
            swap(&x1, &x3) ;
         else if (x1 < x2 && x2 > x3)
         {
            if (x1 < x3)
            {
               swap(&x1, &x2);
               swap(&x2, &x3);
            }
            else
               swap(&x1, &x2);
         }
         else if (x1 > x2 && x2 < x3)
         {
            if (x1 < x3)
            {
               swap(&x2, &x3);
               swap(&x1, &x2);
            }
            else
               swap(&x2, &x3);
         }
      }
      else
      { // only one root
         if (R < 0.0)
         {
            x1 = (cbrt(sqrt((R*R)-(Q*Q*Q))+fabs(R))
                  + (Q / cbrt(sqrt((R*R)-(Q*Q*Q))+fabs(R))))
               - (a/3.0);
         }
         else
         {
            x1 = -(cbrt(sqrt((R*R)-(Q*Q*Q))+fabs(R))
                   + (Q / cbrt(sqrt((R*R)-(Q*Q*Q))+fabs(R))))
               - (a/3.0);
         }
         x2 = x3 = 0.0 ;
      }
   }

// compute two lines from Q
   bool
   compute_two_lines(const Matrix3d& Q,
                     Vector3d& n1,
                     Vector3d& n2,
                     double f0
   )
   {
      double B2AC;
   
      B2AC = sqr(Q(0,1)) - Q(0,0)*Q(1,1);

      if (B2AC <= 0.0)  return false;

      // coefficients for two lines
      n1(0) = n2(0) = Q(0,0);
      n1(1) = Q(0,1) - sqrt(B2AC);
      n2(1) = Q(0,1) + sqrt(B2AC);
      n1(2) = f0 * (Q(0,2) - (Q(0,1)*Q(0,2) - Q(0,0)*Q(1,2)) / sqrt(B2AC));
      n2(2) = f0 * (Q(0,2) + (Q(0,1)*Q(0,2) - Q(0,0)*Q(1,2)) / sqrt(B2AC));

      n1.normalize();
      n2.normalize();

      return true;
   }

// compute intersections
   bool
   compute_intersections(const Matrix3d& Q, const Vector3d& n,
                         Vector2d& P1, Vector2d& P2, double f0)
   {
      double       a, b, c;

      if (fabs(n(1)) >= fabs(n(0)))
      {
         double q, x1, x2;
      
         // set coefficients
         a = Q(0,0)*sqr(n(1)) - 2.0*Q(0,1)*n(0)*n(1) + Q(1,1)*sqr(n(0));
         b = 2.0*f0*(Q(2,0)*sqr(n(1)) + Q(1,1)*n(0)*n(2) - Q(0,1)*n(1)*n(2)
                     - Q(1,2)*n(0)*n(1));
         c = sqr(f0)*(Q(1,1)*sqr(n(2)) - 2.0*Q(1,2)*n(1)*n(2)
                      + Q(2,2)*sqr(n(1)));

         // solve w.r.t. x
         q = - (b + sgn(b) * sqrt(sqr(b) - 4.0*a*c)) / 2.0;
         P1(0) = q/a;
         P2(0) = c/q;
         P1(1) = - (n(0)*P1(0) + n(2)*f0)/n(1);
         P2(1) = - (n(0)*P2(0) + n(2)*f0)/n(1);
      }
      else
      {
         double q, y1, y2;

         // set coefficients
         a = Q(0,0)*sqr(n(1)) - 2.0*Q(0,1)*n(0)*n(1) + Q(1,1)*sqr(n(0));
         b = 2.0*f0*(Q(1,2)*sqr(n(0)) + Q(0,0)*n(1)*n(2) - Q(0,1)*n(0)*n(2)
                     - Q(0,2)*n(0)*n(1));
         c = sqr(f0)*(Q(0,0)*sqr(n(2)) - 2.0*Q(0,2)*n(0)*n(2)
                      + Q(2,2)*sqr(n(0)));

         // solve w.r.t y
         q = - (b + sgn(b) * sqrt(sqr(b) - 4.0*a*c)) / 2.0;
         P1(1) = q/a;
         P2(1) = c/q;
         P1(0) = - (n(1)*P1(1) + n(2)*f0)/n(0);
         P2(0) = - (n(1)*P2(1) + n(2)*f0)/n(0);
      }

      return true;
   }
}
// end of unnamed namespace

// computation of itersection of two ellipses
bool
EllipseAnalysis::ellipse_intersections(
   const Matrix3d& Q1,  // 1st Ellipse parameter:       INPUT
   const Matrix3d& Q2,  // 2nd Ellipse parameter:       INPUT
   Vector2d& Pa,        // 1st intersection:            OUTPUT
   Vector2d& Pb,        // 2nd intersection:            OUTPUT
   Vector2d& Pc,        // 3rd intersection:            OUTPUT
   Vector2d& Pd,        // 4th intersection:            OUTPUT
   double f0            // Default focal length:        INPUT w. default
)
{
   Matrix3d     Q;
   double       x1, x2, x3;
   Vector4d     coeff;
   Vector3d     n1, n2;

   // Solve cubic equation |lambda*Q1 + Q2| = 0
   set_coefficients(Q1, Q2, coeff);
   solve_cubic_equation(coeff, x1, x2, x3);

   // solutions
//   std::cerr << "x1 = " << x1 << std::endl;
//   std::cerr << "x2 = " << x2 << std::endl;
//   std::cerr << "x3 = " << x3 << std::endl;

   // Compute 2 lines
   Q = x1*Q1 + Q2;
   if (compute_two_lines(Q, n1, n2, f0))
   {
      // Compute intersections and return
      compute_intersections(Q1, n1, Pa, Pb, f0);
      compute_intersections(Q1, n2, Pc, Pd, f0);
      return true;
   }
   
   // Compute 2 lines from x2
   Q = x2*Q1 + Q2;
   if (compute_two_lines(Q, n1, n2, f0))
   {
      // Compute intersections and return
      compute_intersections(Q1, n1, Pa, Pb, f0);
      compute_intersections(Q1, n2, Pc, Pd, f0);
      return true;
   }

   // Compute 2 lines from x3
   Q = x3*Q1 + Q2;
   if (compute_two_lines(Q, n1, n2, f0))
   {
      // Compute intersections and return
      compute_intersections(Q1, n1, Pa, Pb, f0);
      compute_intersections(Q1, n2, Pc, Pd, f0);
      return true;
   }

   return false;
}
