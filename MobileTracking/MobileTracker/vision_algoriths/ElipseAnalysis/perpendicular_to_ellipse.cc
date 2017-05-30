//
// The foot of perpendicular line of ellipse
//

#include <iostream>
#include "EllipseAnalysis.h"

using namespace Eigen;

// The foot of perpendicular line of ellipse
bool
EllipseAnalysis::perpendicular_to_ellipse(
   const Matrix3d& Q,   // Ellipse parameter:           INPUT
   const Vector2d& p,   // Point not on the ellipse:    INPUT
   Vector2d& q,         // Point on the ellipse:        OUTPUT
   double f0            // Default focal length:        INPUT w. default
)
{
   Matrix3d     D;
   Vector2d     qq[4];
   double       mindist, d;

   // set elements of D
   D(0,0) = Q(0,1);
   D(1,1) = -Q(0,1);
   D(0,1) = D(1,0) = (Q(1,1) - Q(0,0))/2.0;
   D(0,2) = D(2,0) = (Q(0,0)*p(1) - Q(0,1)*p(0) + Q(1,2)*f0) / (2.0*f0);
   D(1,2) = D(2,1) = (Q(0,1)*p(1) - Q(1,1)*p(0) - Q(0,2)*f0) / (2.0*f0);
   D(2,2) = (Q(0,2)*p(1) - Q(1,2)*p(0))/f0;

   // compute intersections
   if (!ellipse_intersections(Q, D, qq[0], qq[1], qq[2], qq[3], f0))
   {
      return false;
   }


   // find the nearest one
   mindist = Large_Number;
   for (int i = 0; i < 4; i++)
   {
      if (sqr(qq[i](0) - p(0)) + sqr(qq[i](1) - p(1)) < mindist)
      {
         q(0) = qq[i](0);
         q(1) = qq[i](1);
      }
   }
   return true;
}
