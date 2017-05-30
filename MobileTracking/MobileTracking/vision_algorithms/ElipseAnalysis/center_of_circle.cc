//
// The center of circle
//

#include "EllipseAnalysis.h"

using namespace Eigen;

// compute the center of a circle
bool EllipseAnalysis::center_of_circle(
   const Matrix3d& Q,           // Ellipse parameter:           INPUT
   const Vector3d& normal,      // Normal of supporting plane:  INPUT
   Vector2d& center,            // Center of circle:            OUTPUT
   double f,                    // Focal length:                INPUT
   double f0                    // Default focal length:        INPUT w. default
)
{
   DiagMatrix3d D(f0, f0, f);
   Vector3d     m;
   double       lmd1, lmd2, lmd3;

   // compute the vector m
   m = D * Q.inverse() * D * normal;

   // center of the circle
   center(0) = f * m(0) / m(2);
   center(1) = f * m(1) / m(2);
   
   return true;
}
