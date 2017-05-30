//
// Ellipse Analysis and 3-D Reconstruction of circle
//
#ifndef _EllipseAnalysis_H_
#define _EllipseAnalysis_H_

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

using namespace Eigen;

typedef DiagonalMatrix<double,3> DiagMatrix3d;

#include "EllipseCommon.h"

// Namespace EllipseAnalysis
namespace EllipseAnalysis {

   // constants for defualt values
   const double Default_f0 = 600.0;
   const double Large_Number = 1e99;
   const double Almost_Zero = 1e-25;

   // computation of itersection of two ellipses
   bool ellipse_intersections(const Matrix3d& Q1,
                              const Matrix3d& Q2,
                              Vector2d& Pa,
                              Vector2d& Pb,
                              Vector2d& Pc,
                              Vector2d& Pd,
                              double f0 = Default_f0
   );

   // The foot of perpendicular line of ellipse
   bool perpendicular_to_ellipse(const Matrix3d& Q,
                                 const Vector2d& p,
                                 Vector2d& q,
                                 double f0 = Default_f0
   );

   // reconstruct the supporting plane of a circle
   bool supporting_plane_reconstruction(const Matrix3d& Q,
                                        double radius,
                                        Vector3d& normal,
                                        double *dist,
                                        double f,
                                        double f0 = Default_f0
   );

   // compute the center of a circle
   bool center_of_circle(const Matrix3d& Q,
                         const Vector3d& normal,
                         Vector2d& center,
                         double f,
                         double f0 = Default_f0
   );

   // convert frontal image
   bool front_image_of_the_circle(const Matrix3d& Q,
                                  double radius,
                                  cv::Mat& simg,
                                  cv::Mat& dimg,
                                  double f,
                                  double f0
   );

}
// end of namepsace EllipseAnalysis

#endif // _EllipseAnalysis_H_
