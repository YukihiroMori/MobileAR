//
// Convert to frontal image of circle
//

#include <iostream>

// OpenCV 2
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Eigen and local headers
#include "EllipseAnalysis.h"

using namespace Eigen;

// unnamed namespace
namespace {
// Calc Rotation matrix from n1 to n2
   Matrix3d
   rotation_matrix(const Vector3d& n1,
                   const Vector3d& n2)
   {
      Matrix3d	R;
      Vector3d	n;
      double	th, co, si, co1;

      // rotation axis
      n = n1.cross(n2);

      // rotation angle
      th = asin(n.norm());
      co = cos(th);
      si = sin(th);
      co1 = 1.0 - co;

      // noralize rotation axis
      n.normalize();

      // rotation matrix from Rodrigues's formula
      R <<
         co + n(0)*n(0)*co1,      n(0)*n(1)*co1 - n(2)*si, n(0)*n(2)*co1 + n(1)*si,
         n(1)*n(0)*co1 + n(2)*si, co + n(1)*n(1)*co1,      n(1)*n(2)*co1 - n(0)*si,
         n(2)*n(0)*co1 - n(1)*si, n(2)*n(1)*co1 + n(0)*si, co + n(2)*n(2)*co1;
    
      return R;
   }

// interpolation of pixels
   int
   get_pixels(const cv::Mat& simg,
              double su, double sv, int ch)
   {
      int	su0, su1, sv0, sv1;
      double    ddu, ddv;
      double    val00, val01, val10, val11, val;
      int       ival;
    
      // interpolation
      su0 = (int)floor(su);
      su1 = (int)ceil(su);
      sv0 = (int)floor(sv);
      sv1 = (int)ceil(sv);
      ddu = su - su0;
      ddv = sv - sv0;

      if (su0 < 0 || sv0 < 0 || su1 > simg.cols || sv1 > simg.rows)
         return 0;

      val00 = simg.data[sv0 * simg.step + su0 * simg.elemSize() + ch];
      val01 = simg.data[sv0 * simg.step + su1 * simg.elemSize() + ch];
      val10 = simg.data[sv1 * simg.step + su0 * simg.elemSize() + ch];
      val11 = simg.data[sv1 * simg.step + su1 * simg.elemSize() + ch];

      ival = (int)((1.0 - ddu) * ((1.0 - ddv) * val00 + ddv * val01)
                   + ddu * ((1.0 - ddv) * val10 + ddv * val11)
                   + 0.5);

      if (ival < 0) ival = 0;
      else if (ival > 255) ival = 255;

      return ival;
   }

// (inverse) matrix for onverting frontal image
   bool set_convert_matrix(const Matrix3d& Q, double radius,
                           Matrix3d& Hinv, double f, double f0)
   {
      Matrix3d	R, H, H0;
      Vector3d	normal, k, c, cp;
      Vector2d	c2;
      double	dist;
      DiagMatrix3d	D(f0, f0, f);
      DiagMatrix3d	D1(1.0/f0, 1.0/f0, 1.0/f);

      // compue support plane
      if (!EllipseAnalysis::supporting_plane_reconstruction(Q, radius, normal, &dist, f, f0))
         return false;

      // compute center of circle
      if (!EllipseAnalysis::center_of_circle(Q, normal, c2, f, f0))
         return false;
  
      // Homography corresponding to the rotation matrix from normal to z-axis
      k << 0.0, 0.0, 1.0;
      R = rotation_matrix(normal, k);
      H = D1 * R.transpose() * D;

      // center of circle in new image
      c << c2(0)/f0, c2(1)/f0, 1.0;
      cp = H * c;
      
      // Homography of translating the new center to the origin
      H0 << 1.0, 0.0, -cp(0)/cp(2),
         0.0, 1.0, -cp(1)/cp(2),
         0.0, 0.0, 1.0;

      // Inverse of whole transformation
      Hinv = H.inverse() * H0.inverse();

      return true;
   }

// convert image
   bool convert_image(const Matrix3d& Hinv,
                      cv::Mat& simg, cv::Mat& dimg, double f0)
   {
      int		su, sv, du, dv;
      int		su0, su1, sv0, sv1;
      int		suc, svc, duc, dvc;
      double	sx, sy, dx, dy;
      Vector3d	sp, dp;
      int		ch;
      int		R, G, B, grey;
      Matrix3d          H;

      // phisical centers of images
      suc = simg.cols / 2;
      svc = simg.rows / 2;
      duc = dimg.cols / 2;
      dvc = dimg.rows / 2;

      if ((ch = simg.channels()) != dimg.channels())
         return false;

      // convert pixels
      for (dv = 0; dv < dimg.rows; dv++)
      {
         // convert (u,v) to (x,y)
         dx = dvc - dv;
         for (du = 0; du < dimg.cols; du++)
         {
            // convert (u,v) to (x,y) in dimg
            dy = du - duc;
            dp << dx / f0, dy / f0, 1.0;
            
            // source image point
            sp = Hinv * dp;

            sx = f0 * sp(0) / sp(2);
            sy = f0 * sp(1) / sp(2);
            // convert (x,y) to (u,v) in simg
            su = sy + suc;
            sv = svc - sx;

            // color or monochrome
            if (ch == 3)
	    {
               dimg.data[dv * dimg.step + du * dimg.elemSize()]
                  = get_pixels(simg, su, sv, 0);
               dimg.data[dv * dimg.step + du * dimg.elemSize() + 1]
                  = get_pixels(simg, su, sv, 1);
               dimg.data[dv * dimg.step + du * dimg.elemSize() + 2]
                  = get_pixels(simg, su, sv, 2);
	    }
            else
               dimg.data[dv * dimg.step + du]
                  = get_pixels(simg, su, sv, 0);
         }
      }
   }

}
// end of unnamed namespace

// convert to frontal image
bool
EllipseAnalysis::front_image_of_the_circle(
   const Matrix3d& Q,   // Ellipse parameter:    INPUT
   double radius,       // Radius of circle:     INPUT
   cv::Mat& simg,       // Input image:          INPUT
   cv::Mat& dimg,       // Output front image:   OUTPUT
   double f,            // Focal length:         INPUT
   double f0            // Default focal length: INPUT w. default
)   
{
   Matrix3d	Hinv;

   // compute matrix of conversion
   if (!set_convert_matrix(Q, radius, Hinv, f, f0))
      return false;

   // convert image
   if (!convert_image(Hinv, simg, dimg, f0))
      return false;

   return true;
}
