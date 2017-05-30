// unnamed namespace for tiny functions

#ifndef _ELLIPSE_COMMON_H_
#define _ELLIPSE_COMMON_H_

namespace {
   // square
   inline double sqr(double x) { return x*x; }
   // swap 
   inline void swap(double *x, double *y)
   {
      double t = *x;
      *x = *y;
      *y = t;
   }
   // sign function
   double sgn(double x)
   {       
      if (x >= 0.0) return 1.0;
      else          return -1.0;
   }
}
// end of unnamed namespace  

#endif // _ELLIPSE_COMMON_H_
