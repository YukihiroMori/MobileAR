//
// Check solution by parameters
//

#include "PlaneReconstruction.h"

// choose by parameters
bool
PlaneReconstruction::choose_solution_by_parameters(
   const Vector3d& np, const Vector3d& nm,
   double dist,
   const Matrix3d& Rp, const Matrix3d& Rm,
   bool cam1, bool cam2,
   bool check[]
)
{
   Vector3d Rnp, Rnm;
   int c;

   bool check1[4];
   bool check2[4];

   // clear flags
   for (int i = 0; i < 4; i++)
      check[i] = check1[i] = check2[i] = false;

   // camera1 check
   if (cam1)
   { // case1
      if (np(2) > 0)
         check1[0] = true; // 1st solution
      else
         check1[2] = true; // 3rd solution

      if (nm(2) > 0)
         check1[1] = true; // 2nd solution
      else
         check1[3] = true; // 4th solution
   }
   else
   { // case2
      if (np(2) < 0)
         check1[0] = true; // 1st solution
      else
         check1[2] = true; // 3rd solution

      if (nm(2) < 0)
         check1[1] = true; // 2nd solution
      else
         check1[3] = true; // 4th solution
   }

   // camera2 check
   Rnp = Rp.transpose() * np;
   Rnm = Rm.transpose() * nm;
   if (cam2)
   { // case1
      if (Rnp(2) > 0)
         check2[0] = true; // 1st solution
      else
         check2[2] = true; // 3rd solution

      if (Rnm(2) > 0)
         check2[1] = true; // 2nd solution
      else
         check2[3] = true; // 4th solution
   }
   else
   { // case2
      if (Rnp(2) < 0)
         check2[0] = true; // 1st solution
      else
         check2[2] = true; // 3rd solution

      if (Rnm(2) < 0)
         check2[1] = true; // 2nd solution
      else
         check2[3] = true; // 4th solution
   }

   // check solutions
   // count # of solutions
   c = 0;
   for (int i = 0; i < 4; i++) {
      check[i] = check1[i] && check2[i];
      if (check[i]) c++;
   }

   // multiple solutions
   if (c != 1) return false;

   return true;
}
