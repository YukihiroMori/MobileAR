//
// Check solution by points
//

#include "PlaneReconstruction.h"
#include "Triangulation.h"

// Triangulation
bool
PlaneReconstruction::check_solution(
   Vector2d pos0[],
   Vector2d pos1[],
   int Num,
   const Vector3d& tr,
   const Matrix3d& R,
   double fl0,
   double fl1,
   Vector3d Xp[],
   double f0
)
{
   Matrix34d    P0, P1, tP0, tP1;
   DiagMatrix3d F0(fl0, fl0, f0);
   DiagMatrix3d F1(fl1, fl1, f0);
   bool         flag;

   // Common initialization
   tP0.block<3,3>(0,0) = I3;
   tP0.block<3,1>(0,3) = ZeroVec3;

   // Projection matrices
   tP1.block<3,3>(0,0) = R.transpose();
   tP1.block<3,1>(0,3) = - R.transpose() * tr;

   P0 = F0 * tP0;
   P1 = F1 * tP1;

   // Triangulation by solution 1
   Triangulation::least_squares(pos0, pos1, Num, P0, P1, Xp, f0);

   // check
   flag = true;
   for (int i = 0; i < Num; i++)
   {
      if (Xp[i](2) < 0.0)
         flag = false;
   }

   return flag;
}

bool
PlaneReconstruction::choose_solution_by_points(
   Vector2d pos0[], Vector2d pos1[],
   int Num,
   const Vector3d& tp, const Matrix3d& Rp,
   const Vector3d& tm, const Matrix3d& Rm,
   double fl0,
   double fl1,
   bool check[],
   double f0
)
{
   Vector3d     Xp[Num];
   int ct;

   // reset flags
   for (int i = 0; i < 4; i++)
      check[i] = false;

   // Triangulation by solution 1: np, tp, and Rp
   if (check_solution(pos0, pos1, Num, tp, Rp, fl0, fl1, Xp, f0))
      check[0] = true;

   // Triangulation by solution 2: nm, tm, and Rm
   if (check_solution(pos0, pos1, Num, tm, Rm, fl0, fl1, Xp, f0))
      check[1] = true;

   // Triangulation by solution 3: -np, -tp, and Rp
   if (check_solution(pos0, pos1, Num, -tp, Rp, fl0, fl1, Xp, f0))
      check[2] = true;

   // Triangulation by solution 4: -nm, -tm, and Rm
   if (check_solution(pos0, pos1, Num, -tm, Rm, fl0, fl1, Xp, f0))
      check[3] = true;

   // check
   ct = 0;
   for (int i = 0; i < 4; i++)
      if (check[i]) ct++;

   // multiple solution
   if (ct != 1)  return false;

   return true;
}
