#include "udf.h"


DEFINE_SDOF_PROPERTIES(hull_user_udf, prop, dt, time, dtime)
{
   prop[SDOF_MASS] = 45;
   prop[SDOF_IXX]  = 30;
   prop[SDOF_IYY]  = 30;
   prop[SDOF_IZZ]  = 30;

   prop[SDOF_ZERO_TRANS_X] = TRUE;
   prop[SDOF_ZERO_TRANS_Y] = TRUE;
   prop[SDOF_ZERO_TRANS_Z] = FALSE;

   prop[SDOF_ZERO_ROT_X] = TRUE;
   prop[SDOF_ZERO_ROT_Y] = FALSE;
   prop[SDOF_ZERO_ROT_Z] = TRUE;

   printf ("\nhull_6dof: Updated 6DOF properties");
}
