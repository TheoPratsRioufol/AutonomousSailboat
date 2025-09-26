#include "udf.h"


DEFINE_SDOF_PROPERTIES(boat, prop, dt, time, dtime)
{
   prop[SDOF_MASS] = 22;
   prop[SDOF_IXX]  = 30;
   prop[SDOF_IYY]  = 30;
   prop[SDOF_IZZ]  = 30;

   prop[SDOF_ZERO_TRANS_X] = TRUE;
   prop[SDOF_ZERO_TRANS_Y] = TRUE;
   prop[SDOF_ZERO_TRANS_Z] = TRUE;

   prop[SDOF_ZERO_ROT_X] = TRUE;
   prop[SDOF_ZERO_ROT_Y] = FALSE;
   prop[SDOF_ZERO_ROT_Z] = TRUE;

   printf ("\nhull_6dof: Updated 6DOF properties");
}
