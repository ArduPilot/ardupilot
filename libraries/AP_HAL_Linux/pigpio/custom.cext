/*
This version is for pigpio version 26+

If you want customised functions replace this file with your own
definitions for gpioCustom1 and gpioCustom2.
*/

#include "pigpio.h"

int gpioCustom1(unsigned arg1, unsigned arg2, char *argx, unsigned count)
{
   int i;
   unsigned max;

   DBG(DBG_USER, "arg1=%d arg2=%d count=%d [%s]",
      arg1, arg2, count, myBuf2Str(count, argx));

   CHECK_INITED;

   /* for dummy just return max parameter */

   if (arg1 > arg2) max = arg1; else max = arg2;

   for (i=0; i<count; i++) if (argx[i] > max) max = argx[i];

   return max;
}


int gpioCustom2(unsigned arg1, char *argx, unsigned count,
                char *retBuf, unsigned retMax)
{
   int i, j, t;

   DBG(DBG_USER, "arg1=%d count=%d [%s] retMax=%d",
      arg1, count, myBuf2Str(count, argx), retMax);

   CHECK_INITED;

   /* for dummy just return argx reversed */

   if (count > retMax) count = retMax;

   for (i=0, j=count-1; i<=j; i++, j--)
   {
      /* t used as argx and retBuf may be the same buffer */
      t = argx[i];
      retBuf[i] = argx[j];
      retBuf[j] = t;
   }

   return count;
}

