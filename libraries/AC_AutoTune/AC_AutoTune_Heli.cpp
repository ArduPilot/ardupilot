/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for autotune of helicopters. Based on original autotune code from ArduCopter, written by Leonard Hall
  Converted to a library by Andrew Tridgell, and rewritten to include helicopters by Bill Geyer
 */

#define AUTOTUNE_HELI_TARGET_ANGLE_RLLPIT_CD     2000   // target roll/pitch angle during AUTOTUNE FeedForward rate test
#define AUTOTUNE_HELI_TARGET_RATE_RLLPIT_CDS     5000   // target roll/pitch rate during AUTOTUNE FeedForward rate test
#define AUTOTUNE_FFI_RATIO_FOR_TESTING     0.5f     // I is set 2x smaller than VFF during testing
#define AUTOTUNE_FFI_RATIO_FINAL           0.5f     // I is set 0.5x VFF after testing
#define AUTOTUNE_PI_RATIO_FINAL            1.0f     // I is set 1x P after testing
#define AUTOTUNE_YAW_PI_RATIO_FINAL        0.1f     // I is set 1x P after testing
#define AUTOTUNE_RD_STEP                  0.0005f     // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                  0.005f     // minimum increment when increasing/decreasing Rate P term
#define AUTOTUNE_SP_STEP                  0.05f     // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1f     // I is set 10x smaller than P during testing
#define AUTOTUNE_RD_MAX                  0.020f     // maximum Rate D value
#define AUTOTUNE_RLPF_MIN                  1.0f     // minimum Rate Yaw filter value
#define AUTOTUNE_RLPF_MAX                  20.0f     // maximum Rate Yaw filter value
#define AUTOTUNE_RP_MIN                   0.01f     // minimum Rate P value
#define AUTOTUNE_RP_MAX                    2.0f     // maximum Rate P value
#define AUTOTUNE_SP_MAX                   10.0f     // maximum Stab P value
#define AUTOTUNE_SP_MIN                    0.5f     // maximum Stab P value
#define AUTOTUNE_D_UP_DOWN_MARGIN          0.2f     // The margin below the target that we tune D in

#include "AC_AutoTune_Heli.h"

