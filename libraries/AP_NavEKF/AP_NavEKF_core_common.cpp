/*
  NavEKF_core_common holds scratch data shared by EKF2 and EKF3

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
#include "AP_NavEKF_core_common.h"

NavEKF_core_common::Matrix24 NavEKF_core_common::KH;
NavEKF_core_common::Matrix24 NavEKF_core_common::KHP;
NavEKF_core_common::Matrix24 NavEKF_core_common::nextP;
NavEKF_core_common::Vector28 NavEKF_core_common::Kfusion;

/*
  fill common scratch variables, for detecting re-use of variables between loops in SITL
 */
void NavEKF_core_common::fill_scratch_variables(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // fill the common variables with NaN, so we catch any cases in
    // SITL where they are used without initialisation. These are all
    // supposed to be scratch variables that are not used between
    // iterations
    fill_nanf(&KH[0][0], sizeof(KH)/sizeof(ftype));
    fill_nanf(&KHP[0][0], sizeof(KHP)/sizeof(ftype));
    fill_nanf(&nextP[0][0], sizeof(nextP)/sizeof(ftype));
    fill_nanf(&Kfusion[0], sizeof(Kfusion)/sizeof(ftype));
#endif
}
