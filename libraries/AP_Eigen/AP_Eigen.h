#ifndef AP_EIGEN_H
#define AP_EIGEN_H
/*
#include <cstdlib>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD != HAL_BOARD_PX4
    #include <omp.h>
#endif
*/
/*
 * Eigen interface extensions
 */
#define EIGEN_MATRIXBASE_PLUGIN <AP_Eigen/AP_BaseAddons.h>
#define EIGEN_MATRIX_PLUGIN     <AP_Eigen/AP_MatrixAddons.h>
    
/*
 * The PX4 stack creates a lot of drama if just using C99 math features :D
 */
#pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wshadow"
    #pragma GCC diagnostic ignored "-Wlogical-op"       // there are a lot of warnings together with an ARM compiler

    // C99 std lib additions
    #include "AP_C99Math4PX4.h"                         // there none-eabi for ARM is not C99 complete (important for PX4 only)
    
    #include "../../modules/eigen/Eigen/Core"
    #include "../../modules/eigen/Eigen/Dense"
#pragma GCC diagnostic pop

/*
 * Some general math functions and converter classes :D
 */
#include <AP_Eigen/AP_Compat.h>     // Type converters for ArduPilot types
#include <AP_Eigen/AP_Algebra.h>    // Replacement for AP_Math

#endif //AP_INTF_H