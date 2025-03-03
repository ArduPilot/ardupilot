#include "Blimp.h"
#include <AC_AttitudeControl/AC_PosControl.h>

const AP_Param::GroupInfo Loiter::var_info[] = {
    // @Param: VELX_P
    // @DisplayName: X axis velocity controller P gain
    // @Description: X axis velocity controller P gain.  Corrects in proportion to the difference between the desired X velocity vs actual X velocity
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: VELX_I
    // @DisplayName: X axis velocity controller I gain
    // @Description: X axis velocity controller I gain.  Corrects long-term difference in desired X velocity vs actual X velocity
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: VELX_IMAX
    // @DisplayName: X axis velocity controller I gain maximum
    // @Description: X axis velocity controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: VELX_D
    // @DisplayName: X axis velocity controller D gain
    // @Description: X axis velocity controller D gain.  Compensates for short-term change in desired X velocity vs actual X velocity
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: VELX_FF
    // @DisplayName: X axis velocity controller feed forward
    // @Description: X axis velocity controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: VELX_FLTT
    // @DisplayName: X axis velocity controller target frequency in Hz
    // @Description: X axis velocity controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: VELX_FLTE
    // @DisplayName: X axis velocity controller error frequency in Hz
    // @Description: X axis velocity controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: VELX_FLTD
    // @DisplayName: X axis velocity controller derivative frequency in Hz
    // @Description: X axis velocity controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: VELX_SMAX
    // @DisplayName: X axis velocity slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: VELX_PDMX
    // @DisplayName: X axis velocity controller PD sum maximum
    // @Description: X axis velocity controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VELX_D_FF
    // @DisplayName: X axis velocity derivative feedforward gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: VELX_NTF
    // @DisplayName: X axis velocity target notch filter index
    // @Description: X axis velocity target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: VELX_NEF
    // @DisplayName: X axis velocity error notch filter index
    // @Description: X axis velocity error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_SUBGROUPINFO(pid_vel_x, "VELX_", 0, Loiter, AC_PID),

    // @Param: VELY_P
    // @DisplayName: Y axis velocity controller P gain
    // @Description: Y axis velocity controller P gain.  Corrects in proportion to the difference between the desired Y velocity vs actual Y velocity
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: VELY_I
    // @DisplayName: Y axis velocity controller I gain
    // @Description: Y axis velocity controller I gain.  Corrects long-term difference in desired Y velocity vs actual Y velocity
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: VELY_IMAX
    // @DisplayName: Y axis velocity controller I gain maximum
    // @Description: Y axis velocity controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: VELY_D
    // @DisplayName: Y axis velocity controller D gain
    // @Description: Y axis velocity controller D gain.  Compensates for short-term change in desired Y velocity vs actual Y velocity
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: VELY_FF
    // @DisplayName: Y axis velocity controller feed forward
    // @Description: Y axis velocity controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: VELY_FLTT
    // @DisplayName: Y axis velocity controller target frequency in Hz
    // @Description: Y axis velocity controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: VELY_FLTE
    // @DisplayName: Y axis velocity controller error frequency in Hz
    // @Description: Y axis velocity controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: VELY_FLTD
    // @DisplayName: Y axis velocity controller derivative frequency in Hz
    // @Description: Y axis velocity controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: VELY_SMAX
    // @DisplayName: Y axis velocity slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: VELY_PDMX
    // @DisplayName: Y axis velocity controller PD sum maximum
    // @Description: Y axis velocity controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VELY_D_FF
    // @DisplayName: Y axis velocity derivative feedforward gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: VELY_NTF
    // @DisplayName: Y axis velocity target notch filter index
    // @Description: Y axis velocity target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: VELY_NEF
    // @DisplayName: Y axis velocity error notch filter index
    // @Description: Y axis velocity error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_SUBGROUPINFO(pid_vel_y, "VELY_", 1, Loiter, AC_PID),

    // @Param: VELZ_P
    // @DisplayName: Z axis velocity controller P gain
    // @Description: Z axis velocity controller P gain.  Corrects in proportion to the difference between the desired Z velocity vs actual Z velocity
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: VELZ_I
    // @DisplayName: Z axis velocity controller I gain
    // @Description: Z axis velocity controller I gain.  Corrects long-term difference in desired Z velocity vs actual Z velocity
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: VELZ_IMAX
    // @DisplayName: Z axis velocity controller I gain maximum
    // @Description: Z axis velocity controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: VELZ_D
    // @DisplayName: Z axis velocity controller D gain
    // @Description: Z axis velocity controller D gain.  Compensates for short-term change in desired Z velocity vs actual Z velocity
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: VELZ_FF
    // @DisplayName: Z axis velocity controller feed forward
    // @Description: Z axis velocity controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: VELZ_FLTT
    // @DisplayName: Z axis velocity controller target frequency in Hz
    // @Description: Z axis velocity controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: VELZ_FLTE
    // @DisplayName: Z axis velocity controller error frequency in Hz
    // @Description: Z axis velocity controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: VELZ_FLTD
    // @DisplayName: Z axis velocity controller derivative frequency in Hz
    // @Description: Z axis velocity controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: VELZ_SMAX
    // @DisplayName: Z axis velocity slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: VELZ_PDMX
    // @DisplayName: Z axis velocity controller PD sum maximum
    // @Description: Z axis velocity controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VELZ_D_FF
    // @DisplayName: Z axis velocity derivative feedforward gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: VELZ_NTF
    // @DisplayName: Z axis velocity target notch filter index
    // @Description: Z axis velocity target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: VELZ_NEF
    // @DisplayName: Z axis velocity error notch filter index
    // @Description: Z axis velocity error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_SUBGROUPINFO(pid_vel_z, "VELZ_", 2, Loiter, AC_PID),

    // @Param: VELYAW_P
    // @DisplayName: Yaw axis velocity controller P gain
    // @Description: Yaw axis velocity controller P gain.  Corrects in proportion to the difference between the desired Yaw velocity vs actual Yaw velocity
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: VELYAW_I
    // @DisplayName: Yaw axis velocity controller I gain
    // @Description: Yaw axis velocity controller I gain.  Corrects long-term difference in desired Yaw velocity vs actual Yaw velocity
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: VELYAW_IMAX
    // @DisplayName: Yaw axis velocity controller I gain maximum
    // @Description: Yaw axis velocity controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: VELYAW_D
    // @DisplayName: Yaw axis velocity controller D gain
    // @Description: Yaw axis velocity controller D gain.  Compensates for short-term change in desired Yaw velocity vs actual Yaw velocity
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: VELYAW_FF
    // @DisplayName: Yaw axis velocity controller feed forward
    // @Description: Yaw axis velocity controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: VELYAW_FLTT
    // @DisplayName: Yaw axis velocity controller target frequency in Hz
    // @Description: Yaw axis velocity controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: VELYAW_FLTE
    // @DisplayName: Yaw axis velocity controller error frequency in Hz
    // @Description: Yaw axis velocity controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: VELYAW_FLTD
    // @DisplayName: Yaw axis velocity controller derivative frequency in Hz
    // @Description: Yaw axis velocity controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: VELYAW_SMAX
    // @DisplayName: Yaw axis velocity slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: VELYAW_PDMX
    // @DisplayName: Yaw axis velocity controller PD sum maximum
    // @Description: Yaw axis velocity controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VELYAW_D_FF
    // @DisplayName: Yaw axis velocity derivative feedforward gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: VELYAW_NTF
    // @DisplayName: Yaw axis velocity target notch filter index
    // @Description: Yaw axis velocity target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: VELYAW_NEF
    // @DisplayName: Yaw axis velocity error notch filter index
    // @Description: Yaw axis velocity error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_SUBGROUPINFO(pid_vel_yaw, "VELYAW_", 3, Loiter, AC_PID),

    // @Param: POSX_P
    // @DisplayName: X axis position controller P gain
    // @Description: X axis position controller P gain.  Corrects in proportion to the difference between the desired X position vs actual X position
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: POSX_I
    // @DisplayName: X axis position controller I gain
    // @Description: X axis position controller I gain.  Corrects long-term difference in desired X position vs actual X position
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: POSX_IMAX
    // @DisplayName: X axis position controller I gain maximum
    // @Description: X axis position controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: POSX_D
    // @DisplayName: X axis position controller D gain
    // @Description: X axis position controller D gain.  Compensates for short-term change in desired X position vs actual X position
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: POSX_FF
    // @DisplayName: X axis position controller feed forward
    // @Description: X axis position controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: POSX_FLTT
    // @DisplayName: X axis position controller target frequency in Hz
    // @Description: X axis position controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSX_FLTE
    // @DisplayName: X axis position controller error frequency in Hz
    // @Description: X axis position controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSX_FLTD
    // @DisplayName: X axis position controller derivative frequency in Hz
    // @Description: X axis position controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSX_SMAX
    // @DisplayName: X axis position slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: POSX_PDMX
    // @DisplayName: X axis position controller PD sum maximum
    // @Description: X axis position controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POSX_D_FF
    // @DisplayName: X axis position derivative feedforward gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: POSX_NTF
    // @DisplayName: X axis position target notch filter index
    // @Description: X axis position target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: POSX_NEF
    // @DisplayName: X axis position error notch filter index
    // @Description: X axis position error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_SUBGROUPINFO(pid_pos_x, "POSX_", 4, Loiter, AC_PID),

    // @Param: POSY_P
    // @DisplayName: Y axis position controller P gain
    // @Description: Y axis position controller P gain.  Corrects in proportion to the difference between the desired Y position vs actual Y position
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: POSY_I
    // @DisplayName: Y axis position controller I gain
    // @Description: Y axis position controller I gain.  Corrects long-term difference in desired Y position vs actual Y position
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: POSY_IMAX
    // @DisplayName: Y axis position controller I gain maximum
    // @Description: Y axis position controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: POSY_D
    // @DisplayName: Y axis position controller D gain
    // @Description: Y axis position controller D gain.  Compensates for short-term change in desired Y position vs actual Y position
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: POSY_FF
    // @DisplayName: Y axis position controller feed forward
    // @Description: Y axis position controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: POSY_FLTT
    // @DisplayName: Y axis position controller target frequency in Hz
    // @Description: Y axis position controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSY_FLTE
    // @DisplayName: Y axis position controller error frequency in Hz
    // @Description: Y axis position controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSY_FLTD
    // @DisplayName: Y axis position controller derivative frequency in Hz
    // @Description: Y axis position controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSY_SMAX
    // @DisplayName: Y axis position slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: POSY_PDMX
    // @DisplayName: Y axis position controller PD sum maximum
    // @Description: Y axis position controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POSY_D_FF
    // @DisplayName: Y axis position derivative feedforward gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: POSY_NTF
    // @DisplayName: Y axis position target notch filter index
    // @Description: Y axis position target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: POSY_NEF
    // @DisplayName: Y axis position error notch filter index
    // @Description: Y axis position error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_SUBGROUPINFO(pid_pos_y, "POSY_", 5, Loiter, AC_PID),

    // @Param: POSZ_P
    // @DisplayName: Z axis position controller P gain
    // @Description: Z axis position controller P gain.  Corrects in proportion to the difference between the desired Z position vs actual Z position
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: POSZ_I
    // @DisplayName: Z axis position controller I gain
    // @Description: Z axis position controller I gain.  Corrects long-term difference in desired Z position vs actual Z position
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: POSZ_IMAX
    // @DisplayName: Z axis position controller I gain maximum
    // @Description: Z axis position controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: POSZ_D
    // @DisplayName: Z axis position controller D gain
    // @Description: Z axis position controller D gain.  Compensates for short-term change in desired Z position vs actual Z position
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: POSZ_FF
    // @DisplayName: Z axis position controller feed forward
    // @Description: Z axis position controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: POSZ_FLTT
    // @DisplayName: Z axis position controller target frequency in Hz
    // @Description: Z axis position controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSZ_FLTE
    // @DisplayName: Z axis position controller error frequency in Hz
    // @Description: Z axis position controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSZ_FLTD
    // @DisplayName: Z axis position controller derivative frequency in Hz
    // @Description: Z axis position controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSZ_SMAX
    // @DisplayName: Z axis position slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: POSZ_PDMX
    // @DisplayName: Z axis position controller PD sum maximum
    // @Description: Z axis position controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POSZ_D_FF
    // @DisplayName: Z axis position derivative feedforward gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: POSZ_NTF
    // @DisplayName: Z axis position target notch filter index
    // @Description: Z axis position target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: POSZ_NEF
    // @DisplayName: Z axis position error notch filter index
    // @Description: Z axis position error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_SUBGROUPINFO(pid_pos_z, "POSZ_", 6, Loiter, AC_PID),

    // @Param: POSYAW_P
    // @DisplayName: Yaw axis position controller P gain
    // @Description: Yaw axis position controller P gain.  Corrects in proportion to the difference between the desired Yaw position vs actual Yaw position
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: POSYAW_I
    // @DisplayName: Yaw axis position controller I gain
    // @Description: Yaw axis position controller I gain.  Corrects long-term difference in desired Yaw position vs actual Yaw position
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: POSYAW_IMAX
    // @DisplayName: Yaw axis position controller I gain maximum
    // @Description: Yaw axis position controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: POSYAW_D
    // @DisplayName: Yaw axis position controller D gain
    // @Description: Yaw axis position controller D gain.  Compensates for short-term change in desired Yaw position vs actual Yaw position
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: POSYAW_FF
    // @DisplayName: Yaw axis position controller feed forward
    // @Description: Yaw axis position controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: POSYAW_FLTT
    // @DisplayName: Yaw axis position controller target frequency in Hz
    // @Description: Yaw axis position controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSYAW_FLTE
    // @DisplayName: Yaw axis position controller error frequency in Hz
    // @Description: Yaw axis position controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSYAW_FLTD
    // @DisplayName: Yaw axis position controller derivative frequency in Hz
    // @Description: Yaw axis position controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSYAW_SMAX
    // @DisplayName: Yaw axis position slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: POSYAW_PDMX
    // @DisplayName: Yaw axis position controller PD sum maximum
    // @Description: Yaw axis position controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POSYAW_D_FF
    // @DisplayName: Yaw axis position derivative feedforward gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: POSYAW_NTF
    // @DisplayName: Yaw axis position target notch filter index
    // @Description: Yaw axis position target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: POSYAW_NEF
    // @DisplayName: Yaw axis position error notch filter index
    // @Description: Yaw axis position error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_SUBGROUPINFO(pid_pos_yaw, "POSYAW_", 7, Loiter, AC_PID),

    // @Param: MAX_VELX
    // @DisplayName: Max X Velocity
    // @Description: Sets the maximum X velocity, in m/s
    // @Range: 0.2 5
    // @User: Standard
    AP_GROUPINFO("MAX_VELX", 8, Loiter, max_vel_x, 0.5),

    // @Param: MAX_VELY
    // @DisplayName: Max Y Velocity
    // @Description: Sets the maximum Y velocity, in m/s
    // @Range: 0.2 5
    // @User: Standard
    AP_GROUPINFO("MAX_VELY", 9, Loiter, max_vel_y, 0.5),

    // @Param: MAX_VELZ
    // @DisplayName: Max Z Velocity
    // @Description: Sets the maximum Z velocity, in m/s
    // @Range: 0.2 5
    // @User: Standard
    AP_GROUPINFO("MAX_VELZ", 10, Loiter, max_vel_z, 0.4),

    // @Param: MAX_VELYAW
    // @DisplayName: Max yaw Velocity
    // @Description: Sets the maximum yaw velocity, in rad/s
    // @Range: 0.2 5
    // @User: Standard
    AP_GROUPINFO("MAX_VELYAW", 11, Loiter, max_vel_yaw, 0.5),

    // @Param: MAX_POSX
    // @DisplayName: Max X Position change
    // @Description: Sets the maximum X position change, in m/s
    // @Range: 0.1 5
    // @User: Standard
    AP_GROUPINFO("MAX_POSX", 12, Loiter, max_pos_x, 0.2),

    // @Param: MAX_POSY
    // @DisplayName: Max Y Position change
    // @Description: Sets the maximum Y position change, in m/s
    // @Range: 0.1 5
    // @User: Standard
    AP_GROUPINFO("MAX_POSY", 13, Loiter, max_pos_y, 0.2),

    // @Param: MAX_POSZ
    // @DisplayName: Max Z Position change
    // @Description: Sets the maximum Z position change, in m/s
    // @Range: 0.1 5
    // @User: Standard
    AP_GROUPINFO("MAX_POSZ", 14, Loiter, max_pos_z, 0.15),

    // @Param: MAX_POSYAW
    // @DisplayName: Max Yaw Position change
    // @Description: Sets the maximum Yaw position change, in rad/s
    // @Range: 0.1 5
    // @User: Standard
    AP_GROUPINFO("MAX_POSYAW", 15, Loiter, max_pos_yaw, 0.3),

    // @Param: DIS_MASK
    // @DisplayName: Disable output mask
    // @Description: Mask for disabling (setting to zero) one or more of the 4 output axis in Velocity or Loiter modes
    // @Bitmask: 0:Right,1:Front,2:Down,3:Yaw
    // @User: Standard
    AP_GROUPINFO("DIS_MASK", 16, Loiter, dis_mask, 0),
    //R1,F2,D4,Y8. Disable Right & Front = 3, Disable Yaw and Down = 12
    //Only Front & Yaw = 1+4 = 5
    //Only Front 1+4+8 = 13
    //Only Down 1+2+8 = 11

    // @Param: PID_DZ
    // @DisplayName: Deadzone for the position PIDs
    // @Description: Output 0 thrust signal when blimp is within this distance (in meters) of the target position. Warning: If this param is greater than LOIT_MAX_POS_X multiplied by LOIT_LAG then the blimp won't move at all in the X axis in Loiter mode. Same for the other axes.
    // @Units: m
    // @Range: 0.1 1
    // @User: Standard
    AP_GROUPINFO("PID_DZ", 17, Loiter, pid_dz, 0),

    // @Param: SCALER_SPD
    // @DisplayName: Loiter scaler speed
    // @Description: Factor for scaler filter,  speed at which the scaler updates. Zero means immediate change (no filter). Higher number means slower change.
    // @Range: 0 0.999
    // @User: Advanced
    AP_GROUPINFO("SCALER_SPD", 18, Loiter, scaler_spd, 0.99),

    // @Param: POS_LAG
    // @DisplayName: Loiter position lag
    // @Description: Number of seconds' worth of travel that the actual position can be behind the target position.
    // @Units: s
    // @Range: 0 0.999
    // @User: Standard
    AP_GROUPINFO("POS_LAG", 19, Loiter, pos_lag, 1),

    // @Param: LVLPIT_P
    // @DisplayName: Pitch axis level controller P gain
    // @Description: Pitch axis level controller P gain.  Corrects in proportion to the difference between the desired Yaw position vs actual Yaw position
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: LVLPIT_I
    // @DisplayName: Pitch axis level controller I gain
    // @Description: Pitch axis level controller I gain.  Corrects long-term difference in desired Yaw position vs actual Yaw position
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: LVLPIT_IMAX
    // @DisplayName: Pitch axis level controller I gain maximum
    // @Description: Pitch axis level controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: LVLPIT_D
    // @DisplayName: Pitch axis level controller D gain
    // @Description: Pitch axis level controller D gain.  Compensates for short-term change in desired Yaw position vs actual Yaw position
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: LVLPIT_FF
    // @DisplayName: Pitch axis level controller feed forward
    // @Description: Pitch axis level controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: LVLPIT_FLTT
    // @DisplayName: Pitch axis level controller target frequency in Hz
    // @Description: Pitch axis level controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: LVLPIT_FLTE
    // @DisplayName: Pitch axis level controller error frequency in Hz
    // @Description: Pitch axis level controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: LVLPIT_FLTD
    // @DisplayName: Pitch axis level controller derivative frequency in Hz
    // @Description: Pitch axis level controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: LVLPIT_SMAX
    // @DisplayName: Pitch axis level slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: LVLPIT_PDMX
    // @DisplayName: Pitch axis level controller PD sum maximum
    // @Description: Pitch axis level controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: LVLPIT_D_FF
    // @DisplayName: Pitch axis level derivative feedforward gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: LVLPIT_NTF
    // @DisplayName: Pitch axis level target notch filter index
    // @Description: Pitch axis level target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: LVLPIT_NEF
    // @DisplayName: Pitch axis level error notch filter index
    // @Description: Pitch axis level error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_SUBGROUPINFO(pid_lvl_pitch, "LVLPIT_", 20, Loiter, AC_PID),

    // @Param: LVLRLL_P
    // @DisplayName: Roll axis level controller P gain
    // @Description: Roll axis level controller P gain.  Corrects in proportion to the difference between the desired Yaw position vs actual Yaw position
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: LVLRLL_I
    // @DisplayName: Roll axis level controller I gain
    // @Description: Roll axis level controller I gain.  Corrects long-term difference in desired Yaw position vs actual Yaw position
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: LVLRLL_IMAX
    // @DisplayName: Roll axis level controller I gain maximum
    // @Description: Roll axis level controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: LVLRLL_D
    // @DisplayName: Roll axis level controller D gain
    // @Description: Roll axis level controller D gain.  Compensates for short-term change in desired Yaw position vs actual Yaw position
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: LVLRLL_FF
    // @DisplayName: Roll axis level controller feed forward
    // @Description: Roll axis level controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: LVLRLL_FLTT
    // @DisplayName: Roll axis level controller target frequency in Hz
    // @Description: Roll axis level controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: LVLRLL_FLTE
    // @DisplayName: Roll axis level controller error frequency in Hz
    // @Description: Roll axis level controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: LVLRLL_FLTD
    // @DisplayName: Roll axis level controller derivative frequency in Hz
    // @Description: Roll axis level controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: LVLRLL_SMAX
    // @DisplayName: Roll axis level slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: LVLRLL_PDMX  
    // @DisplayName: Roll axis level controller PD sum maximum
    // @Description: Roll axis level controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: LVLRLL_D_FF
    // @DisplayName: Roll axis level derivative feedforward gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: LVLRLL_NTF
    // @DisplayName: Roll axis level target notch filter index
    // @Description: Roll axis level target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: LVLRLL_NEF
    // @DisplayName: Roll axis level error notch filter index
    // @Description: Roll axis level error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_SUBGROUPINFO(pid_lvl_roll, "LVLRLL_", 21, Loiter, AC_PID),

    // @Param: LVLMAX
    // @DisplayName: Max Level throttle
    // @Description: Max throttle output to level, 0 to disable leveling. Use only for prop blimp.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("LVLMAX", 22, Loiter, lvl_max, 0),

    // @Param: OPTIONS
    // @DisplayName: Options
    // @Description: Mask for enabling different Level mode options
    // @Bitmask: 0:Yaw rate,1:Yaw pos,2:Z Rate
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 25, Loiter, options, 0),

    // @Param: LVLRELTC
    // @DisplayName: Level Relax Timeconstant
    // @Description: Timeconstant for the speed at which the integrator for the two level controllers are relaxed. Set to 0 to disable
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("LVLRELTC", 27, Loiter, lvl_relax_tc, 0.16),

    AP_GROUPEND
};

void Loiter::run(Vector3f& target_pos, float& target_yaw, Vector4b axes_disabled)
{
    targ_dist = sqrtf(blimp.pos_ned.distance_squared(target_pos));

    const float dt = blimp.scheduler.get_last_loop_time_s();

    float yaw_ef = blimp.ahrs.get_yaw();
    Vector3f err_xyz = target_pos - blimp.pos_ned;
    float err_yaw = wrap_PI(target_yaw - yaw_ef);

    Vector4b zero;
    if ((fabsf(err_xyz.x) < pid_dz) || !blimp.motors->_armed || (dis_mask & (1<<(2-1)))) {
        zero.x = true;
    }
    if ((fabsf(err_xyz.y) < pid_dz) || !blimp.motors->_armed || (dis_mask & (1<<(1-1)))) {
        zero.y = true;
    }
    if ((fabsf(err_xyz.z) < pid_dz) || !blimp.motors->_armed || (dis_mask & (1<<(3-1)))) {
        zero.z = true;
    }
    if ((fabsf(err_yaw)   < pid_dz) || !blimp.motors->_armed || (dis_mask & (1<<(4-1)))) {
        zero.yaw = true;
    }

    //Disabled means "don't update PIDs or output anything at all". Zero means actually output zero thrust. I term is limited in either case."
    Vector4b limit = zero || axes_disabled;

    Vector3f target_vel_ef;
    if (!axes_disabled.x)
        target_vel_ef.x = pid_pos_x.update_all(target_pos.x, blimp.pos_ned.x, dt, limit.x);
    if (!axes_disabled.x)
        target_vel_ef.y = pid_pos_y.update_all(target_pos.y, blimp.pos_ned.y, dt, limit.y);
    if (!axes_disabled.z) {
        target_vel_ef.z = pid_pos_z.update_all(target_pos.z, blimp.pos_ned.z, dt, limit.z);
    }

    float target_vel_yaw = 0;
    if (!axes_disabled.yaw) {
        target_vel_yaw = pid_pos_yaw.update_error(wrap_PI(target_yaw - yaw_ef), dt, limit.yaw);
        pid_pos_yaw.set_target_rate(target_yaw);
        pid_pos_yaw.set_actual_rate(yaw_ef);
    }

    Vector3f target_vel_ef_c{constrain_float(target_vel_ef.x, -max_vel_x, max_vel_x),
                             constrain_float(target_vel_ef.y, -max_vel_y, max_vel_y),
                             constrain_float(target_vel_ef.z, -max_vel_z, max_vel_z)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -max_vel_yaw, max_vel_yaw);

    if (!blimp.motors->armed()) {
        pid_pos_x.set_integrator(0);
        pid_pos_y.set_integrator(0);
        pid_pos_z.set_integrator(0);
        pid_pos_yaw.set_integrator(0);
    }

#if HAL_LOGGING_ENABLED
    AC_PosControl::Write_PSCN(0.0, target_pos.x * 100.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_ef_c.x * 100.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCE(0.0, target_pos.y * 100.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_ef_c.y * 100.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCD(0.0, -target_pos.z * 100.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_ef_c.z * 100.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
#endif

    run_vel(target_vel_ef_c, target_vel_yaw_c, axes_disabled, false);
}

void Loiter::run_vel(Vector3f& target_vel_ef, float& target_vel_yaw, Vector4b axes_disabled, bool log)
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

    blimp.rotate_NE_to_BF(target_vel_ef.xy());
    //Just for the sake of clarity...
    Vector3f target_vel_bf = target_vel_ef;

    //New value for scaler
    float scaler_x_n = 1;
    float scaler_y_n = 1;
    float scaler_z_n = 1;
    float scaler_yaw_n = 1;

    if ((Fins::motor_frame_class)blimp.g2.frame_class.get() == Fins::MOTOR_FRAME_FISHBLIMP) {
        float xz_out = fabsf(blimp.motors->front_out) + fabsf(blimp.motors->down_out);
        if (xz_out > 1) {
            scaler_x_n = 1 / xz_out;
            scaler_z_n = 1 / xz_out;
        }
        float yyaw_out = fabsf(blimp.motors->right_out) + fabsf(blimp.motors->yaw_out);
        if (yyaw_out > 1) {
            scaler_y_n = 1 / yyaw_out;
            scaler_yaw_n = 1 / yyaw_out;
        }
    }
    else if ((Fins::motor_frame_class)blimp.g2.frame_class.get() == Fins::MOTOR_FRAME_FOUR_MOTOR) {
        float xyaw_out = fabsf(blimp.motors->front_out) + fabsf(blimp.motors->yaw_out);
        if (xyaw_out > 1) {
            scaler_x_n = 1 / xyaw_out;
            scaler_yaw_n = 1 / xyaw_out;
        }
    }

    scaler_x = scaler_x*scaler_spd + scaler_x_n*(1-scaler_spd);
    scaler_y = scaler_y*scaler_spd + scaler_y_n*(1-scaler_spd);
    scaler_z = scaler_z*scaler_spd + scaler_z_n*(1-scaler_spd);
    scaler_yaw = scaler_yaw*scaler_spd + scaler_yaw_n*(1-scaler_spd);

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("BSC", "TimeUS,x,y,z,yaw,xn,yn,zn,yawn",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                scaler_x, scaler_y, scaler_z, scaler_yaw, scaler_x_n, scaler_y_n, scaler_z_n, scaler_yaw_n);
#endif
    if (!is_equal(float(blimp.g.stream_rate), 0.0f) && AP_HAL::millis() % int((1 / blimp.g.stream_rate) * 1000) < 30){
        gcs().send_named_float("BSCXN", scaler_x_n);
        gcs().send_named_float("BSCYN", scaler_y_n);
        gcs().send_named_float("BSCZN", scaler_z_n);
        gcs().send_named_float("BSCYAWN", scaler_yaw_n);
        gcs().send_named_float("BSCX", scaler_x);
        gcs().send_named_float("BSCY", scaler_y);
        gcs().send_named_float("BSCZ", scaler_z);
        gcs().send_named_float("BSCYAW", scaler_yaw);
    }

    Vector4b zero;
    if (!blimp.motors->_armed || (dis_mask & (1<<(2-1)))) {
        zero.x = true;
    }
    if (!blimp.motors->_armed || (dis_mask & (1<<(1-1)))) {
        zero.y = true;
    }
    if (!blimp.motors->_armed || (dis_mask & (1<<(3-1)))) {
        zero.z = true;
    }
    if (!blimp.motors->_armed || (dis_mask & (1<<(4-1)))) {
        zero.yaw = true;
    }
    //Disabled means "don't update PIDs or output anything at all". Zero means actually output zero thrust. I term is limited in either case."
    Vector4b limit = zero || axes_disabled;

    Vector3f target_vel_bf_c{constrain_float(target_vel_bf.x, -max_vel_x, max_vel_x),
                             constrain_float(target_vel_bf.y, -max_vel_y, max_vel_y),
                             constrain_float(target_vel_bf.z, -max_vel_z, max_vel_z)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -max_vel_yaw, max_vel_yaw);

    Vector3f vel_bf_filtd = blimp.vel_ned_filtd;
    blimp.rotate_NE_to_BF(vel_bf_filtd.xy());

    Vector2f actuator;
    if (!axes_disabled.x) {
        actuator.x = pid_vel_x.update_all(target_vel_bf_c.x * scaler_x, vel_bf_filtd.x, dt, limit.x);
    }

    if (!axes_disabled.y) {
        actuator.y = pid_vel_y.update_all(target_vel_bf_c.y * scaler_y, vel_bf_filtd.y, dt, limit.y);
    }

    float act_down = 0;
    if (!axes_disabled.z) {
        act_down = pid_vel_z.update_all(target_vel_bf_c.z * scaler_z, vel_bf_filtd.z, dt, limit.z);
    }

    float act_yaw = 0;
    if (!axes_disabled.yaw) {
        act_yaw = pid_vel_yaw.update_all(target_vel_yaw_c * scaler_yaw, blimp.vel_yaw_filtd, dt, limit.yaw);
    }

    if (!blimp.motors->armed()) {
        pid_vel_x.set_integrator(0);
        pid_vel_y.set_integrator(0);
        pid_vel_z.set_integrator(0);
        pid_vel_yaw.set_integrator(0);
    }

    //We're already in body-frame, so we can output directly.
    if (zero.x) {
        blimp.motors->front_out = 0;
    } else if (axes_disabled.x);
    else {
        run_level_pitch(actuator.x);
    }
    if (zero.y) {
        blimp.motors->right_out = 0;
    } else if (axes_disabled.y);
    else {
        run_level_roll(actuator.y);
    }
    if (zero.z) {
        blimp.motors->down_out = 0;
    } else if (axes_disabled.z);
    else {
        blimp.motors->down_out = act_down;
    }
    if (zero.yaw) {
        blimp.motors->yaw_out  = 0;
    } else if (axes_disabled.yaw);
    else {
        blimp.motors->yaw_out = act_yaw;
    }

#if HAL_LOGGING_ENABLED
if(log) {
    AC_PosControl::Write_PSCN(0.0, 0.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_bf_c.x * 100.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCE(0.0, 0.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_bf_c.y * 100.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCD(0.0, 0.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_bf_c.z * 100.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
    }
#endif
}

void Loiter::run_level_roll(float& out_right_com)
{
    if (is_zero(lvl_max)) {
        blimp.motors->right_out = out_right_com;
        return;
    }
    const float dt = blimp.scheduler.get_last_loop_time_s();
    const float roll = blimp.ahrs.get_roll();

    float lvl_out = -blimp.loiter->pid_lvl_roll.update_all(0, roll, dt, 0);
    if (!blimp.motors->armed()) {
        blimp.loiter->pid_lvl_roll.set_integrator(0);
    }
    if(!is_zero(lvl_relax_tc)) {
        pid_lvl_roll.relax_integrator(0.0, dt, lvl_relax_tc);
    }

    float out_right_lvl = constrain_float(lvl_out, -lvl_max, lvl_max);
    blimp.motors->right_out = out_right_com + out_right_lvl;

    if (!is_equal(float(blimp.g.stream_rate), 0.0f) && AP_HAL::millis() % int((1 / blimp.g.stream_rate) * 1000) < 30){
        gcs().send_named_float("LVLRol", out_right_lvl);
        gcs().send_named_float("LVLRoc", out_right_com);
        gcs().send_named_float("LVLRs", lvl_scaler_rll);
    }
#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("LVLR", "TimeUS,ol,oc,s", "Qfff",
                                            AP_HAL::micros64(),
                                            out_right_lvl,
                                            out_right_com,
                                            lvl_scaler_rll);
#endif
}

void Loiter::run_level_pitch(float& out_front_com)
{
    if (is_zero(lvl_max)) {
        blimp.motors->front_out = out_front_com;
        return;
    }
    const float dt = blimp.scheduler.get_last_loop_time_s();
    const float pitch = blimp.ahrs.get_pitch();

    float lvl_out = pid_lvl_pitch.update_all(0, pitch, dt);

    if (!blimp.motors->armed()) {
        blimp.loiter->pid_lvl_roll.set_integrator(0);
    }

    if(!is_zero(lvl_relax_tc)) {
        pid_lvl_pitch.relax_integrator(0.0, dt, lvl_relax_tc);
    }

    float out_front_lvl = constrain_float(lvl_out, -lvl_max, lvl_max);
    blimp.motors->front_out = out_front_com + out_front_lvl;

    if (!is_equal(float(blimp.g.stream_rate), 0.0f) && AP_HAL::millis() % int((1 / blimp.g.stream_rate) * 1000) < 30){
        gcs().send_named_float("LVLPol", out_front_lvl);
        gcs().send_named_float("LVLPoc", out_front_com);
        gcs().send_named_float("LVLPs", lvl_scaler_pit);

    }
#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("LVLP", "TimeUS,ol,oc,s", "Qfff",
                                            AP_HAL::micros64(),
                                            out_front_lvl,
                                            out_front_com,
                                            lvl_scaler_pit);
#endif
}

void Loiter::run_yaw_stab(float& out_yaw_com, float& target_yaw)
{
    if (!((options & Loiter::LVL_EN_YAW_RATE) || (options & Loiter::LVL_EN_YAW_POS))) {
        blimp.motors->yaw_out = out_yaw_com*blimp.g.max_man_thr;
        return;
    }
    const float dt = blimp.scheduler.get_last_loop_time_s();

    float target_vel_yaw;
    if (options & Loiter::LVL_EN_YAW_POS) {
        const float yaw = blimp.ahrs.get_yaw();
        const float pilot_yaw = out_yaw_com * max_pos_yaw * dt;
        if (fabsf(wrap_PI(target_yaw-yaw)) < max_pos_yaw*pos_lag) {
            target_yaw = wrap_PI(target_yaw + pilot_yaw);
        }
        target_vel_yaw = pid_pos_yaw.update_error(wrap_PI(target_yaw - yaw), dt);
        pid_pos_yaw.set_target_rate(target_yaw);
        pid_pos_yaw.set_actual_rate(yaw);
        target_vel_yaw = constrain_float(target_vel_yaw, -max_vel_yaw, max_vel_yaw);
    } else {
        target_vel_yaw = out_yaw_com*max_vel_yaw;
    }
    float out = pid_vel_yaw.update_all(target_vel_yaw, blimp.vel_yaw_filtd, dt);

    if (!blimp.motors->armed()) {
        pid_vel_yaw.set_integrator(0);
    }

    blimp.motors->yaw_out = out;
}

void Loiter::run_down_stab(float& out_down_com)
{
    float velD;
    bool valid = blimp.ahrs.get_vert_pos_rate_D(velD);
    if (!(options & Loiter::LVL_EN_Z_RATE) || !valid) {
        blimp.motors->down_out = out_down_com*blimp.g.max_man_thr;
        return;
    }
    const float dt = blimp.scheduler.get_last_loop_time_s();

    float out = pid_vel_z.update_all(out_down_com*max_vel_z, velD, dt);

    if (!blimp.motors->armed()) {
        pid_vel_z.set_integrator(0);
    }

    blimp.motors->down_out = out;
}
