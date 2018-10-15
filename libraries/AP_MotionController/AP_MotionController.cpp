/* 
    File: AP_MotionController.cpp
    Author: Dr. -Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Created: 07 Oct 2018
    Last Modified: 07 Oct 2018
    Contributation: Dr. -Ing. Ahmad Kamal Nasir, 
    Description: 
    This library is used to communicate with motion control board such as 
    roboclaw through serial port communication. It uses the serial_manager
    to get the first instance of the serial port containing ESC Telemetry 
    protocol (16). Some parameters are also defined which can be accessed
    through mission planner GUI interface in order to configure roboclaws
    settings.Up to four roboclaw boards can be connected to the same
    serial bus and can be managed by the library. Each roboclaw can control
    two motors, therefore, eight motors in total. 
 */
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>
#include "AP_MotionController.h"

// @Param:
// @DisplayName: SERVO_MOCR_
// @Description:
// @Range:
// @Units: DN
// @Increment: 1
// @User: Advanced
//AP_GROUPINFO("", , AP_MotionController, , 0),

extern const AP_HAL::HAL &hal; // External reference for console debugging

const AP_Param::GroupInfo AP_MotionController::var_info[] = {
    // @Param: IX
    // @DisplayName: SERVO_MOCR_IX
    // @Description: Longitudinal wheel offset from robot center
    // @Range: 0 10
    // @Units: m
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("IX", 0, AP_MotionController, Ix, AP_MOTIONCONTROLLER_IX),

    // @Param: IY
    // @DisplayName: SERVO_MOCR_IY
    // @Description: Lateral wheel offset from robot center
    // @Range:0 10
    // @Units: m
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("IY", 1, AP_MotionController, Iy, AP_MOTIONCONTROLLER_IY),

    // @Param: RW
    // @DisplayName: SERVO_MOCR_RW
    // @Description: Wheel Radius
    // @Range:
    // @Units: m
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("RW", 2, AP_MotionController, Rw, AP_MOTIONCONTROLLER_RW),

    // @Param: QPPR
    // @DisplayName: SERVO_MOCR_QPPR
    // @Description: Quadrature pulses per revolution
    // @Range: 0 100000
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("QPPR", 3, AP_MotionController, QPPR, AP_MOTIONCONTROLLER_QPPR),

    // @Param: RC1
    // @DisplayName: SERVO_MOCR_RC1
    // @Description: The address of the first roboclaw
    // @Range: 0x80 0x90
    // @Units: DN
    // @Increment: 2
    // @User: Advanced
    AP_GROUPINFO("RC1", 4, AP_MotionController, rc[0].address, 0x80),

    // @Param: RC2
    // @DisplayName: SERVO_MOCR_RC2
    // @Description: The address of the second roboclaw
    // @Range: 0x80 0x90
    // @Units: DN
    // @Increment: 2
    // @User: Advanced
    AP_GROUPINFO("RC2", 5, AP_MotionController, rc[1].address, 0x82),

    // @Param: RC3
    // @DisplayName: SERVO_MOCR_RC3
    // @Description: The address of the third roboclaw
    // @Range: 0x80 0x90
    // @Units: DN
    // @Increment: 2
    // @User: Advanced
    AP_GROUPINFO("RC3", 6, AP_MotionController, rc[2].address, 0x84),

    // @Param: RC4
    // @DisplayName: SERVO_MOCR_RC4
    // @Description: The address of the fourth roboclaw
    // @Range: 0x80 0x90
    // @Units: DN
    // @Increment: 2
    // @User: Advanced
    AP_GROUPINFO("RC4", 7, AP_MotionController, rc[3].address, 0x86),

    // =============================================================================================
    // @Param: VKP1
    // @DisplayName: SERVO_MOCR_VKP1
    // @Description: Velocity controller propotional constant for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKP1", 8, AP_MotionController, rc[0].m1.vel.kp, AP_MOTIONCONTROLLER_VEL_KP),

    // @Param: VKI1
    // @DisplayName: SERVO_MOCR_VKI1
    // @Description: Velocity controller integral constant for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKI1", 9, AP_MotionController, rc[0].m1.vel.ki, AP_MOTIONCONTROLLER_VEL_KI),

    // @Param: VKD1
    // @DisplayName: SERVO_MOCR_VKD1
    // @Description: Velocity controller derivative constant for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKD1", 10, AP_MotionController, rc[0].m1.vel.kd, AP_MOTIONCONTROLLER_VEL_KD),

    // @Param: VQP1
    // @DisplayName: SERVO_MOCR_VQP1
    // @Description: Velocity controller qudrature pulses per seconds for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VQP1", 11, AP_MotionController, rc[0].m1.vel.qpps, AP_MOTIONCONTROLLER_VEL_QPPS),

    // @Param: PKP1
    // @DisplayName: SERVO_MOCR_PKP1
    // @Description: Position controller propotinal constant for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKP1", 12, AP_MotionController, rc[0].m2.pos.kp, AP_MOTIONCONTROLLER_POS_KP),

    // @Param: PKI1
    // @DisplayName: SERVO_MOCR_PKI1
    // @Description: Position controller integral constant for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKI1", 13, AP_MotionController, rc[0].m2.pos.ki, AP_MOTIONCONTROLLER_POS_KI),

    // @Param: PKD1
    // @DisplayName: SERVO_MOCR_PKD1
    // @Description: Position controller derivative constant for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKD1", 14, AP_MotionController, rc[0].m2.pos.kd, AP_MOTIONCONTROLLER_POS_KD),

    // @Param: PIM1
    // @DisplayName: SERVO_MOCR_PIM1
    // @Description: Position controller maximum integration limit for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PIM1", 15, AP_MotionController, rc[0].m2.pos.maxi, AP_MOTIONCONTROLLER_POS_IMAX),

    // @Param: PDB1
    // @DisplayName: SERVO_MOCR_PDB1
    // @Description: Position controller deadband for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PDB1", 16, AP_MotionController, rc[0].m2.pos.deadband, AP_MOTIONCONTROLLER_POS_DEADBAND),

    // @Param: PMX1
    // @DisplayName: SERVO_MOCR_PMX1
    // @Description: Position controller maximum position limit for first roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMX1", 17, AP_MotionController, rc[0].m2.pos.maxpos, AP_MOTIONCONTROLLER_POS_MAXPOS),

    // @Param: PMN1
    // @DisplayName: SERVO_MOCR_PMN1
    // @Description: Position controller minimum position limit for first roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMN1", 18, AP_MotionController, rc[0].m2.pos.minpos, AP_MOTIONCONTROLLER_POS_MINPOS),

    // @Param: PAC1
    // @DisplayName: SERVO_MOCR_PAC1
    // @Description: Position controller acceleration for first roboclaw
    // @Range: 0 1e9
    // @Units: QPPS/S
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PAC1", 19, AP_MotionController, rc[0].m2.acceleration, AP_MOTIONCONTROLLER_POS_ACCEL),

    // @Param: PVE1
    // @DisplayName: SERVO_MOCR_PVE1
    // @Description: Position controller velocity for first roboclaw
    // @Range: 0 1e9
    // @Units: QPPS
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PVE1", 20, AP_MotionController, rc[0].m2.velocity, AP_MOTIONCONTROLLER_POS_VEL),

    // =============================================================================================
    // @Param: VKP2
    // @DisplayName: SERVO_MOCR_VKP2
    // @Description: Velocity controller propotional constant for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKP2", 21, AP_MotionController, rc[1].m1.vel.kp, AP_MOTIONCONTROLLER_VEL_KP),

    // @Param: VKI2
    // @DisplayName: SERVO_MOCR_VKI2
    // @Description: Velocity controller integral constant for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKI2", 22, AP_MotionController, rc[1].m1.vel.ki, AP_MOTIONCONTROLLER_VEL_KI),

    // @Param: VKD2
    // @DisplayName: SERVO_MOCR_VKD2
    // @Description: Velocity controller derivative constant for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKD2", 23, AP_MotionController, rc[1].m1.vel.kd, AP_MOTIONCONTROLLER_VEL_KD),

    // @Param: VQP2
    // @DisplayName: SERVO_MOCR_VQP2
    // @Description: Velocity controller qudrature pulses per seconds for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VQP2", 24, AP_MotionController, rc[1].m1.vel.qpps, AP_MOTIONCONTROLLER_VEL_QPPS),

    // @Param: PKP2
    // @DisplayName: SERVO_MOCR_PKP2
    // @Description: Position controller propotinal constant for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKP2", 25, AP_MotionController, rc[1].m2.pos.kp, AP_MOTIONCONTROLLER_POS_KP),

    // @Param: PKI2
    // @DisplayName: SERVO_MOCR_PKI2
    // @Description: Position controller integral constant for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKI2", 26, AP_MotionController, rc[1].m2.pos.ki, AP_MOTIONCONTROLLER_POS_KI),

    // @Param: PKD2
    // @DisplayName: SERVO_MOCR_PKD2
    // @Description: Position controller derivative constant for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKD2", 27, AP_MotionController, rc[1].m2.pos.kd, AP_MOTIONCONTROLLER_POS_KD),

    // @Param: PIM2
    // @DisplayName: SERVO_MOCR_PIM2
    // @Description: Position controller maximum integration limit for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PIM2", 28, AP_MotionController, rc[1].m2.pos.maxi, AP_MOTIONCONTROLLER_POS_IMAX),

    // @Param: PDB2
    // @DisplayName: SERVO_MOCR_PDB2
    // @Description: Position controller deadband for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PDB2", 29, AP_MotionController, rc[1].m2.pos.deadband, AP_MOTIONCONTROLLER_POS_DEADBAND),

    // @Param: PMX2
    // @DisplayName: SERVO_MOCR_PMX2
    // @Description: Position controller maximum position limit for second roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMX2", 30, AP_MotionController, rc[1].m2.pos.maxpos, AP_MOTIONCONTROLLER_POS_MAXPOS),

    // @Param: PMN2
    // @DisplayName: SERVO_MOCR_PMN2
    // @Description: Position controller minimum position limit for second roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMN2", 31, AP_MotionController, rc[1].m2.pos.minpos, AP_MOTIONCONTROLLER_POS_MINPOS),

    // @Param: PAC2
    // @DisplayName: SERVO_MOCR_PAC2
    // @Description: Position controller acceleration for second roboclaw
    // @Range: 0 1e9
    // @Units: QPPS/S
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PAC2", 32, AP_MotionController, rc[1].m2.acceleration, AP_MOTIONCONTROLLER_POS_ACCEL),

    // @Param: PVE2
    // @DisplayName: SERVO_MOCR_PVE2
    // @Description: Position controller velocity for second roboclaw
    // @Range: 0 1e9
    // @Units: QPPS
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PVE2", 33, AP_MotionController, rc[1].m2.velocity, AP_MOTIONCONTROLLER_POS_VEL),

    // =============================================================================================
    // @Param: VKP3
    // @DisplayName: SERVO_MOCR_VKP3
    // @Description: Velocity controller propotional constant for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKP3", 34, AP_MotionController, rc[2].m1.vel.kp, AP_MOTIONCONTROLLER_VEL_KP),

    // @Param: VKI3
    // @DisplayName: SERVO_MOCR_VKI3
    // @Description: Velocity controller integral constant for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKI3", 35, AP_MotionController, rc[2].m1.vel.ki, AP_MOTIONCONTROLLER_VEL_KI),

    // @Param: VKD3
    // @DisplayName: SERVO_MOCR_VKD3
    // @Description: Velocity controller derivative constant for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKD3", 36, AP_MotionController, rc[2].m1.vel.kd, AP_MOTIONCONTROLLER_VEL_KD),

    // @Param: VQP3
    // @DisplayName: SERVO_MOCR_VQP3
    // @Description: Velocity controller qudrature pulses per seconds for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VQP3", 37, AP_MotionController, rc[2].m1.vel.qpps, AP_MOTIONCONTROLLER_VEL_QPPS),

    // @Param: PKP3
    // @DisplayName: SERVO_MOCR_PKP3
    // @Description: Position controller propotinal constant for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKP3", 38, AP_MotionController, rc[2].m2.pos.kp, AP_MOTIONCONTROLLER_POS_KP),

    // @Param: PKI3
    // @DisplayName: SERVO_MOCR_PKI3
    // @Description: Position controller integral constant for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKI3", 39, AP_MotionController, rc[2].m2.pos.ki, AP_MOTIONCONTROLLER_POS_KI),

    // @Param: PKD3
    // @DisplayName: SERVO_MOCR_PKD3
    // @Description: Position controller derivative constant for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKD3", 40, AP_MotionController, rc[2].m2.pos.kd, AP_MOTIONCONTROLLER_POS_KD),

    // @Param: PIM3
    // @DisplayName: SERVO_MOCR_PIM3
    // @Description: Position controller maximum integration limit for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PIM3", 41, AP_MotionController, rc[2].m2.pos.maxi, AP_MOTIONCONTROLLER_POS_IMAX),

    // @Param: PDB3
    // @DisplayName: SERVO_MOCR_PDB3
    // @Description: Position controller deadband for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PDB3", 42, AP_MotionController, rc[2].m2.pos.deadband, AP_MOTIONCONTROLLER_POS_DEADBAND),

    // @Param: PMX3
    // @DisplayName: SERVO_MOCR_PMX3
    // @Description: Position controller maximum position limit for third roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMX3", 43, AP_MotionController, rc[2].m2.pos.maxpos, AP_MOTIONCONTROLLER_POS_MAXPOS),

    // @Param: PMN3
    // @DisplayName: SERVO_MOCR_PMN3
    // @Description: Position controller minimum position limit for third roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMN3", 44, AP_MotionController, rc[2].m2.pos.minpos, AP_MOTIONCONTROLLER_POS_MINPOS),

    // @Param: PAC3
    // @DisplayName: SERVO_MOCR_PAC3
    // @Description: Position controller acceleration for third roboclaw
    // @Range: 0 1e9
    // @Units: QPPS/S
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PAC3", 45, AP_MotionController, rc[2].m2.acceleration, AP_MOTIONCONTROLLER_POS_ACCEL),

    // @Param: PVE3
    // @DisplayName: SERVO_MOCR_PVE3
    // @Description: Position controller velocity for third roboclaw
    // @Range: 0 1e9
    // @Units: QPPS
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PVE3", 46, AP_MotionController, rc[2].m2.velocity, AP_MOTIONCONTROLLER_POS_VEL),

    // =============================================================================================
    // @Param: VKP4
    // @DisplayName: SERVO_MOCR_VKP4
    // @Description: Velocity controller propotional constant for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKP4", 47, AP_MotionController, rc[3].m1.vel.kp, AP_MOTIONCONTROLLER_VEL_KP),

    // @Param: VKI4
    // @DisplayName: SERVO_MOCR_VKI4
    // @Description: Velocity controller integral constant for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKI4", 48, AP_MotionController, rc[3].m1.vel.ki, AP_MOTIONCONTROLLER_VEL_KI),

    // @Param: VKD4
    // @DisplayName: SERVO_MOCR_VKD4
    // @Description: Velocity controller derivative constant for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKD4", 49, AP_MotionController, rc[3].m1.vel.kd, AP_MOTIONCONTROLLER_VEL_KD),

    // @Param: VQP4
    // @DisplayName: SERVO_MOCR_VQP4
    // @Description: Velocity controller qudrature pulses per seconds for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VQP4", 50, AP_MotionController, rc[3].m1.vel.qpps, AP_MOTIONCONTROLLER_VEL_QPPS),

    // @Param: PKP4
    // @DisplayName: SERVO_MOCR_PKP4
    // @Description: Position controller propotinal constant for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKP4", 51, AP_MotionController, rc[3].m2.pos.kp, AP_MOTIONCONTROLLER_POS_KP),

    // @Param: PKI4
    // @DisplayName: SERVO_MOCR_PKI4
    // @Description: Position controller integral constant for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKI4", 52, AP_MotionController, rc[3].m2.pos.ki, AP_MOTIONCONTROLLER_POS_KI),

    // @Param: PKD4
    // @DisplayName: SERVO_MOCR_PKD4
    // @Description: Position controller derivative constant for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKD4", 53, AP_MotionController, rc[3].m2.pos.kd, AP_MOTIONCONTROLLER_POS_KD),

    // @Param: PIM4
    // @DisplayName: SERVO_MOCR_PIM4
    // @Description: Position controller maximum integration limit for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PIM4", 54, AP_MotionController, rc[3].m2.pos.maxi, AP_MOTIONCONTROLLER_POS_IMAX),

    // @Param: PDB4
    // @DisplayName: SERVO_MOCR_PDB4
    // @Description: Position controller deadband for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PDB4", 55, AP_MotionController, rc[3].m2.pos.deadband, AP_MOTIONCONTROLLER_POS_DEADBAND),

    // @Param: PMX4
    // @DisplayName: SERVO_MOCR_PMX4
    // @Description: Position controller maximum position limit for fourth roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMX4", 56, AP_MotionController, rc[3].m2.pos.maxpos, AP_MOTIONCONTROLLER_POS_MAXPOS),

    // @Param: PMN4
    // @DisplayName: SERVO_MOCR_PMN4
    // @Description: Position controller minimum position limit for fourth roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMN4", 57, AP_MotionController, rc[3].m2.pos.minpos, AP_MOTIONCONTROLLER_POS_MINPOS),

    // @Param: PAC4
    // @DisplayName: SERVO_MOCR_PAC4
    // @Description: Position controller acceleration for fourth roboclaw
    // @Range: 0 1e9
    // @Units: QPPS/S
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PAC4", 58, AP_MotionController, rc[3].m2.acceleration, AP_MOTIONCONTROLLER_POS_ACCEL),

    // @Param: PVE4
    // @DisplayName: SERVO_MOCR_PVE4
    // @Description: Position controller velocity for fourth roboclaw
    // @Range: 0 1e9
    // @Units: QPPS
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PVE4", 59, AP_MotionController, rc[3].m2.velocity, AP_MOTIONCONTROLLER_POS_VEL),

    AP_GROUPEND};

AP_MotionController::AP_MotionController()
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

AP_MotionController::~AP_MotionController()
{
}

void AP_MotionController::init(void)
{
    AP_SerialManager &serial_manager = AP::serialmanager();
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ESCTelemetry, 0);
    roboclaw.init(port, 10000);
    for (int i = 0; i < 4; i++)
    {
        roboclaw.ResetEncoders(rc[i].address);
        roboclaw.SetEncM1(rc[i].address, 0);
        roboclaw.SetEncM2(rc[i].address, 0);
        //roboclaw.SetLogicVoltages(rc[i].address, 5, 34);
        //roboclaw.SetMainVoltages(rc[i].address, 5, 34);
        //roboclaw.SetM1MaxCurrent(rc[i].address, 15);
        //roboclaw.SetM2MaxCurrent(rc[i].address, 15);
        roboclaw.SetM1VelocityPID(rc[i].address, rc[i].m1.vel.kp, rc[i].m1.vel.ki, rc[i].m1.vel.kd, rc[i].m1.vel.qpps);
        roboclaw.SetM2VelocityPID(rc[i].address, 0, 0, 0, 50); //TODO
        roboclaw.SetM2PositionPID(rc[i].address, rc[i].m2.pos.kp, rc[i].m2.pos.ki, rc[i].m2.pos.kd, rc[i].m2.pos.maxi, rc[i].m2.pos.deadband, rc[i].m2.pos.minpos, rc[i].m2.pos.maxpos);
        //roboclaw.WriteNVM(rc[i].address); // Store settings into EEPROM, TODO: read and compare if settings needs to be resaved
    }
    //hal.console->printf("AP_MotionController initialized.\n");
    //gcs().send_text(MAV_SEVERITY_INFO, "AP_MotionController Initialized.");
}

double AP_MotionController::constrain_map(double x, double in_min, double in_max, double out_min, double out_max)
{
    //constrain the input value between in_min and in_max
    if (x < in_min)
        x = in_min;
    else if (x > in_max)
        x = in_max;
    //map the value from the in-scale (in_min->in_max) into out-scale (out_min->out_max)
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void AP_MotionController::update(uint16_t steering, uint16_t throttle)
{
    static int counter = 0;
    const double QPPM = QPPR / (2 * M_PI * Rw); // Quadrature pulses per meter
    const double QPPRAD = QPPR / (2 * M_PI);    // Quadrature pulses per Radian
    // Convert the input steering and throttle value into angular and linear velocity respectively.
    double v = AP_MotionController::constrain_map(steering, SRV_Channels::srv_channel(1)->get_output_min(), SRV_Channels::srv_channel(1)->get_output_max(), -1, 1);
    double w = AP_MotionController::constrain_map(steering, SRV_Channels::srv_channel(3)->get_output_min(), SRV_Channels::srv_channel(3)->get_output_max(), -0.5236, 0.5236);
    if (fabsf(w) <= 1e-5)
    {
        //Velocity Setpoints (M1):
        rc[0].m1.vel.setpoint = QPPM * v / Rw;
        rc[1].m1.vel.setpoint = QPPM * v / Rw;
        rc[2].m1.vel.setpoint = rc[0].m1.vel.setpoint;
        rc[3].m1.vel.setpoint = rc[1].m1.vel.setpoint;
        //Position/Angle Setpoints (M2):
        rc[0].m2.pos.setpoint = 0;
        rc[1].m2.pos.setpoint = 0;
        rc[2].m2.pos.setpoint = 0;
        rc[3].m2.pos.setpoint = 0;
    }
    else
    {
        //Velocity Setpoints (M1):
        float Rins = fabsf(v / w);
        rc[0].m1.vel.setpoint = QPPM * v * (2.0 / (Rw * Rins)) * powf((powf(Rins / 2.0 - Iy, 2.0) + powf(Ix / 2.0, 2.0)), 0.5);
        rc[1].m1.vel.setpoint = QPPM * v * (2.0 / (Rw * Rins)) * powf((powf(Rins / 2.0 + Iy, 2.0) + powf(Ix / 2.0, 2.0)), 0.5);
        rc[2].m1.vel.setpoint = rc[0].m1.vel.setpoint;
        rc[3].m1.vel.setpoint = rc[1].m1.vel.setpoint;
        //Position/Angle Setpoints (M2):
        rc[0].m2.pos.setpoint = ((w > 0) ? 1 : -1) * QPPRAD * atan2f(Iy, Rins - Ix);
        rc[1].m2.pos.setpoint = ((w > 0) ? 1 : -1) * QPPRAD * atan2f(Iy, Rins + Ix);
        rc[2].m2.pos.setpoint = -rc[0].m2.pos.setpoint;
        rc[3].m2.pos.setpoint = -rc[1].m2.pos.setpoint;
    }
    for (int i = 0; i < 4; i++)
    {
        roboclaw.SpeedAccelM1(rc[i].address, rc[i].m1.acceleration, rc[i].m1.vel.setpoint);
        roboclaw.SpeedAccelDeccelPositionM2(rc[i].address, rc[i].m2.acceleration, rc[i].m2.velocity, rc[i].m2.acceleration, rc[i].m2.pos.setpoint, 1);
        //roboclaw.ForwardBackwardM1(rc[i].address, v);
        //roboclaw.ForwardBackwardM2(rc[i].address, w);
    }
    if (++counter >= 10) // Call every fifth of a second (given 50Hz calling rate)
    {
        uint32_t tempUINT32;
        uint16_t tempUINT16;
        int16_t tempINT16_1, tempINT16_2;
        for (int i = 0; i < 4; i++)
        {
            tempUINT32 = roboclaw.ReadSpeedM1(rc[i].address, &rc[i].m1.stat, &rc[i].m1.valid);
            if (rc[i].m1.valid)
                rc[i].m1.speed = tempUINT32;
            tempUINT32 = roboclaw.ReadSpeedM2(rc[i].address, &rc[i].m2.stat, &rc[i].m2.valid);
            if (rc[i].m2.valid)
                rc[i].m2.speed = tempUINT32;
            tempUINT32 = roboclaw.ReadEncM1(rc[i].address, &rc[i].m1.stat, &rc[i].m1.valid);
            if (rc[i].m1.valid)
                rc[i].m1.encoder = tempUINT32;
            tempUINT32 = roboclaw.ReadEncM2(rc[i].address, &rc[i].m2.stat, &rc[i].m2.valid);
            if (rc[i].m2.valid)
                rc[i].m2.encoder = tempUINT32;
            if (roboclaw.ReadCurrents(rc[i].address, tempINT16_1, tempINT16_2))
            {
                rc[i].m1.current = tempINT16_1;
                rc[i].m2.current = tempINT16_2;
            }
            tempUINT16 = roboclaw.ReadMainBatteryVoltage(rc[i].address, &rc[i].m1.valid);
            if (rc[i].m1.valid)
                rc[i].batteryVoltage = tempUINT16;
            tempUINT16 = roboclaw.ReadLogicBatteryVoltage(rc[i].address, &rc[i].m1.valid);
            if (rc[i].m1.valid)
                rc[i].logicVoltage = tempUINT16;
            if (roboclaw.ReadTemp(rc[i].address, tempUINT16))
                rc[i].temperature = tempUINT16;
        }
        //hal.console->printf("Time:%05.3f Steering:%d Throttle:%d\n", AP_HAL::millis64() / 1000.0, v, w);
        //gcs().send_text(MAV_SEVERITY_INFO, "Time:%05.3f Steering:%d Throttle:%d", AP_HAL::millis64()/1000.0, v, w);
        DataFlash_Class::instance()->Log_Write("MOCR", "TimeUS,AngVel,LinVel", "Qff", AP_HAL::micros64(), v, w);
        counter = 0;
    }
}
