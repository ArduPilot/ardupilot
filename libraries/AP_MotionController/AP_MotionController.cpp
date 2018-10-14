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
    AP_GROUPINFO("RC1", 4, AP_MotionController, Address[0], 0x80),

    // @Param: RC2
    // @DisplayName: SERVO_MOCR_RC2
    // @Description: The address of the second roboclaw
    // @Range: 0x80 0x90
    // @Units: DN
    // @Increment: 2
    // @User: Advanced
    AP_GROUPINFO("RC2", 5, AP_MotionController, Address[1], 0x82),

    // @Param: RC3
    // @DisplayName: SERVO_MOCR_RC3
    // @Description: The address of the third roboclaw
    // @Range: 0x80 0x90
    // @Units: DN
    // @Increment: 2
    // @User: Advanced
    AP_GROUPINFO("RC3", 6, AP_MotionController, Address[2], 0x84),

    // @Param: RC4
    // @DisplayName: SERVO_MOCR_RC4
    // @Description: The address of the fourth roboclaw
    // @Range: 0x80 0x90
    // @Units: DN
    // @Increment: 2
    // @User: Advanced
    AP_GROUPINFO("RC4", 7, AP_MotionController, Address[3], 0x86),

    // =============================================================================================
    // @Param: VKP1
    // @DisplayName: SERVO_MOCR_VKP1
    // @Description: Velocity controller propotional constant for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKP1", 8, AP_MotionController, controlSettings[0][0], AP_MOTIONCONTROLLER_VEL_KP),

    // @Param: VKI1
    // @DisplayName: SERVO_MOCR_VKI1
    // @Description: Velocity controller integral constant for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKI1", 9, AP_MotionController, controlSettings[0][1], AP_MOTIONCONTROLLER_VEL_KI),

    // @Param: VKD1
    // @DisplayName: SERVO_MOCR_VKD1
    // @Description: Velocity controller derivative constant for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKD1", 10, AP_MotionController, controlSettings[0][2], AP_MOTIONCONTROLLER_VEL_KD),

    // @Param: VQP1
    // @DisplayName: SERVO_MOCR_VQP1    
    // @Description: Velocity controller qudrature pulses per seconds for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VQP1", 11, AP_MotionController, controlSettings[0][3], AP_MOTIONCONTROLLER_VEL_QPPS),

    // @Param: PKP1
    // @DisplayName: SERVO_MOCR_PKP1
    // @Description: Position controller propotinal constant for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKP1", 12, AP_MotionController, controlSettings[0][4], AP_MOTIONCONTROLLER_POS_KP),

    // @Param: PKI1
    // @DisplayName: SERVO_MOCR_PKI1
    // @Description: Position controller integral constant for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKI1", 13, AP_MotionController, controlSettings[0][5], AP_MOTIONCONTROLLER_POS_KI),

    // @Param: PKD1
    // @DisplayName: SERVO_MOCR_PKD1
    // @Description: Position controller derivative constant for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKD1", 14, AP_MotionController, controlSettings[0][6], AP_MOTIONCONTROLLER_POS_KD),

    // @Param: PIM1
    // @DisplayName: SERVO_MOCR_PIM1
    // @Description: Position controller maximum integration limit for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PIM1", 15, AP_MotionController, controlSettings[0][7], AP_MOTIONCONTROLLER_POS_IMAX),

    // @Param: PDB1
    // @DisplayName: SERVO_MOCR_PDB1
    // @Description: Position controller deadband for first roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PDB1", 16, AP_MotionController, controlSettings[0][8], AP_MOTIONCONTROLLER_POS_DEADBAND),

    // @Param: PMX1
    // @DisplayName: SERVO_MOCR_PMX1
    // @Description: Position controller maximum position limit for first roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMX1", 17, AP_MotionController, controlSettings[0][9], AP_MOTIONCONTROLLER_POS_MAXPOS),

    // @Param: PMN1
    // @DisplayName: SERVO_MOCR_PMN1
    // @Description: Position controller minimum position limit for first roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMN1", 18, AP_MotionController, controlSettings[0][10], AP_MOTIONCONTROLLER_POS_MINPOS),

    // @Param: PAC1
    // @DisplayName: SERVO_MOCR_PAC1
    // @Description: Position controller acceleration for first roboclaw
    // @Range: 0 1e9
    // @Units: QPPS/S
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PAC1", 19, AP_MotionController, controlSettings[0][11], AP_MOTIONCONTROLLER_POS_ACCEL),

    // @Param: PVE1
    // @DisplayName: SERVO_MOCR_PVE1
    // @Description: Position controller velocity for first roboclaw
    // @Range: 0 1e9
    // @Units: QPPS
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PVE1", 20, AP_MotionController, controlSettings[0][12], AP_MOTIONCONTROLLER_POS_VEL),

    // =============================================================================================
    // @Param: VKP2
    // @DisplayName: SERVO_MOCR_VKP2
    // @Description: Velocity controller propotional constant for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKP2", 21, AP_MotionController, controlSettings[1][0], AP_MOTIONCONTROLLER_VEL_KP),

    // @Param: VKI2
    // @DisplayName: SERVO_MOCR_VKI2
    // @Description: Velocity controller integral constant for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKI2", 22, AP_MotionController, controlSettings[1][1], AP_MOTIONCONTROLLER_VEL_KI),

    // @Param: VKD2
    // @DisplayName: SERVO_MOCR_VKD2
    // @Description: Velocity controller derivative constant for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKD2", 23, AP_MotionController, controlSettings[1][2], AP_MOTIONCONTROLLER_VEL_KD),

    // @Param: VQP2
    // @DisplayName: SERVO_MOCR_VQP2
    // @Description: Velocity controller qudrature pulses per seconds for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VQP2", 24, AP_MotionController, controlSettings[1][3], AP_MOTIONCONTROLLER_VEL_QPPS),

    // @Param: PKP2
    // @DisplayName: SERVO_MOCR_PKP2
    // @Description: Position controller propotinal constant for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKP2", 25, AP_MotionController, controlSettings[1][4], AP_MOTIONCONTROLLER_POS_KP),

    // @Param: PKI2
    // @DisplayName: SERVO_MOCR_PKI2
    // @Description: Position controller integral constant for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKI2", 26, AP_MotionController, controlSettings[1][5], AP_MOTIONCONTROLLER_POS_KI),

    // @Param: PKD2
    // @DisplayName: SERVO_MOCR_PKD2
    // @Description: Position controller derivative constant for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKD2", 27, AP_MotionController, controlSettings[1][6], AP_MOTIONCONTROLLER_POS_KD),

    // @Param: PIM2
    // @DisplayName: SERVO_MOCR_PIM2
    // @Description: Position controller maximum integration limit for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PIM2", 28, AP_MotionController, controlSettings[1][7], AP_MOTIONCONTROLLER_POS_IMAX),

    // @Param: PDB2
    // @DisplayName: SERVO_MOCR_PDB2
    // @Description: Position controller deadband for second roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PDB2", 29, AP_MotionController, controlSettings[1][8], AP_MOTIONCONTROLLER_POS_DEADBAND),

    // @Param: PMX2
    // @DisplayName: SERVO_MOCR_PMX2
    // @Description: Position controller maximum position limit for second roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMX2", 30, AP_MotionController, controlSettings[1][9], AP_MOTIONCONTROLLER_POS_MAXPOS),

    // @Param: PMN2
    // @DisplayName: SERVO_MOCR_PMN2
    // @Description: Position controller minimum position limit for second roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMN2", 31, AP_MotionController, controlSettings[1][10], AP_MOTIONCONTROLLER_POS_MINPOS),

    // @Param: PAC2
    // @DisplayName: SERVO_MOCR_PAC2
    // @Description: Position controller acceleration for second roboclaw
    // @Range: 0 1e9
    // @Units: QPPS/S
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PAC2", 32, AP_MotionController, controlSettings[1][11], AP_MOTIONCONTROLLER_POS_ACCEL),

    // @Param: PVE2
    // @DisplayName: SERVO_MOCR_PVE2
    // @Description: Position controller velocity for second roboclaw
    // @Range: 0 1e9
    // @Units: QPPS
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PVE2", 33, AP_MotionController, controlSettings[1][12], AP_MOTIONCONTROLLER_POS_VEL),
	
    // =============================================================================================
    // @Param: VKP3
    // @DisplayName: SERVO_MOCR_VKP3
    // @Description: Velocity controller propotional constant for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKP3", 34, AP_MotionController, controlSettings[2][0], AP_MOTIONCONTROLLER_VEL_KP),

    // @Param: VKI3
    // @DisplayName: SERVO_MOCR_VKI3
    // @Description: Velocity controller integral constant for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKI3", 35, AP_MotionController, controlSettings[2][1], AP_MOTIONCONTROLLER_VEL_KI),

    // @Param: VKD3
    // @DisplayName: SERVO_MOCR_VKD3
    // @Description: Velocity controller derivative constant for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKD3", 36, AP_MotionController, controlSettings[2][2], AP_MOTIONCONTROLLER_VEL_KD),

    // @Param: VQP3
    // @DisplayName: SERVO_MOCR_VQP3    
    // @Description: Velocity controller qudrature pulses per seconds for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VQP3", 37, AP_MotionController, controlSettings[2][3], AP_MOTIONCONTROLLER_VEL_QPPS),

    // @Param: PKP3
    // @DisplayName: SERVO_MOCR_PKP3
    // @Description: Position controller propotinal constant for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKP3", 38, AP_MotionController, controlSettings[2][4], AP_MOTIONCONTROLLER_POS_KP),

    // @Param: PKI3
    // @DisplayName: SERVO_MOCR_PKI3
    // @Description: Position controller integral constant for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKI3", 39, AP_MotionController, controlSettings[2][5], AP_MOTIONCONTROLLER_POS_KI),

    // @Param: PKD3
    // @DisplayName: SERVO_MOCR_PKD3
    // @Description: Position controller derivative constant for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKD3", 40, AP_MotionController, controlSettings[2][6], AP_MOTIONCONTROLLER_POS_KD),

    // @Param: PIM3
    // @DisplayName: SERVO_MOCR_PIM3
    // @Description: Position controller maximum integration limit for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PIM3", 41, AP_MotionController, controlSettings[2][7], AP_MOTIONCONTROLLER_POS_IMAX),

    // @Param: PDB3
    // @DisplayName: SERVO_MOCR_PDB3
    // @Description: Position controller deadband for third roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PDB3", 42, AP_MotionController, controlSettings[2][8], AP_MOTIONCONTROLLER_POS_DEADBAND),

    // @Param: PMX3
    // @DisplayName: SERVO_MOCR_PMX3
    // @Description: Position controller maximum position limit for third roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMX3", 43, AP_MotionController, controlSettings[2][9], AP_MOTIONCONTROLLER_POS_MAXPOS),

    // @Param: PMN3
    // @DisplayName: SERVO_MOCR_PMN3
    // @Description: Position controller minimum position limit for third roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMN3", 44, AP_MotionController, controlSettings[2][10], AP_MOTIONCONTROLLER_POS_MINPOS),

    // @Param: PAC3
    // @DisplayName: SERVO_MOCR_PAC3
    // @Description: Position controller acceleration for third roboclaw
    // @Range: 0 1e9
    // @Units: QPPS/S
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PAC3", 45, AP_MotionController, controlSettings[2][11], AP_MOTIONCONTROLLER_POS_ACCEL),

    // @Param: PVE3
    // @DisplayName: SERVO_MOCR_PVE3
    // @Description: Position controller velocity for third roboclaw
    // @Range: 0 1e9
    // @Units: QPPS
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PVE3", 46, AP_MotionController, controlSettings[2][12], AP_MOTIONCONTROLLER_POS_VEL),

    // =============================================================================================
    // @Param: VKP4
    // @DisplayName: SERVO_MOCR_VKP4
    // @Description: Velocity controller propotional constant for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKP4", 47, AP_MotionController, controlSettings[3][0], AP_MOTIONCONTROLLER_VEL_KP),

    // @Param: VKI4
    // @DisplayName: SERVO_MOCR_VKI4
    // @Description: Velocity controller integral constant for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKI4", 48, AP_MotionController, controlSettings[3][1], AP_MOTIONCONTROLLER_VEL_KI),

    // @Param: VKD4
    // @DisplayName: SERVO_MOCR_VKD4
    // @Description: Velocity controller derivative constant for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VKD4", 49, AP_MotionController, controlSettings[3][2], AP_MOTIONCONTROLLER_VEL_KD),

    // @Param: VQP4
    // @DisplayName: SERVO_MOCR_VQP4
    // @Description: Velocity controller qudrature pulses per seconds for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VQP4", 50, AP_MotionController, controlSettings[3][3], AP_MOTIONCONTROLLER_VEL_QPPS),

    // @Param: PKP4
    // @DisplayName: SERVO_MOCR_PKP4
    // @Description: Position controller propotinal constant for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKP4", 51, AP_MotionController, controlSettings[3][4], AP_MOTIONCONTROLLER_POS_KP),

    // @Param: PKI4
    // @DisplayName: SERVO_MOCR_PKI4
    // @Description: Position controller integral constant for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKI4", 52, AP_MotionController, controlSettings[3][5], AP_MOTIONCONTROLLER_POS_KI),

    // @Param: PKD4
    // @DisplayName: SERVO_MOCR_PKD4
    // @Description: Position controller derivative constant for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PKD4", 53, AP_MotionController, controlSettings[3][6], AP_MOTIONCONTROLLER_POS_KD),

    // @Param: PIM4
    // @DisplayName: SERVO_MOCR_PIM4
    // @Description: Position controller maximum integration limit for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PIM4", 54, AP_MotionController, controlSettings[3][7], AP_MOTIONCONTROLLER_POS_IMAX),

    // @Param: PDB4
    // @DisplayName: SERVO_MOCR_PDB4
    // @Description: Position controller deadband for fourth roboclaw
    // @Range: 0 1e6
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PDB4", 55, AP_MotionController, controlSettings[3][8], AP_MOTIONCONTROLLER_POS_DEADBAND),

    // @Param: PMX4
    // @DisplayName: SERVO_MOCR_PMX4
    // @Description: Position controller maximum position limit for fourth roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMX4", 56, AP_MotionController, controlSettings[3][9], AP_MOTIONCONTROLLER_POS_MAXPOS),

    // @Param: PMN4
    // @DisplayName: SERVO_MOCR_PMN4
    // @Description: Position controller minimum position limit for fourth roboclaw
    // @Range: 0 1e9
    // @Units: QP
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PMN4", 57, AP_MotionController, controlSettings[3][10], AP_MOTIONCONTROLLER_POS_MINPOS),

    // @Param: PAC4
    // @DisplayName: SERVO_MOCR_PAC4
    // @Description: Position controller acceleration for fourth roboclaw
    // @Range: 0 1e9
    // @Units: QPPS/S
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PAC4", 58, AP_MotionController, controlSettings[3][11], AP_MOTIONCONTROLLER_POS_ACCEL),

    // @Param: PVE4
    // @DisplayName: SERVO_MOCR_PVE4
    // @Description: Position controller velocity for fourth roboclaw
    // @Range: 0 1e9
    // @Units: QPPS
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PVE4", 59, AP_MotionController, controlSettings[3][12], AP_MOTIONCONTROLLER_POS_VEL),
	
    AP_GROUPEND};

AP_MotionController::AP_MotionController()
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
    QPPM = QPPR / (2 * M_PI * Rw); // Quadrature pulses per meter
    QPPRAD = QPPR / (2 * M_PI);    // Quadrature pulses per Radian
}

AP_MotionController::~AP_MotionController()
{

}

void AP_MotionController::init(void)
{
    AP_SerialManager &serial_manager = AP::serialmanager();
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ESCTelemetry, 0);
    roboclaw.init(port, 10000);
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
    //map the value from the scale (in_min->in_max) into scale (out_min->out_max)
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void AP_MotionController::update(uint16_t steering, uint16_t throttle)
{
    static int counter = 0;
    uint32_t m1 = AP_MotionController::constrain_map(steering, SRV_Channels::srv_channel(1)->get_output_min(), SRV_Channels::srv_channel(1)->get_output_max(), 0, 127);
    uint32_t m2 = AP_MotionController::constrain_map(steering, SRV_Channels::srv_channel(3)->get_output_min(), SRV_Channels::srv_channel(3)->get_output_max(), 0, 127);
    roboclaw.ForwardBackwardM1(Address[0], m1);
    roboclaw.ForwardBackwardM2(Address[0], m2);
    //roboclaw.SpeedM1M2(Address[0], steering, throttle);
    if (++counter >= 10) // Call every fifth of a second (50Hz calling rate)
    {
        //hal.console->printf("RC1:%d RC2:%d RC3:%d RC4:%d\n", (int)Address[0], (int)Address[1], (int)Address[2], (int)Address[3]);
        //hal.console->printf("Time:%05.3f Steering:%d Throttle:%d\n", AP_HAL::millis64() / 1000.0, m1, m2);
        //port->printf("Time:%05.3f Steering:%d Throttle:%d\n", AP_HAL::millis64()/1000.0, steering, throttle);
        //gcs().send_text(MAV_SEVERITY_INFO, "Time:%05.3f Steering:%d Throttle:%d", AP_HAL::millis64()/1000.0, steering, throttle);
        DataFlash_Class::instance()->Log_Write("MOCR", "TimeUS,Steering,Throttle", "Qff", AP_HAL::micros64(), m1, m2);
        counter = 0;
    }
}
