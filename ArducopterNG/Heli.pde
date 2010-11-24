/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Heli.pde
 Desc     : code specific to traditional helicopters
 Version  : v1.0, Aug 27, 2010
 Author(s): ArduCopter Team
             Ted Carancho (aeroquad), Jose Julio, Jordi Muñoz,
             Jani Hirvinen, Ken McEwans, Roberto Navoni,          
             Sandro Benigno, Chris Anderson
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.

* ************************************************************** *
ChangeLog:


* ************************************************************** *
TODO:


* ************************************************************** */

#if AIRFRAME == HELI

/**********************************************************************/
// heli_readUserConfig - reads values in from EEPROM
void heli_readUserConfig() 
{ 
    float magicNum = 0;
    magicNum = readEEPROM(EEPROM_MAGIC_NUMBER_ADDR);
    if( magicNum != EEPROM_MAGIC_NUMBER ) {
        SerPri("No heli settings found in EEPROM.  Using defaults");
        heli_defaultUserConfig();
    }else{
        frontLeftCCPMmin = readEEPROM(FRONT_LEFT_CCPM_MIN_ADDR);
        frontLeftCCPMmax = readEEPROM(FRONT_LEFT_CCPM_MAX_ADDR);
        frontRightCCPMmin = readEEPROM(FRONT_RIGHT_CCPM_MIN_ADDR);
        frontRightCCPMmax = readEEPROM(FRONT_RIGHT_CCPM_MAX_ADDR);
        rearCCPMmin = readEEPROM(REAR_CCPM_MIN_ADDR);
        rearCCPMmax = readEEPROM(REAR_CCPM_MAX_ADDR);
        yawMin = readEEPROM(YAW_MIN_ADDR);
        yawMax = readEEPROM(YAW_MAX_ADDR);
    }
}

/**********************************************************************/
// default the heli specific values to defaults
void heli_defaultUserConfig() 
{
  // default CCPM values.
  frontLeftCCPMmin =        1200;
  frontLeftCCPMmax =        1800;
  frontRightCCPMmin  =      1900;
  frontRightCCPMmax =       1100;
  rearCCPMmin =             1200;
  rearCCPMmax =             1800;
  yawMin =                  1200;
  yawMax =                  1800; 
  
  // default PID values - Roll
  KP_QUAD_ROLL               = 1.000;
  KI_QUAD_ROLL               = 0.150;
  STABLE_MODE_KP_RATE_ROLL   = 0.000;
  
  // default PID values - Pitch
  KP_QUAD_PITCH              = 1.200;
  KI_QUAD_PITCH              = 0.120;
  STABLE_MODE_KP_RATE_PITCH  = 0.000;
  
  // default PID values - Yaw
  Kp_RateYaw                 = 2.000;  // heading P term
  Ki_RateYaw                 = 0.100;  // heading I term
  KP_QUAD_YAW                = 0.150;  // yaw rate P term
  KI_QUAD_YAW                = 0.030;  // yaw rate I term
  STABLE_MODE_KP_RATE_YAW    = 0.000;  // not used
}

/**********************************************************************/
// displaySettings - displays heli specific user settings
void heli_displaySettings() 
{
    SerPri("frontLeftCCPM min: ");
    SerPri(frontLeftCCPMmin);
    SerPri("\tmax:");
    SerPri(frontLeftCCPMmax);
    
    if( abs(frontLeftCCPMmin-frontLeftCCPMmax)<50 || frontLeftCCPMmin < 900 || frontLeftCCPMmin > 2100 || frontLeftCCPMmax < 900 || frontLeftCCPMmax > 2100 )
        SerPrln("\t\t<-- check");
    else
        SerPrln();
    
    SerPri("frontRightCCPM min: ");
    SerPri(frontRightCCPMmin);
    SerPri("\tmax:");
    SerPri(frontRightCCPMmax);
    if( abs(frontRightCCPMmin-frontRightCCPMmax)<50 || frontRightCCPMmin < 900 || frontRightCCPMmin > 2100 || frontRightCCPMmax < 900 || frontRightCCPMmax > 2100 )
        SerPrln("\t\t<-- check");
    else
        SerPrln();    
    
    SerPri("rearCCPM min: ");
    SerPri(rearCCPMmin);
    SerPri("\tmax:");
    SerPri(rearCCPMmax);
    if( abs(rearCCPMmin-rearCCPMmax)<50 || rearCCPMmin < 900 || rearCCPMmin > 2100 || rearCCPMmax < 900 || rearCCPMmax > 2100 )
        SerPrln("\t\t<-- check");
    else
        SerPrln();
    
    SerPri("yaw min: ");
    SerPri(yawMin);
    SerPri("\tmax:");
    SerPri(yawMax);
    if( abs(yawMin-yawMax)<50 || yawMin < 900 || yawMin > 2100 || yawMax < 900 || yawMax > 2100 )
        SerPrln("\t\t<-- check");
    else
        SerPrln();   

    SerPrln();
}

////////////////////////////////////////////////////////////////////////////////
//  Setup Procedure
////////////////////////////////////////////////////////////////////////////////
void heli_setup()
{
  // read heli specific settings (like CCPM values) from EEPROM
  heli_readUserConfig();
  
  // update CCPM values
  frontLeftCCPMslope =      100 / (frontLeftCCPMmax - frontLeftCCPMmin);
  frontLeftCCPMintercept =  100 - (frontLeftCCPMslope * frontLeftCCPMmax);
  frontRightCCPMslope =     100 / (frontRightCCPMmax - frontRightCCPMmin);
  frontRightCCPMintercept = 100 - (frontRightCCPMslope * frontRightCCPMmax);
  rearCCPMslope =           100 / (rearCCPMmax - rearCCPMmin);
  rearCCPMintercept =       100 - (rearCCPMslope * rearCCPMmax);
  yawSlope =                100 / (yawMax - yawMin);
  yawIntercept =            50 - (yawSlope * yawMax);
  
  // hardcode mids because we will use ccpm
  roll_mid = ROLL_MID;
  pitch_mid = PITCH_MID;
  collective_mid = 1500;
  yaw_mid = (yawMin+yawMax)/2;
}

/**********************************************************************/
// Radio decoding
void heli_read_radio()
{
    static int count = 0;
    // left channel
    ccpmPercents.x  = frontLeftCCPMslope * APM_RC.InputCh(CHANNEL_FRONT_LEFT) + frontLeftCCPMintercept;
    
    // right channel
    ccpmPercents.y = frontRightCCPMslope * APM_RC.InputCh(CHANNEL_FRONT_RIGHT) + frontRightCCPMintercept;
    
    // rear channel
    ccpmPercents.z = rearCCPMslope * APM_RC.InputCh(CHANNEL_REAR) + rearCCPMintercept;
    
    // decode the ccpm
    rollPitchCollPercent = ccpmDeallocation * ccpmPercents;
    
    // get the yaw (not coded)
    yawPercent = yawSlope * APM_RC.InputCh(CHANNEL_YAW) + yawIntercept;
    
    // we add 1500 to make it fit in with rest of arduCopter code..
    ch_roll = rollPitchCollPercent.x;
    ch_pitch = rollPitchCollPercent.y;
    ch_collective = rollPitchCollPercent.z;
    
    // allow a bit of a dead zone for the yaw
    if( abs(yawPercent) < 2 )
        ch_yaw = 0;
    else
        ch_yaw = yawPercent;
    
    // convert to absolute angles
    command_rx_roll = ch_roll / HELI_STICK_TO_ANGLE_FACTOR + HELI_ADJUST_ROLL;        // Convert stick position to absolute angles
    command_rx_pitch = ch_pitch / HELI_STICK_TO_ANGLE_FACTOR + HELI_ADJUST_PITCH;      // Convert stick position to absolute angles
    command_rx_collective = ch_collective;
    command_rx_yaw = ch_yaw / HELI_YAW_STICK_TO_ANGLE_FACTOR;      // Convert stick position to turn rate
    
    // hardcode flight mode
    flightMode = STABLE_MODE;
}

/**********************************************************************/
// output to swash plate based on control variables
// Uses these global variables:
// control_roll       : -50 ~ 50
// control_pitch      : -50 ~ 50
// control_collective :   0 ~ 100
// control_yaw        : -50 ~ 50
void heli_moveSwashPlate()
{
    static int count = 0;
    // turn pitch, roll, collective commands into ccpm values (i.e. values for each servo)
    ccpmPercents_out = ccpmAllocation * Vector3f(control_roll, control_pitch, control_collective);

    // calculate values to be sent out to RC Output channels
    leftOut =  (ccpmPercents_out.x - frontLeftCCPMintercept) / frontLeftCCPMslope;
    rightOut = (ccpmPercents_out.y - frontRightCCPMintercept) / frontRightCCPMslope;
    rearOut =  (ccpmPercents_out.z - rearCCPMintercept) / rearCCPMslope;
    yawOut =   (control_yaw - yawIntercept) / yawSlope;
      
    APM_RC.OutputCh(CHANNEL_FRONT_LEFT,leftOut);
    APM_RC.OutputCh(CHANNEL_FRONT_RIGHT,rightOut);
    APM_RC.OutputCh(CHANNEL_REAR,rearOut);
    APM_RC.OutputCh(CHANNEL_YAW,yawOut);
    // InstantPWM
    APM_RC.Force_Out0_Out1();
    APM_RC.Force_Out2_Out3();
}

/**********************************************************************/
// ROLL, PITCH and YAW PID controls... 
// Input : desired Roll, Pitch absolute angles
//         collective as a percentage from 0~100
//         yaw as a rate of rotation
// Output : control_roll - roll servo as a percentage (-50 to 50)
//          control_pitch - pitch servo as a percentage (-50 to 50)
//          control_collective - collective servo as a percentage (0 to 100)
//          control_yaw - yaw servo as a percentage (0 to 100)
void heli_attitude_control(int command_roll, int command_pitch, int command_collective, int command_yaw)
{
    static float firstIteration = 1;
    static float command_yaw_previous = 0;
    static float previousYawRate = 0;
    float stable_roll, stable_pitch;
    float currentYawRate;
    float control_yaw_rate;
    float err_heading;
    static int aCounter = 0;
    float heli_G_Dt;
    
    // get current time
    heli_G_Dt = (currentTimeMicros-heli_previousTimeMicros) * 0.000001;   // Microseconds!!!
    heli_previousTimeMicros = currentTimeMicros;
    
    // always pass through collective command
    control_collective = command_rx_collective;

    // ROLL CONTROL -- TWO PIDS - ANGLE CONTROL + RATE CONTROL
    // P term
    err_roll = command_roll - ToDeg(roll);    
    err_roll = constrain(err_roll,-25,25);  // to limit max roll command...
    // I term
    roll_I += err_roll*heli_G_Dt*KI_QUAD_ROLL;
    roll_I = constrain(roll_I,-10,10);
    // D term
    roll_D = ToDeg(Omega[0]) * STABLE_MODE_KP_RATE_ROLL;  // Take into account Angular velocity
    roll_D = constrain(roll_D,-25,25);
  
    // PID control
    control_roll = KP_QUAD_ROLL*err_roll + roll_I + roll_D;
    control_roll = constrain(control_roll,-50,50);
  
    // PITCH CONTROL -- TWO PIDS - ANGLE CONTROL + RATE CONTROL
    // P term
    err_pitch = command_pitch - ToDeg(pitch);
    err_pitch = constrain(err_pitch,-25,25);  // to limit max pitch command...
    // I term
    pitch_I += err_pitch * heli_G_Dt * KI_QUAD_PITCH;
    pitch_I = constrain(pitch_I,-10,10);
    // D term
    pitch_D = ToDeg(Omega[1]) * STABLE_MODE_KP_RATE_PITCH; // Take into account Angular velocity
    pitch_D = constrain(pitch_D,-25,25);
    // PID control
    control_pitch = KP_QUAD_PITCH*err_pitch + pitch_I + pitch_D;
    control_pitch = constrain(control_pitch,-50,50);


    // YAW CONTROL
    if( command_yaw == 0 )  // heading hold mode 
    {
        // check we don't need to reset targetHeading
        if( command_yaw_previous != 0 )
            targetHeading = ToDeg(yaw);

        // ensure reasonable targetHeading
        if( firstIteration || targetHeading > 180 || targetHeading < -180 )
        {
            firstIteration = 0;
            targetHeading = ToDeg(yaw);
        }
            
        err_heading = Normalize_angle(targetHeading - ToDeg(yaw));     
        err_heading = constrain(err_heading,-90,90);  // don't make it travel any faster beyond 90 degrees
      
        heading_I += err_heading * heli_G_Dt * Ki_RateYaw;
        heading_I = constrain(heading_I,-20,20);

        // PID control - a bit bad - we're using the acro mode's PID values because there's not PID for heading
        control_yaw_rate = Kp_RateYaw*err_heading + heading_I;
        control_yaw_rate = constrain(control_yaw_rate,-100,100);  // to limit max yaw command
    }else{      // rate mode
        err_heading = 0;
        control_yaw_rate = command_yaw;
    }
    command_yaw_previous = command_yaw;

    // YAW RATE CONTROL
    currentYawRate = ToDeg(Gyro_Scaled_Z(read_adc(2)));
    //currentYawRate = ToDeg(Omega_Vector[2]);  <-- makes things very unstable!!
    err_yaw = control_yaw_rate-currentYawRate;
  
    yaw_I += err_yaw * heli_G_Dt * KI_QUAD_YAW;
    yaw_I = constrain(yaw_I, -20, 20);
  
    //yaw_D = currentYawRate - previousYawRate;
    previousYawRate = currentYawRate;
  
    // PID control
    control_yaw = (KP_QUAD_YAW*err_yaw + yaw_I);
    control_yaw = constrain(control_yaw,-50,50);
}

#endif  // #if AIRFRAME == HELI
