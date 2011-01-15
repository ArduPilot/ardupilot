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
  KP_QUAD_ROLL               = 1.100;
  KI_QUAD_ROLL               = 0.200;
  STABLE_MODE_KP_RATE_ROLL   = -0.001;
  
  // default PID values - Pitch
  KP_QUAD_PITCH              = 1.100;
  KI_QUAD_PITCH              = 0.120;
  STABLE_MODE_KP_RATE_PITCH  = -0.001;
  
  // default PID values - Yaw
  Kp_RateYaw                 = 3.500;  // heading P term
  Ki_RateYaw                 = 0.100;  // heading I term
  KP_QUAD_YAW                = 0.200;  // yaw rate P term
  KI_QUAD_YAW                = 0.040;  // yaw rate I term
  STABLE_MODE_KP_RATE_YAW    = -0.010;  // yaw rate D term
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
  
  // capture trims
  heli_read_radio_trims();
  
  // hardcode mids because we will use ccpm
  roll_mid = ROLL_MID;
  pitch_mid = PITCH_MID;
  collective_mid = 1500;
  yaw_mid = (yawMin+yawMax)/2;
  
  // determine which axis APM will control
  roll_control_switch = !SW_DIP1;
  pitch_control_switch = !SW_DIP2;
  yaw_control_switch = !SW_DIP3; 
  collective_control_switch = !SW_DIP4;
  //position_control_switch = !SW_DIP4;  // switch 4 controls whether we will do GPS hold or not
}

/*****************************************************************************************************/
// heli_read_radio_trims - captures roll, pitch and yaw trims (mids) although only yaw is actually used
//                         trim_yaw is used to center output to the tail which tends to be far from the
//                         physical middle of where the rudder can move.  This helps keep the PID's I
//                         value low and avoid sudden turns left on takeoff
void heli_read_radio_trims()
{
    int i;
    float sumRoll = 0, sumPitch = 0, sumYaw = 0;
    
    // initialiase trims to zero incase this is called more than once
    trim_roll = 0.0;
    trim_pitch = 0.0;
    trim_yaw = 0.0;
    
    // read radio a few times
    for(int i=0; i<10; i++ )
    {
        // make sure new data has arrived
        while( APM_RC.GetState() != 1 )
        {
            delay(20);
        }
        heli_read_radio();
        sumRoll += ch_roll;
        sumPitch += ch_pitch;
        sumYaw += ch_yaw;
    }
    
    // set trim to average 
    trim_roll = sumRoll / 10.0;
    trim_pitch = sumPitch / 10.0;
    trim_yaw = sumYaw / 10.0;
    
    // double check all is ok
    if( trim_roll > 50.0 || trim_roll < -50 )
        trim_roll = 0.0;
    if( trim_pitch >50.0 || trim_roll < -50.0 )
        trim_pitch = 0.0;
    if( trim_yaw > 50.0 || trim_yaw < -50.0 )
        trim_yaw = 0.0;

}

/**********************************************************************/
// Radio decoding
void heli_read_radio()
{
    // left channel
    ccpmPercents.x  = frontLeftCCPMslope * APM_RC.InputCh(CHANNEL_FRONT_LEFT) + frontLeftCCPMintercept;
    
    // right channel
    ccpmPercents.y = frontRightCCPMslope * APM_RC.InputCh(CHANNEL_FRONT_RIGHT) + frontRightCCPMintercept;
    
    // rear channel
    ccpmPercents.z = rearCCPMslope * APM_RC.InputCh(CHANNEL_REAR) + rearCCPMintercept;
    
    // decode the ccpm
    rollPitchCollPercent = ccpmDeallocation * ccpmPercents;
    
    // get the yaw (not coded)
    yawPercent = (yawSlope * APM_RC.InputCh(CHANNEL_YAW) + yawIntercept) - trim_yaw;
    
    // put decoded values into the global variables
    ch_roll = rollPitchCollPercent.x;
    ch_pitch = rollPitchCollPercent.y;
    ch_collective = rollPitchCollPercent.z;
    
    // allow a bit of a dead zone for the yaw
    if( fabs(yawPercent) < 2 )
        ch_yaw = 0;
    else
        ch_yaw = yawPercent;
    
    // get the aux channel (for tuning on/off autopilot)
    ch_aux = APM_RC.InputCh(CH_5) * ch_aux_slope + ch_aux_offset;        
    
    // convert to absolute angles
    command_rx_roll = ch_roll / HELI_STICK_TO_ANGLE_FACTOR;        // Convert stick position to absolute angles
    command_rx_pitch = ch_pitch / HELI_STICK_TO_ANGLE_FACTOR;      // Convert stick position to absolute angles
    command_rx_collective = ch_collective;
    command_rx_yaw = ch_yaw / HELI_YAW_STICK_TO_ANGLE_FACTOR;      // Convert stick position to turn rate
    
    // for use by non-heli parts of code
    ch_throttle = 1000 + (ch_collective * 10);
    
    // hardcode flight mode
    flightMode = STABLE_MODE;
    
    // Autopilot mode (only works on Stable mode)
    if (flightMode == STABLE_MODE)
    {
      if(ch_aux < 1300) {
        AP_mode = AP_AUTOMATIC_MODE;   // Automatic mode : GPS position hold mode + altitude hold
        //SerPrln("autopilot ON!");
      }else{ 
        AP_mode = AP_NORMAL_MODE;   // Normal mode
        //SerPrln("autopilot OFF!");
      }
    }    
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
    control_collective = command_collective;

    // ROLL CONTROL -- ONE PID
    if( roll_control_switch ) 
    {
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
    }else{
        // straight pass through
        control_roll = ch_roll;
    }
  
    // PITCH CONTROL -- ONE PIDS
    if( pitch_control_switch ) 
    {    
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
    }else{
        control_pitch = ch_pitch;
    }

    // YAW CONTROL
    if( yaw_control_switch ) 
    {     
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
          
            // I term
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
      
        // I term
        yaw_I += err_yaw * heli_G_Dt * KI_QUAD_YAW;
        yaw_I = constrain(yaw_I, -20, 20);
        // D term
        yaw_D = (currentYawRate-previousYawRate) * STABLE_MODE_KP_RATE_YAW; // Take into account the change in angular velocity
        yaw_D = constrain(yaw_D,-25,25);    
        previousYawRate = currentYawRate;
      
        // PID control
        control_yaw = trim_yaw + (KP_QUAD_YAW*err_yaw + yaw_I + yaw_D);  // add back in the yaw trim so that it is our center point
        control_yaw = constrain(control_yaw,-50,50);
    }else{
        // straight pass through
        control_yaw = ch_yaw;
    }
    
    Log_Write_PID(7,KP_QUAD_ROLL*err_roll,roll_I,roll_D,control_roll);
    Log_Write_PID(8,KP_QUAD_PITCH*err_pitch,pitch_I,pitch_D,control_pitch);
    Log_Write_PID(9,Kp_RateYaw*err_heading,heading_I,0,control_yaw_rate);
    Log_Write_PID(10,KP_QUAD_YAW*err_yaw,yaw_I,yaw_D,control_yaw);
}

#endif  // #if AIRFRAME == HELI
