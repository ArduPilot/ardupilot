/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Mixer.pde
 Version  : v1.0, Aug 27, 2010
 Author(s): ArduCopter Team
 Jose Julio, Jordi Muñoz,
 Jani Hirvinen, Roberto Navoni,          
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
#ifdef USEMIXER

#include <stdlib.h>

float MotorPitch[MAX_MOTORS];
float MotorRoll[MAX_MOTORS];
float MotorYaw[MAX_MOTORS];
float MotorGas[MAX_MOTORS];
float motorAxisCommandPitch[MAX_MOTORS];
float motorAxisCommandRoll[MAX_MOTORS];
float motorAxisCommandYaw[MAX_MOTORS];
//float motorAxisCommand[3]; // Command on Roll YAW PITCH Recived from IMU


#if FRAME_MODEL == QUAD
// This is an example of QuadX configuration for mixertable
void init_mixer_table()
{
  // Example for Quad configuration
  MotorGas[0]     = 100;
  MotorPitch[0]   = -100;  
  MotorRoll[0]    = 100;  
  MotorYaw[0]     = 100;  

  MotorGas[1]     = 100;
  MotorPitch[1]   = 100;  
  MotorRoll[1]    = -100;  
  MotorYaw[1]     = 100;  

  MotorGas[2]     = 100;
  MotorPitch[2]   = -100; 
  MotorRoll[2]    = -100;  
  MotorYaw[2]     = -100;  

  MotorGas[3]     = 100;
  MotorPitch[3]   = 100; 
  MotorRoll[3]    = 100; 
  MotorYaw[3]     = -100;  
}

#elif FRAME_MODEL == HEXA
void init_mixer_table()
{
  // Hexa configuration
  MotorPitch[0]   = -100;  
  MotorRoll[0]    = 0;  
  MotorYaw[0]     = 100;  

  MotorPitch[1]   = -50;  
  MotorRoll[1]    = -100;  
  MotorYaw[1]     = -100;  

  MotorPitch[2]   = +50; 
  MotorRoll[2]    = -100;  
  MotorYaw[2]     = 100;  

  MotorPitch[3]   =  +100;  
  MotorRoll[3]    =  0;  
  MotorYaw[3]     = -100;  


  MotorPitch[4]   = +50;  
  MotorRoll[4]    =  100;  
  MotorYaw[4]     =  100;  

  MotorPitch[5]   =  -50;  
  MotorRoll[5]    =  100;  
  MotorYaw[5]     =  -100;  
}
#elif FRAME_MODEL == HEXARADIAL
void init_mixer_table()
{
  // Example for Hexa configuration
  MotorGas[0]     = 100;
  MotorPitch[0]   = -100;  
  MotorRoll[0]    = 0;  
  MotorYaw[0]     = 100;  

  MotorGas[1]     = 100;
  MotorPitch[1]   = -50;  
  MotorRoll[1]    = -100;  
  MotorYaw[1]     = -100;  

  MotorGas[2]     = 100;
  MotorPitch[2]   = +50; 
  MotorRoll[2]    = -100;  
  MotorYaw[2]     = 100;  

  MotorGas[3]     = 100;
  MotorPitch[3]   = +100;  
  MotorRoll[3]    = 0;  
  MotorYaw[3]     = -100;  

  MotorGas[4]     = 100;
  MotorPitch[4]   = +50;  
  MotorRoll[4]    =  100;  
  MotorYaw[4]     =  100;  

  MotorGas[5]     = 100;
  MotorPitch[5]   = -50;  
  MotorRoll[5]    = 100;  
  MotorYaw[5]     = -100;  
}

#elif FRAME_MODEL == HEXACOAXIAL
void init_mixer_table()
{
  // Example for Hexa configuration
  MotorGas[0]     = 95;
  MotorPitch[0]   = 100;  
  MotorRoll[0]    = 0;  
  MotorYaw[0]     = 100; 

  MotorGas[1]     = 100;
  MotorPitch[1]   = 100;  
  MotorRoll[1]    = 0;  
  MotorYaw[1]     = -100;  

  MotorGas[2]     = 95;
  MotorPitch[2]   = -50; 
  MotorRoll[2]    =  100;  
  MotorYaw[2]     =  -100;  

  MotorGas[3]     = 100;
  MotorPitch[3]   =  -50;  
  MotorRoll[3]    =  100;  
  MotorYaw[3]     =  100;  

  MotorGas[4]     = 95;
  MotorPitch[4]   = -50;  
  MotorRoll[4]    =  -100; 
  MotorYaw[4]     =  -100;  

  MotorGas[5]     = 100;
  MotorPitch[5]   =  -50;  
  MotorRoll[5]    =  -100;  
  MotorYaw[5]     =  100;  
}

#elif FRAME_MODEL == OCTO
void init_mixer_table()
{
  // Octo configuration. Motors are numberd CW viewed from above starting at front = 1 (CW prop rotation)
  // Motor rotation is CCW for odd numbered motors
  MotorGas[0]     = 100;
  MotorPitch[0]   = 100;  
  MotorRoll[0]    = 0;  
  MotorYaw[0]     = -100;  

  MotorGas[1]     = 100;
  MotorPitch[1]   = 100;  
  MotorRoll[1]    = -100;  
  MotorYaw[1]     = 100;  

  MotorGas[2]     = 100;
  MotorPitch[2]   = 0; 
  MotorRoll[2]    = -100;  
  MotorYaw[2]     = -100;  

  MotorGas[3]     = 100;
  MotorPitch[3]   = -100; 
  MotorRoll[3]    = -100; 
  MotorYaw[3]     = 100;  

  MotorGas[4]     = 100;
  MotorPitch[4]   = -100; 
  MotorRoll[4]    = 0; 
  MotorYaw[4]     = -100;  

  MotorGas[5]     = 100;
  MotorPitch[5]   = -100; 
  MotorRoll[5]    = 100; 
  MotorYaw[5]     = 100;  

  MotorGas[6]     = 100;
  MotorPitch[6]   = 0; 
  MotorRoll[6]    = 100; 
  MotorYaw[6]     = -100; 

  MotorGas[7]     = 100;
  MotorPitch[7]   = 100; 
  MotorRoll[7]    = 100; 
  MotorYaw[7]     = 100;  
}
#else
# error You need define your frame configuration on ArduUser.h
#endif



void motor_axis_correction()
{
  int i;
  for (i=0;i<MAX_MOTORS;i++)
  {
    motorAxisCommandPitch[i] = (control_pitch / 100.0) * MotorPitch[i];
    motorAxisCommandRoll[i] = (control_roll / 100.0) * MotorRoll[i];
    motorAxisCommandYaw[i] = (control_yaw / 100.0) * MotorYaw[i];
  }
}


//After that we can mix them together:
void motor_matrix_command()
{
  int i;
  float valuemotor;
  for (i=0;i<MAX_MOTORS;i++)
  {
    valuemotor = ((ch_throttle* MotorGas[i])/100) + motorAxisCommandPitch[i] + motorAxisCommandYaw[i] + motorAxisCommandRoll[i];
    //valuemotor = Throttle + motorAxisCommandPitch[i] + motorAxisCommandYaw[i] + motorAxisCommandRoll[i]; // OLD VERSION WITHOUT GAS CONOL ON Mixertable
      valuemotor = constrain(valuemotor, 1000, 2000);
    motorCommand[i]=valuemotor;
  }
}


void matrix_debug()
{

  Serial.println();
  Serial.println("--------------------------");
  Serial.println("        Motori Mixertable " );
  Serial.println("--------------------------");
  Serial.println();
  Serial.println("--------------------------");
  Serial.println("   Quad  Motor Debug     " );
  Serial.println("--------------------------");

  Serial.print("AIL:");
  Serial.print(ch_roll);
  Serial.print(" ELE:");
  Serial.print(ch_pitch);
  Serial.print(" THR:");
  Serial.print( ch_throttle);
  Serial.print(" YAW:");
  Serial.print( ch_yaw);
  Serial.print(" AUX:");
  Serial.print(ch_aux);
  Serial.print(" AUX2:");
  Serial.print(ch_aux2);
  Serial.println();
  Serial.print("CONTROL_ROLL:");
  Serial.print(control_roll);
  Serial.print(" CONTROL_PITCH:");
  Serial.print(control_pitch);
  Serial.print(" CONTROL_YAW:");
  Serial.print(control_yaw);
  Serial.print(" SONAR_VALUE:");
  //  Serial.print(sonar_value);
  //  Serial.print(" TARGET_SONAR_VALUE:");
  //  Serial.print(target_sonar_altitude);
  //  Serial.print(" ERR_SONAR_VALUE:");
  //  Serial.print(err_altitude);
  //  Serial.println();
  //  Serial.print("latitude:");
  //  Serial.print(GPS_np.Lattitude);
  //  Serial.print(" longitude:");
  //  Serial.print(GPS_np.Longitude);
  Serial.print(" command gps roll:");
  Serial.print(command_gps_roll);
  Serial.print(" command gps pitch:");
  Serial.print(command_gps_pitch);
  //  Serial.print(" Lon_diff:");
  //  Serial.print(Lon_diff);
  //  Serial.print(" Lon_diff");
  //  Serial.print(command_gps_pitch);
  Serial.println();
  Serial.print("AP MODE:");
  Serial.print((int)AP_mode);
  Serial.println();

#ifdef HEXARADIAL
  Serial.println();
  Serial.print((unsigned int)MotorI2C[5]);
  comma();
  Serial.print((unsigned int)MotorI2C[0]);
  comma();
  Serial.print((unsigned int)MotorI2C[1]);
  comma();
  Serial.println();
  Serial.print((unsigned int)MotorI2C[4]);
  comma();
  Serial.print((unsigned int)MotorI2C[3]);
  comma();
  Serial.println((unsigned int)MotorI2C[2]);
  Serial.println("---------------");
  Serial.println();
#endif

}


#endif    // usemixer
