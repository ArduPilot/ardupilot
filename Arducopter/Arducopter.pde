/* ********************************************************************** */
/*                    ArduCopter Quadcopter code                          */
/*                                                                        */
/* Quadcopter code from AeroQuad project and ArduIMU quadcopter project   */
/* IMU DCM code from Diydrones.com                                        */
/* (Original ArduIMU code from Jordi Muñoz and William Premerlani)        */
/* Ardupilot core code : from DIYDrones.com development team              */
/* Authors : Arducopter development team                                  */
/*           Ted Carancho (aeroquad), Jose Julio, Jordi Muñoz,            */
/*           Jani Hirvinen, Ken McEwans, Roberto Navoni,                  */
/*           Sandro Benigno, Chris Anderson                               */
/* Date : 04-07-2010                                                      */
/* Version : 1.3 beta                                                     */
/* Hardware : ArduPilot Mega + Sensor Shield (Production versions)        */
/* Mounting position : RC connectors pointing backwards                   */
/* This code use this libraries :                                         */
/*   APM_RC : Radio library (with InstantPWM)                             */
/*   APM_ADC : External ADC library                                       */
/*   DataFlash : DataFlash log library                                    */
/*   APM_BMP085 : BMP085 barometer library                                */
/*   APM_Compass : HMC5843 compass library [optional]                     */
/*   GPS_UBLOX or GPS_NMEA: GPS library    [optional]                     */
/* ********************************************************************** */

/*
**** Switch Functions *****
AUX1 ON = Stable Mode
AUX1 OFF = Acro Mode
GEAR ON = GPS Hold
GEAR OFF = Flight Assist (Stable Mode)

**** LED Feedback ****
Green LED On = APM Initialization Finished
Yellow LED On = GPS Hold Mode
Yellow LED Off = Flight Assist Mode (No GPS)
Red LED On = GPS Fix
Red LED Off = No GPS Fix
*/

#include <Wire.h>
#include <APM_ADC.h>
#include <APM_RC.h>
#include <DataFlash.h>
#include <APM_Compass.h>
// Put your GPS library here:
#include <GPS_NMEA.h>  // MTK GPS
//#include <GPS_UBLOX.h>

// EEPROM storage for user configurable values
#include <EEPROM.h>
#include "UserSettings.h"

/* APM Hardware definitions */
#define LED_Yellow 36
#define LED_Red 35
#define LED_Green 37
#define RELE_pin 47
#define SW1_pin 41
#define SW2_pin 40
/* *** */

/* ***************************************************************************** */
/*  CONFIGURATION PART                                                           */
/* ***************************************************************************** */

// ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
// Tested value : 408
#define GRAVITY 408 //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

// IDG500 Sensitivity (from datasheet) => 2.0mV/º/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
// Tested values : 
#define Gyro_Gain_X 0.4  //X axis Gyro gain
#define Gyro_Gain_Y 0.41 //Y axis Gyro gain
#define Gyro_Gain_Z 0.41 //Z axis Gyro gain
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

/*For debugging purposes*/
#define OUTPUTMODE 1  //If value = 1 will print the corrected data, 0 will print uncorrected data of the gyros (with drift), 2 Accel only data

//Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
uint8_t sensors[6] = {1,2,0,4,5,6};  // For ArduPilot Mega Sensor Shield Hardware

//Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
int SENSOR_SIGN[]={1,-1,-1,-1,1,1,-1,-1,-1};  //{-1,1,-1,1,-1,1,-1,-1,-1};

int AN[6]; //array that store the 6 ADC channels
int AN_OFFSET[6]; //Array that store the Offset of the gyros and accelerometers

float G_Dt=0.02;    // Integration time for the gyros (DCM algorithm)

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Accel_Vector_unfiltered[3]= {0,0,0}; //Store the acceleration in a vector
//float Accel_magnitude;
//float Accel_weight;
float Gyro_Vector[3]= {0,0,0};//Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};
float errorCourse=0;
float COGX=0; //Course overground X axis
float COGY=1; //Course overground Y axis

float roll=0;
float pitch=0;
float yaw=0;

unsigned int counter=0;

float DCM_Matrix[3][3]= {
  {1,0,0}
  ,{0,1,0}
  ,{0,0,1}
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here

float Temporary_Matrix[3][3]={
  {0,0,0}
  ,{0,0,0}
  ,{0,0,0}
};

// GPS variables
float speed_3d=0;
int GPS_ground_speed=0;

long timer=0; //general porpuse timer 
long timer_old;

// Attitude control variables
float command_rx_roll=0;        // User commands
float command_rx_roll_old;
float command_rx_roll_diff;
float command_rx_pitch=0;
float command_rx_pitch_old;
float command_rx_pitch_diff;
float command_rx_yaw=0;
float command_rx_yaw_diff;
int control_roll;           // PID control results
int control_pitch;
int control_yaw;
float K_aux;

// Attitude PID controls
float roll_I=0;
float roll_D;
float err_roll;
float pitch_I=0;
float pitch_D;
float err_pitch;
float yaw_I=0;
float yaw_D;
float err_yaw;

//Position control
long target_longitude;
long target_lattitude;
byte target_position;
float gps_err_roll;
float gps_err_roll_old;
float gps_roll_D;
float gps_roll_I=0;
float gps_err_pitch;
float gps_err_pitch_old;
float gps_pitch_D;
float gps_pitch_I=0;
float command_gps_roll;
float command_gps_pitch;

//Altitude control
int Initial_Throttle;
int target_sonar_altitude;
int err_altitude;
int err_altitude_old;
float command_altitude;
float altitude_I;
float altitude_D;

// Sonar variables
int Sonar_value=0;
#define SonarToCm(x) (x*1.26)   // Sonar raw value to centimeters
int Sonar_Counter=0;

// AP_mode : 1=> Position hold  2=>Stabilization assist mode (normal mode)
byte AP_mode = 2;             

long t0;
int num_iter;
float aux_debug;

// Radio definitions
int Neutro_yaw;
int ch_roll;
int ch_pitch;
int ch_throttle;
int ch_yaw;
int ch_aux;
int ch_aux2;
#define CHANN_CENTER 1500
#define MIN_THROTTLE 1040       // Throttle pulse width at minimun...

// Motor variables
#define FLIGHT_MODE_+
//#define FLIGHT_MODE_X
int frontMotor;
int backMotor;
int leftMotor;
int rightMotor;
byte motorArmed = 0;
int minThrottle = 0;

// Serial communication
#define CONFIGURATOR
char queryType;
long tlmTimer = 0;

/* ************************************************************ */
/* Altitude control... (based on sonar) */
void Altitude_control(int target_sonar_altitude)
{
  err_altitude_old = err_altitude;
  err_altitude = target_sonar_altitude - Sonar_value;  
  altitude_D = (float)(err_altitude-err_altitude_old)/G_Dt;
  altitude_I += (float)err_altitude*G_Dt;
  altitude_I = constrain(altitude_I,-100,100);
  command_altitude = Initial_Throttle + KP_ALTITUDE*err_altitude + KD_ALTITUDE*altitude_D + KI_ALTITUDE*altitude_I;
}

/* ************************************************************ */
/* Position control... */
void Position_control(long lat_dest, long lon_dest)
{
  long Lon_diff;
  long Lat_diff;
  float gps_err_roll;
  float gps_err_pitch;
  
  Lon_diff = lon_dest - GPS.Longitude;
  Lat_diff = lat_dest - GPS.Lattitude;
  
  // ROLL
  gps_err_roll_old = gps_err_roll;
  //Optimization : cos(yaw) = DCM_Matrix[0][0] ;  sin(yaw) = DCM_Matrix[1][0] 
  gps_err_roll = (float)Lon_diff*GEOG_CORRECTION_FACTOR*DCM_Matrix[0][0] - (float)Lat_diff*DCM_Matrix[1][0];
  
  gps_roll_D = (gps_err_roll-gps_err_roll_old)/G_Dt;
  
  gps_roll_I += gps_err_roll*G_Dt;
  gps_roll_I = constrain(gps_roll_I,-500,500);
  
  command_gps_roll = KP_GPS_ROLL*gps_err_roll + KD_GPS_ROLL*gps_roll_D + KI_GPS_ROLL*gps_roll_I;
  command_gps_roll = constrain(command_gps_roll,-GPS_MAX_ANGLE,GPS_MAX_ANGLE); // Limit max command
  
  // PITCH
  gps_err_pitch_old = gps_err_pitch;
  gps_err_pitch = -(float)Lat_diff*DCM_Matrix[0][0]- (float)Lon_diff*GEOG_CORRECTION_FACTOR*DCM_Matrix[1][0];
  
  gps_pitch_D = (gps_err_pitch-gps_err_pitch_old)/G_Dt;
  
  gps_pitch_I += gps_err_pitch*G_Dt;
  gps_pitch_I = constrain(gps_pitch_I,-500,500);
  
  command_gps_pitch = KP_GPS_PITCH*gps_err_pitch + KD_GPS_PITCH*gps_pitch_D + KI_GPS_PITCH*gps_pitch_I;
  command_gps_pitch = constrain(command_gps_pitch,-GPS_MAX_ANGLE,GPS_MAX_ANGLE); // Limit max command
}

/* ************************************************************ */
// ROLL, PITCH and YAW PID controls... 
// Input : desired Roll, Pitch and Yaw absolute angles. Output : Motor commands

// Stable Mode
void Attitude_control()
{
  // ROLL CONTROL    
  if (AP_mode==2)        // Normal Mode => Stabilization mode
    err_roll = command_rx_roll - ToDeg(roll);
  else
    err_roll = (command_rx_roll + command_gps_roll) - ToDeg(roll);  // Position control
    
  err_roll = constrain(err_roll,-25,25);  // to limit max roll command...
  
  roll_I += err_roll*G_Dt;
  roll_I = constrain(roll_I,-20,20);
  // D term implementation => two parts: gyro part and command part
  // To have a better (faster) response we can use the Gyro reading directly for the Derivative term...
  // Omega[] is the raw gyro reading plus Omega_I, so it´s bias corrected
  // We also add a part that takes into account the command from user (stick) to make the system more responsive to user inputs
  roll_D = command_rx_roll_diff*KD_QUAD_COMMAND_PART - ToDeg(Omega[0]);  // Take into account Angular velocity of the stick (command)
  
  // PID control
  K_aux = KP_QUAD_ROLL; // Comment this out if you want to use transmitter to adjust gain
  control_roll = K_aux*err_roll + KD_QUAD_ROLL*roll_D + KI_QUAD_ROLL*roll_I; 
  
  // PITCH CONTROL
  if (AP_mode==2)        // Normal mode => Stabilization mode
    err_pitch = command_rx_pitch - ToDeg(pitch);
  else
    err_pitch = (command_rx_pitch + command_gps_pitch) - ToDeg(pitch);  // Position Control
    
  err_pitch = constrain(err_pitch,-25,25);  // to limit max pitch command...
  
  pitch_I += err_pitch*G_Dt;
  pitch_I = constrain(pitch_I,-20,20);
  // D term
  pitch_D = command_rx_pitch_diff*KD_QUAD_COMMAND_PART - ToDeg(Omega[1]);
 
  // PID control
  K_aux = KP_QUAD_PITCH; // Comment this out if you want to use transmitter to adjust gain
  control_pitch = K_aux*err_pitch + KD_QUAD_PITCH*pitch_D + KI_QUAD_PITCH*pitch_I; 
  
  // YAW CONTROL
  
    err_yaw = command_rx_yaw - ToDeg(yaw);
    if (err_yaw > 180)    // Normalize to -180,180
      err_yaw -= 360;
    else if(err_yaw < -180)
      err_yaw += 360;
  
  err_yaw = constrain(err_yaw,-60,60);  // to limit max yaw command...
  
  yaw_I += err_yaw*G_Dt;
  yaw_I = constrain(yaw_I,-20,20);
  yaw_D = command_rx_yaw_diff*KD_QUAD_COMMAND_PART - ToDeg(Omega[2]);
 
  // PID control
  control_yaw = KP_QUAD_YAW*err_yaw + KD_QUAD_YAW*yaw_D + KI_QUAD_YAW*yaw_I;
}

// Acro Mode
void Rate_control()
{
  static float previousRollRate, previousPitchRate, previousYawRate;
  float currentRollRate, currentPitchRate, currentYawRate;
  
  // ROLL CONTROL
  currentRollRate = read_adc(0);      // I need a positive sign here
  
  err_roll = ((ch_roll-1500) * xmitFactor) - currentRollRate;
  
  roll_I += err_roll*G_Dt;
  roll_I = constrain(roll_I,-20,20);

  roll_D = currentRollRate - previousRollRate;
  previousRollRate = currentRollRate;
  
  // PID control
  control_roll = Kp_RateRoll*err_roll + Kd_RateRoll*roll_D + Ki_RateRoll*roll_I; 
  
  // PITCH CONTROL
  currentPitchRate = read_adc(1);
  // err_pitch = ((1500-ch_pitch) * xmitFactor) - currentPitchRate; // was incorrect, inverted ELE between Arco / Stable
  err_pitch = ((ch_pitch - 1500) * xmitFactor) - currentPitchRate;  // correct one, now ELE is ok on both modes
  
  pitch_I += err_pitch*G_Dt;
  pitch_I = constrain(pitch_I,-20,20);

  pitch_D = currentPitchRate - previousPitchRate;
  previousPitchRate = currentPitchRate;
 
  // PID control
  control_pitch = Kp_RatePitch*err_pitch + Kd_RatePitch*pitch_D + Ki_RatePitch*pitch_I; 
  
  // YAW CONTROL
  currentYawRate = read_adc(2);
  err_yaw = ((ch_yaw-1500)* xmitFactor) - currentYawRate;
  
  yaw_I += err_yaw*G_Dt;
  yaw_I = constrain(yaw_I,-20,20);

  yaw_D = currentYawRate - previousYawRate;
  previousYawRate = currentYawRate;
 
  // PID control
  K_aux = KP_QUAD_YAW; // Comment this out if you want to use transmitter to adjust gain
  control_yaw = Kp_RateYaw*err_yaw + Kd_RateYaw*yaw_D + Ki_RateYaw*yaw_I; 
}

// Maximun slope filter for radio inputs... (limit max differences between readings)
int channel_filter(int ch, int ch_old)
{
  int diff_ch_old;
  
  if (ch_old==0)      // ch_old not initialized
    return(ch);
  diff_ch_old = ch - ch_old;      // Difference with old reading
  if (diff_ch_old<0)
     {
     if (diff_ch_old<-40)
       return(ch_old-40);        // We limit the max difference between readings
     }
  else
     {
     if (diff_ch_old>40)    
       return(ch_old+40);
     }
  //return((ch+ch_old)>>1);   // Small filtering
  return(ch);
}


/* ****** SETUP ********************************************************************* */
void setup()
{
  int i;
  float aux_float[3];
  
  pinMode(LED_Yellow,OUTPUT); //Yellow LED A  (PC1)
  pinMode(LED_Red,OUTPUT);    //Red LED B     (PC2)
  pinMode(LED_Green,OUTPUT);  //Green LED C   (PC0)
  
  pinMode(SW1_pin,INPUT);  //Switch SW1 (pin PG0)
  
  pinMode(RELE_pin,OUTPUT);      // Rele output
  digitalWrite(RELE_pin,LOW);
  
  delay(250);
  
  APM_RC.Init();    // APM Radio initialization
  APM_ADC.Init();   // APM ADC library initialization
  DataFlash.Init(); // DataFlash log initialization
  GPS.Init();       // GPS Initialization
  
  readUserConfig(); // Load user configurable items from EEPROM

  // RC channels Initialization (Quad motors)  
  APM_RC.OutputCh(0,MIN_THROTTLE);  // Motors stoped
  APM_RC.OutputCh(1,MIN_THROTTLE);
  APM_RC.OutputCh(2,MIN_THROTTLE);
  APM_RC.OutputCh(3,MIN_THROTTLE);
  
  if (MAGNETOMETER == 1)
    APM_Compass.Init();  // I2C initialization
 
  DataFlash.StartWrite(1);   // Start a write session on page 1
  
  //Serial.begin(57600);
  Serial.begin(115200);
  //Serial.println();
  //Serial.println("ArduCopter Quadcopter v1.0");
  
  // Check if we enable the DataFlash log Read Mode (switch)
  // If we press switch 1 at startup we read the Dataflash eeprom
  while (digitalRead(SW1_pin)==0)
    {
    Serial.println("Entering Log Read Mode...");
    Log_Read(1,1000);
    delay(30000);
    }
    
  //delay(3000);
  
  Read_adc_raw();
  delay(20);
 
  // Offset values for accels and gyros...
  AN_OFFSET[3] = acc_offset_x;
  AN_OFFSET[4] = acc_offset_y;
  AN_OFFSET[5] = acc_offset_z;
  aux_float[0] = gyro_offset_roll;
  aux_float[1] = gyro_offset_pitch;
  aux_float[2] = gyro_offset_yaw;
 
  // Take the gyro offset values
  for(i=0;i<300;i++)
    {
    Read_adc_raw();
    for(int y=0; y<=2; y++)   // Read initial ADC values for gyro offset.
      {
      aux_float[y]=aux_float[y]*0.8 + AN[y]*0.2;
      //Serial.print(AN[y]);
      //Serial.print(",");
      }
    //Serial.println();
    Log_Write_Sensor(AN[0],AN[1],AN[2],AN[3],AN[4],AN[5],ch_throttle);
    delay(14);
    }
  for(int y=0; y<=2; y++)   
    AN_OFFSET[y]=aux_float[y];
    
  Neutro_yaw = APM_RC.InputCh(3); // Take yaw neutral radio value
  #ifndef CONFIGURATOR
  for(i=0;i<6;i++)
    {
    Serial.print("AN[]:");
    Serial.println(AN_OFFSET[i]);
    }
   
  Serial.print("Yaw neutral value:");
  Serial.println(Neutro_yaw);
  #endif
  
  #if (RADIO_TEST_MODE)    // RADIO TEST MODE TO TEST RADIO CHANNELS
  while(1)
   {
   if (APM_RC.GetState()==1)
     {
     Serial.print("AIL:");
     Serial.print(APM_RC.InputCh(0));
     Serial.print("ELE:");
     Serial.print(APM_RC.InputCh(1));
     Serial.print("THR:");
     Serial.print(APM_RC.InputCh(2));
     Serial.print("YAW:");
     Serial.print(APM_RC.InputCh(3));
     Serial.print("AUX(mode):");
     Serial.print(APM_RC.InputCh(4));
     Serial.print("AUX2:");
     Serial.print(APM_RC.InputCh(5));
     Serial.println();
     delay(200);
     }
   } 
  #endif  

  delay(1000);
  
  DataFlash.StartWrite(1);   // Start a write session on page 1
  timer = millis();
  tlmTimer = millis();
  Read_adc_raw();        // Initialize ADC readings...
  delay(20);
  motorArmed = 0;
  digitalWrite(LED_Green,HIGH);     // Ready to go...
}

/* ***** MAIN LOOP ***** */
void loop(){
  
  int aux;
  int i;
  float aux_float;
  //Log variables
  int log_roll;
  int log_pitch;
  int log_yaw;

  
  if((millis()-timer)>=10)   // Main loop 100Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
    G_Dt = (timer-timer_old)/1000.0;      // Real time of loop run 
    
    // IMU DCM Algorithm
    Read_adc_raw();
    if (MAGNETOMETER == 1) {
      if (counter > 10)  // Read compass data at 10Hz... (10 loop runs)
        {
        counter=0;
        APM_Compass.Read();     // Read magnetometer
        APM_Compass.Calculate(roll,pitch);  // Calculate heading
        }
    }
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
    // *****************
    
    // Output data
    log_roll = ToDeg(roll)*10;
    log_pitch = ToDeg(pitch)*10;
    log_yaw = ToDeg(yaw)*10;

  #ifndef CONFIGURATOR    
    Serial.print(log_roll);
    Serial.print(",");
    Serial.print(log_pitch);
    Serial.print(",");
    Serial.print(log_yaw);
    
    for (int i=0;i<6;i++)
      {
      Serial.print(AN[i]);
      Serial.print(",");
      }
  #endif
    
    // Write Sensor raw data to DataFlash log
    Log_Write_Sensor(AN[0],AN[1],AN[2],AN[3],AN[4],AN[5],ch_throttle);
    // Write attitude to DataFlash log
    Log_Write_Attitude(log_roll,log_pitch,log_yaw);
    
    if (APM_RC.GetState()==1)   // New radio frame?
      {
      // Commands from radio Rx... 
      // Stick position defines the desired angle in roll, pitch and yaw
      ch_roll = channel_filter(APM_RC.InputCh(0),ch_roll);
      ch_pitch = channel_filter(APM_RC.InputCh(1),ch_pitch);
      ch_throttle = channel_filter(APM_RC.InputCh(2),ch_throttle);
      ch_yaw = channel_filter(APM_RC.InputCh(3),ch_yaw);
      ch_aux = APM_RC.InputCh(4);
      ch_aux2 = APM_RC.InputCh(5);
      command_rx_roll_old = command_rx_roll;
      command_rx_roll = (ch_roll-CHANN_CENTER)/12.0;
      command_rx_roll_diff = command_rx_roll-command_rx_roll_old;
      command_rx_pitch_old = command_rx_pitch;
      command_rx_pitch = (ch_pitch-CHANN_CENTER)/12.0;
      command_rx_pitch_diff = command_rx_pitch-command_rx_pitch_old;
      aux_float = (ch_yaw-Neutro_yaw)/180.0;
      command_rx_yaw += aux_float;
      command_rx_yaw_diff = aux_float;
      if (command_rx_yaw > 180)         // Normalize yaw to -180,180 degrees
        command_rx_yaw -= 360.0;
      else if (command_rx_yaw < -180)
        command_rx_yaw += 360.0;
      
      // Read through comments in Attitude_control() if you wish to use transmitter to adjust P gains
      // I use K_aux (channel 6) to adjust gains linked to a knob in the radio... [not used now]
      //K_aux = K_aux*0.8 + ((ch_aux-1500)/100.0 + 0.6)*0.2;
      K_aux = K_aux*0.8 + ((ch_aux2-1500)/300.0 + 1.7)*0.2;   // /300 + 1.0

      if (K_aux < 0)
        K_aux = 0;
      
      //Serial.print(",");
      //Serial.print(K_aux);
 
      // We read the Quad Mode from Channel 5
      if (ch_aux < 1200)
        {
        AP_mode = 1;           // Position hold mode (GPS position control)
        digitalWrite(LED_Yellow,HIGH); // Yellow LED On
        }
      else
        {
        AP_mode = 2;          // Normal mode (Stabilization assist mode)
        digitalWrite(LED_Yellow,LOW); // Yellow LED off
        }
      // Write Radio data to DataFlash log
      Log_Write_Radio(ch_roll,ch_pitch,ch_throttle,ch_yaw,int(K_aux*100),(int)AP_mode);
      }  // END new radio data
      
    if (AP_mode==1)  // Position Control
      {
      if (target_position==0)   // If this is the first time we switch to Position control, actual position is our target position
        {
        target_lattitude = GPS.Lattitude;
        target_longitude = GPS.Longitude;
        #ifndef CONFIGURATOR
        Serial.println();
        Serial.print("* Target:");
        Serial.print(target_longitude);
        Serial.print(",");
        Serial.println(target_lattitude);
        #endif
        target_position=1;
        //target_sonar_altitude = sonar_value;
        //Initial_Throttle = ch3;
        // Reset I terms
        altitude_I = 0;
        gps_roll_I = 0;
        gps_pitch_I = 0;
        }        
      }
    else
      target_position=0;
      
    //Read GPS
    GPS.Read();
    if (GPS.NewData)  // New GPS data?
      {
      GPS.NewData=0;  // We Reset the flag...
      
      //Output GPS data
      //Serial.print(",");
      Serial.print(GPS.Lattitude);
      Serial.print(",");
      Serial.print(GPS.Longitude);
     
      // Write GPS data to DataFlash log
      Log_Write_GPS(GPS.Time, GPS.Lattitude,GPS.Longitude,GPS.Altitude, GPS.Ground_Speed, GPS.Ground_Course, GPS.Fix, GPS.NumSats);
      
      if (GPS.Fix)
        digitalWrite(LED_Red,HIGH);  // GPS Fix => Blue LED
      else
        digitalWrite(LED_Red,LOW);
        
      if (AP_mode==1)
        {
        if ((target_position==1)&&(GPS.Fix))
          {
          Position_control(target_lattitude,target_longitude);  // Call position hold routine
          }
        else
          {
          //Serial.print("NOFIX");
          command_gps_roll=0;
          command_gps_pitch=0;
          }
        }
      }
    
    // Control methodology selected using AUX2
    if (ch_aux2 < 1200)
      Attitude_control();
    else
      {
      Rate_control();
      // Reset yaw, so if we change to stable mode we continue with the actual yaw direction
      command_rx_yaw = ToDeg(yaw);
      command_rx_yaw_diff = 0;
      }
      
    // Arm motor output
    if (ch_throttle < 1200) {
      control_yaw = 0;
      command_rx_yaw = ToDeg(yaw);
      command_rx_yaw_diff = 0;
      if (ch_yaw > 1800) {
        motorArmed = 1;
        minThrottle = 1100;
      }
      if (ch_yaw < 1200) {
        motorArmed = 0;
        minThrottle = MIN_THROTTLE;
      }
    }
      
    // Quadcopter mix
    // Ask Jose if we still need this IF statement, and if we want to do an ESC calibration
    if (motorArmed == 1) {        
      #ifdef FLIGHT_MODE_+
        rightMotor = constrain(ch_throttle - control_roll - control_yaw, minThrottle, 2000);
        leftMotor = constrain(ch_throttle + control_roll - control_yaw, minThrottle, 2000);
        frontMotor = constrain(ch_throttle + control_pitch + control_yaw, minThrottle, 2000);
        backMotor = constrain(ch_throttle - control_pitch + control_yaw, minThrottle, 2000);
      #endif
      #ifdef FLIGHT_MODE_X
        frontMotor = constrain(ch_throttle + control_roll + control_pitch - control_yaw, minThrottle, 2000); // front left motor
        rightMotor = constrain(ch_throttle - control_roll + control_pitch + control_yaw, minThrottle, 2000); // front right motor
        leftMotor = constrain(ch_throttle + control_roll - control_pitch + control_yaw, minThrottle, 2000);  // rear left motor
        backMotor = constrain(ch_throttle - control_roll - control_pitch - control_yaw, minThrottle, 2000);  // rear right motor
      #endif
    }
    if (motorArmed == 0) {
      rightMotor = MIN_THROTTLE;
      leftMotor = MIN_THROTTLE;
      frontMotor = MIN_THROTTLE;
      backMotor = MIN_THROTTLE;
      roll_I = 0;     // reset I terms of PID controls
      pitch_I = 0;
      yaw_I = 0; 
      // Initialize yaw command to actual yaw when throttle is down...
      command_rx_yaw = ToDeg(yaw);
      command_rx_yaw_diff = 0;
    }
    APM_RC.OutputCh(0, rightMotor);    // Right motor
    APM_RC.OutputCh(1, leftMotor);    // Left motor
    APM_RC.OutputCh(2, frontMotor);   // Front motor
    APM_RC.OutputCh(3, backMotor);   // Back motor     
    // InstantPWM
    APM_RC.Force_Out0_Out1();
    APM_RC.Force_Out2_Out3();

    #ifndef CONFIGURATOR
    Serial.println();  // Line END 
    #endif
 }
  #ifdef CONFIGURATOR
  if((millis()-tlmTimer)>=100) {
    readSerialCommand();
    sendSerialTelemetry();
    tlmTimer = millis();
  }
  #endif
}
