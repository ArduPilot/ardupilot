/****************************************************
 * AMS 5600 Angle of Attack Class for Ardupilot platform
 * Author: Cole Mero
 * Date: 15 Dec 2014
 * File: AS5600_AOA.cpp
 * Version 1.00
 * www.ams.com
 *
 * Description:  This class has been designed to
 * access the AS5600 magnetic encoder sensor to
 * read the angle of attack and record it for experimental purposes
 *
 * It was adapted from the Arduino library available for the AS5600 sensor.
 *
***************************************************/

#include <unistd.h>
#include "AS5600_AOA.h"

extern const AP_HAL::HAL &hal;

/****************************************************
 * Method: AS5600_AOA
 * In: none
 * Out: none
 * Description: constructor for AMS 5600
***************************************************/
 AS5600_AOA::AS5600_AOA()
{

    bus = 1; //Sets the bus number for the device, unclear what this number should be, trial and error to make it work
    address = 0x36; //This is the I2C address for the device, it is set by the manufacturer

   /*load register values*/

  _zmco = 0x00;
  _zpos_hi = 0x01;
  _zpos_lo = 0x02;
  _mpos_hi = 0x03;
  _mpos_lo = 0x04;
  _mang_hi = 0x05;
  _mang_lo = 0x06;
  _conf_hi = 0x07;
  _conf_lo = 0x08;
  _raw_ang_hi = 0x0c;
  _raw_ang_lo = 0x0d;
  _ang_hi = 0x0e;
  _ang_lo = 0x0f;
  _stat = 0x0b;
  _agc = 0x1a;
  _mag_hi = 0x1b;
  _mag_lo = 0x1c;
  _burn = 0xff;

}


void AS5600_AOA::init(){

    dev = hal.i2c_mgr->get_device(bus, address);

    WITH_SEMAPHORE(dev->get_semaphore());
    dev->set_speed(AP_HAL::Device::SPEED_LOW);
    dev->set_retries(2);

}

/* mode = 0, output PWM, mode = 1 output analog (full range from 0% to 100% between GND and VDD*/
void AS5600_AOA::setOutPut(unsigned char mode){
    unsigned char config_status;
    config_status = readOneByte(_conf_lo);
    if(mode == 1){
        config_status = config_status & 0xcf;
    }else{
        config_status = config_status & 0xef;
    }

    /* Note significant variance from the Arduino AMS_5600 library in this line.
     *
     * -> writeOneByte(_conf_lo, lowByte(config_status));
     *  vs writeOneByte(_conf_lo, config_status); I have removed the lowByte() function
     *  since it is part of the arduino.h library and I don't have access to it, however,
     *  since it accesses the lowest byte in the variable, and an unsigned char such as
     *  config_status only HAS one byte regardless, I think the end result should be identical?
     */
    writeOneByte(_conf_lo, config_status);
}


/*******************************************************
 * Method: getRawAngle
 * In: none
 * Out: value of raw angle register
 * Description: gets raw value of magnet position.
 * start, end, and max angle settings do not apply
*******************************************************/
unsigned short AS5600_AOA::getRawAngle(void)
{
  unsigned short angleRaw = readTwoBytes(_raw_ang_hi, _raw_ang_lo);

  unsigned short angleDegrees = angleRaw*0.087;

  AP::logger().Write("AoAR", "Status, TimeUS, Angle", "iQH", int(bool(dev)), AP_HAL::micros64(), angleDegrees);

   return angleDegrees;
}
/*******************************************************
 * Method: highByte
 * In: Unsigned short
 * Out: Highest or leftmost byte of the unsigned short
 * Description: Takes in the unsigned short and returns
 * the leftmost or highest bite
*******************************************************/

unsigned char AS5600_AOA::highByte(unsigned short short_in){

    unsigned char hiByte = ((short_in >> 8) & 0xff);
    return  hiByte;
}

/*******************************************************
 * Method: lowByte
 * In: Unsigned short
 * Out: Lowest or rightmost byte of the unsigned short
 * Description: Takes in the unsigned short and returns
 * the rightmost or lowest byte
*******************************************************/

unsigned char AS5600_AOA::lowByte(unsigned short short_in){

    unsigned char loByte = (short_in & 0xff);
    return  loByte;
}


/*******************************************************
 * Method: setMaxAngle
 * In: new maximum angle to set OR none
 * Out: value of max angle register
 * Description: sets a value in maximum angle register.
 * If no value is provided, method will read position of
 * magnet.  Setting this register zeros out max position
 * register.
*******************************************************/
/*unsigned short AS5600_AOA::setMaxAngle(unsigned short newMaxAngle)
{
  unsigned short retVal;
  if(newMaxAngle == -1)
  {
    maxAngle = getRawAngle();
  }
  else
    maxAngle = newMaxAngle;

  writeOneByte(_mang_hi, highByte(maxAngle));
  usleep(2);
  writeOneByte(_mang_lo, lowByte(maxAngle));
  usleep(2);

  retVal = readTwoBytes(_mang_hi, _mang_lo);
  return retVal;
}

*/

/*******************************************************
 * Method: getMaxAngle
 * In: none
 * Out: value of max angle register
 * Description: gets value of maximum angle register.
*******************************************************/
unsigned short AS5600_AOA::getMaxAngle()
{
  return readTwoBytes(_mang_hi, _mang_lo);
}


/*******************************************************
 * Method: setStartPosition
 * In: new start angle position
 * Out: value of start position register
 * Description: sets a value in start position register.
 * If no value is provided, method will read position of
 * magnet.
*******************************************************/
/*
unsigned short AS5600_AOA::setStartPosition(unsigned short startAngle)
{
  if(startAngle == -1)
  {
    rawStartAngle = getRawAngle();
  }
  else
    rawStartAngle = startAngle;

  writeOneByte(_zpos_hi, highByte(rawStartAngle));
  usleep(2);
  writeOneByte(_zpos_lo, lowByte(rawStartAngle));
  usleep(2);
  zPosition = readTwoBytes(_zpos_hi, _zpos_lo);

  return(zPosition);
}

*/

//int AS5600_AOA::writeOneByte(uint8_t in_adr, uint8_t msg){
//
//    WITH_SEMAPHORE(dev->get_semaphore());
//
//    send[2] = {in_adr, msg};
//
//    bool success = dev->transfer(send, sizeof(send), nullptr, 0);
//
//    return success ? 0 : -1;
//
//}


/*******************************************************
 * Method: readOneByte
 * In: register to read
 * Out: data read from i2c
 * Description: reads one byte register from i2c
*******************************************************/
int AS5600_AOA::readOneByte(uint8_t in_adr)
{

  WITH_SEMAPHORE(dev->get_semaphore());

  uint8_t  send[1] = {in_adr};
  uint8_t  recv[1];

  bool success = dev->transfer(send, sizeof(send), recv, sizeof(recv));

  return success ? recv[0] : -1;
}


/*******************************************************
 * Method: readTwoBytes
 * In: register to read
 * Out: data read from i2c
 * Description: reads two bytes register from i2c
*******************************************************/
int AS5600_AOA::readTwoBytes(uint8_t in_adr1, uint8_t in_adr2)
{

  int firstResult =  readOneByte(in_adr1);
  int secondResult = readOneByte(in_adr2);

  if (firstResult == -1 || secondResult == -1){

      return -1;
  }

  uint8_t firstByte = firstResult;
  uint8_t secondByte = secondResult;

  uint16_t combined = (firstByte << 8) | secondByte;

  return combined;

}


