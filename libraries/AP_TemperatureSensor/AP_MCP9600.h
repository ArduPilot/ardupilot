/*
 * Mcp9600.h
 *
 *  Created on: Sep 17, 2021
 *      Author: Thach
 */

/*I2C drive for Temperature sensor mcp9600 grove*/
//#ifndef LIBRARIES_AP_TEMPERATURESENSOR_MCP9600GROVE_H_
//#define LIBRARIES_AP_TEMPERATURESENSOR_MCP9600GROVE_H_
//#endif /* LIBRARIES_AP_TEMPERATURESENSOR_MCP9600GROVE_H_ */

#pragma once

//Register pointer list
#define HOT_JUNCTION_TEMP_REG          0b00000000 //Thermocouple Hot-Junction
#define JUNCTION_TEMP_DELTA_REG        0b00000001 //Junctions Temperature Delta
#define COLD_JUNCTION_TEMP_REG         0b00000010 //Cold-Junction Temperature
#define RAW_ADC_DATA_REG               0b00000011 //Raw ADC Data register
#define STAT_REG                       0b00000100 //STATUS
#define THERM_SENS_CFG_REG             0b00000101 //Thermocouple Sensor Configuration
#define DEVICE_CFG_REG                 0b00000110 //Device Configuration

#define ALERT1_CFG_REG                 0b00001000 //Alert 1 Configuration
#define ALERT2_CFG_REG                 0b00001001 //Alert 2 Configuration
#define ALERT3_CFG_REG                 0b00001010 //Alert 3 Configuration
#define ALERT4_CFG_REG                 0b00001011 //Alert 4 Configuration
#define ALERT1_HYS_REG                 0b00001100 //Alert 1 Hysteresis
#define ALERT2_HYS_REG                 0b00001101 //Alert 2 Hysteresis
#define ALERT3_HYS_REG                 0b00001110 //Alert 3 Hysteresis
#define ALERT4_HYS_REG                 0b00001111 //Alert 4 Hysteresis

#define TEMP_ALERT1_LIMIT_REG          0b00010000 //Temperature Alert 1 Limit
#define TEMP_ALERT2_LIMIT_REG          0b00010001 //Temperature Alert 2 Limit
#define TEMP_ALERT3_LIMIT_REG          0b00010010 //Temperature Alert 3 Limit
#define TEMP_ALERT4_LIMIT_REG          0b00010011  //Temperature Alert 4 Limit

#define VERSION_ID_REG                 0b00100000 //Device ID/Revision

//Thermocouple Sensor Configuration register
//set thermo-type
#define THER_TYPE_K                    0
#define THER_TYPE_J                    16
#define THER_TYPE_T                    32
#define THER_TYPE_N                    48
#define THER_TYPE_S                    64
#define THER_TYPE_E                    80
#define THER_TYPE_B                    96
#define THER_TYPE_R                    112

//device configuration register see table 5-1 page 23 data sheet

//cold junction resolution
#define COLD_JUNC_RESOLUTION_0_625     0<<7
#define COLD_JUNC_RESOLUTION_0_25      1<<7

//ADC Resolution
#define ADC_18BIT_RESOLUTION           0<<5
#define ADC_16BIT_RESOLUTION           1<<5
#define ADC_14BIT_RESOLUTION           2<<5
#define ADC_12BIT_RESOLUTION           3<<5

//Burst Mode
#define BURST_1_SAMPLE                 0<<2
#define BURST_2_SAMPLE                 1<<2
#define BURST_4_SAMPLE                 2<<2
#define BURST_8_SAMPLE                 3<<2
#define BURST_16_SAMPLE                4<<2
#define BURST_32_SAMPLE                5<<2
#define BURST_64_SAMPLE                6<<2
#define BURST_128_SAMPLE               7<<2

//shutdown modes
#define NORMAL_OPERATION               0
#define SHUTDOWN_MODE                  1
#define BURST_MODE                     2

//ALERT 1, 2, 3 AND 4 Configuration Register
#define ACTIVE_LOW                     0<<2
#define ACTIVE_HIGH                    1<<2

#define INT_MODE                       1<<1
#define COMPARE_MODE                   0<<1

//Status Register
#define UPDATE_FLAG                    1<<6

#define FILT_OFF                       0
#define FILT_MIN                       1
#define FILT_MID                       4
#define FILT_MAX                       7

//misc

//used for numbering of alert
#define ALERT_NUN_1                     1
#define ALERT_NUN_2                     2
#define ALERT_NUN_3                     3
#define ALERT_NUN_4                     4

//I2C address. Distinct for each I2C device, change here
#define DEFAULT_IIC_ADDR                0X60 //set as 0x60 as default after checking

//resolution of temperature?, TODO not finish here
#define RESOLUTION_0_5_DEGREE           0
#define RESOLUTION_0_25_DEGREE          0X01
#define RESOLUTION_0_125_DEGREE         0X02
#define RESOLUTION_0_0625_DEGREE        0X03

#include<AP_HAL/AP_HAL.h>
#include<AP_HAL/Device.h>
#include<AP_HAL/I2CDevice.h>
#include<AP_HAL/utility/OwnPtr.h>
#include<GCS_MAVLink/GCS.h>
#include<stdint.h>
#include<AP_Common/missing/byteswap.h>

class AP_MCP9600{
private:
    float temperature;
    uint16_t address;
    uint32_t lastReadValue; //the value of last read register
    unsigned char thermoType;//K,J,T,N,S,E,B,R thermocouple, default as K
    uint32_t time;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> device;
    bool readByte(uint8_t reg);//ok
    bool readDoubleByte(uint8_t reg);//ok
    bool readTripleByte(uint8_t reg);//ok
    
    void recordTime();//ok

public:
    static uint8_t readFlag;
    AP_MCP9600();
    AP_MCP9600(uint16_t, char);//no default, to avoid overload ambigous class error
    virtual ~AP_MCP9600();
    bool writeByte(uint8_t reg,uint8_t val);//ok
    bool init();
    void setAddress(uint8_t add);/*consider delete this, because there is not have 
                            * a real function to change the device address in
                            * the device class
                            */
    //For registers of temperature and ADC
    bool readHotJunction();//ok
    bool readColdJunction();//ok
    bool readJunctionsDifferent();//ok
    bool readADC();//ok
    //Status Register
    bool readStatus();//ok
    bool checkDataUpdate();//ok
    
    //For Thermocouple Sensor Configuration Reg
    bool setThermoConfig(uint8_t setValue);//ok
    bool readThermoConfig();//ok
    bool setThermoType(char type);//ok
    bool setFilterCofficient(uint8_t setValue);//ok
    //For Device configuration register
    bool setDeviceConfig(uint8_t set_byte);//ok
    bool readDeviveConfig();//ok
    bool setColdJunctionResolution(uint8_t setValue);//ok
    bool setADCMeasureResolution(uint8_t setValue);//ok
    bool setBurstModeSamp(uint8_t setValue);//ok
    bool setShutdownMode(uint8_t setValue);//ok
    //For Alert limit 1-4 registers
    bool setAlertLimit(uint8_t alert_num,uint16_t value);//not implemented
    //For Alert 1-4 hysteresis registers
    bool setAlertHysteresis(uint8_t alert_num,uint16_t value);//not implemented
    //For Alert 1-4 configuration registers
    bool setAlertConfig(uint8_t alertChannel,uint8_t setValue);//ok
    bool readAlertConfig(uint8_t alertChannel);//ok
    bool clearInterruptFlag(uint8_t alertChannel);//ok
    bool setAlertForThOrTc(uint8_t alertChannel,uint8_t setValue);
    bool setAlertLimitDirection(unsigned char alert_num,unsigned char set_byte);
    bool setAlertBit(unsigned char alert_num,unsigned char set_byte);
    bool setAlertModeBit(unsigned char alert_num,unsigned char set_byte);
    bool setAlertEnable(unsigned char alert_num,unsigned char set_byte);
    //other
    uint16_t covertTempToRegForm(float temp);//ok
    float getTemperature();
    uint32_t getLastReg();
    void update();//timer of this, 20Hz ok
    //TODO implement function MAVLINK GSC to send the data
    void dataSend();
};


