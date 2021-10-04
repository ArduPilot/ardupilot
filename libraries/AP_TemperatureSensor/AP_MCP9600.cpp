/*
 * MCP9600Grove.cpp
 *
 *  Created on: Sep 18, 2021
 *      Author: Thach
 */

#include "AP_MCP9600.h"
#include <AP_Math/AP_Math.h>
//#include <AP_HAL/utility/functor.h>


extern const AP_HAL::HAL &hal;
uint8_t AP_MCP9600::readFlag = 0;

AP_MCP9600::AP_MCP9600():address(DEFAULT_IIC_ADDR),temperature(0),
        lastReadValue(0),time(0){}

AP_MCP9600::AP_MCP9600(uint16_t add, char type){
    this->address=add;
    this->setThermoConfig(type);
    this->temperature=0;
    this->lastReadValue=0;
}

AP_MCP9600::~AP_MCP9600(){
}

/**
 * 
 * !!CAUTION!!
 * 
 * @return true if init successful, false otherwise
 */
bool AP_MCP9600::init(){
    bool result=true;
    
    uint8_t bus =1; //the bus number equals 1 by default
    device = hal.i2c_mgr->get_device(bus, this->address);

    if(!device){
        result = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Init is failed");
    } else {
        //WITH_SEMAPHORE(device->get_semaphore());
        this->device->set_speed(AP_HAL::Device::SPEED_HIGH);
        //update every 50ms, setting the timer and updated time,
        //device->register_periodic_callback((50*AP_USEC_PER_MSEC),FUNCTOR_BIND_MEMBER(&MCP9600::timer,void));/*This returns a nullpointer and hence make the connecting failed*/
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Init is finished");
    }
    return result;
}

/**
 * @brief to write a value to a register !!Caution!!
 * @param reg the written register
 * @param value the content to be written
 * @return true if writing is successful, false otherwise
 */
bool AP_MCP9600::writeByte(uint8_t reg, uint8_t val){
    bool result = false;
    /**
     * deactive read flag and connect it to the address
     * to active write condition
     * */
    readFlag = 0;
    uint8_t writeAddress = (DEFAULT_IIC_ADDR<<1)|readFlag;
    this->device->set_address(writeAddress);
    uint8_t data[2]={reg,val};
    result = this->device->transfer(data,sizeof(data),nullptr,0);
    return result;
}

/**
 * @Brief reading a 1 Byte-long register !!Caution!!
 * @param reg the to be read register
 * @param length the length of register in byte
 * @return
 */
bool AP_MCP9600::readByte(uint8_t reg){
    bool result = false;
    uint8_t temp1;
    /**
     * active read flag and connect it to the address
     * to active read condition
     * */
    readFlag = 1;
    uint8_t readAddress = (DEFAULT_IIC_ADDR<<1)|readFlag;
    this->device->set_address(readAddress);
    result = device->transfer(&reg,sizeof(reg),&temp1,sizeof(temp1));
    this->lastReadValue = temp1;
    return result;
}

/**
 * !!CAUTION!!
 * */
bool AP_MCP9600::readDoubleByte(uint8_t reg){
    bool result = false;
    uint8_t temp1[2];
    /**
     * active read flag and connect it to the address
     * to active read condition
     * */
    readFlag = 1;
    uint8_t readAddress = (DEFAULT_IIC_ADDR<<1)|readFlag;
    this->device->set_address(readAddress);
    result = device->transfer(&reg,sizeof(reg),temp1,sizeof(temp1));
    this->lastReadValue = (temp1[1]<<8)|temp1[0];
    return result;
}

/**
 * !!CAUTION!!
 * */
bool AP_MCP9600::readTripleByte(uint8_t reg){
    bool result = false;
    uint8_t temp1[3];
    /**
     * active read flag and connect it to the address
     * to active read condition
     * */
    readFlag = 1;
    uint8_t readAddress = (DEFAULT_IIC_ADDR<<1)|readFlag;
    this->device->set_address(readAddress);
    result = device->transfer(&reg,sizeof(reg),temp1,sizeof(temp1));
    this->lastReadValue = (temp1[2]<<16)|(temp1[1]<<8)|temp1[0];
    return result;
}

/**
 * 
 * //TODO to implement
 */
void AP_MCP9600::update(){
}

//TODO let it there not implement
void AP_MCP9600::recordTime(){
    bool test = this->readADC();//test if the sensor is active or not
    if(test){
        //if the test is ok, begin to record time
        this->time=AP_HAL::millis();
    }
}

/**
 * @brief return the temperature of measured point (hot junction is the 
 * measured point), the result will be stored in temperature-attribute of class
 * @param value
 * @return 
 */
bool AP_MCP9600::readHotJunction(){
    bool result=false;
    //result = this->readByte(HOT_JUNCTION_TEMP_REG,2);//see you anytime anywhere
    result = this->readDoubleByte(HOT_JUNCTION_TEMP_REG);
    if(result){
        if(lastReadValue&0x8000){
            this->temperature = ((float)(lastReadValue>>8))*16.0
                    +((float)(lastReadValue&0x00ff))/16.0-4096.0;
        }
        else{
            this->temperature = ((float)(lastReadValue>>8))*16.0
                    +((float)(lastReadValue&0x00ff))/16.0;
        }
    }
    return result;
}

/**
 * 
 * @param value
 * @return 
 */
bool AP_MCP9600::readColdJunction(){
    bool result = false;
    //result = this->readByte(COLD_JUNCTION_TEMP_REG,2);
    result = this->readDoubleByte(COLD_JUNCTION_TEMP_REG);
    if(result){
        if(lastReadValue&0x8000){
            this->temperature = ((float)(lastReadValue>>8))*16.0
                    +((float)(lastReadValue&0x00ff))/16.0-4096.0;
        }
        else{
            this->temperature = ((float)(lastReadValue>>8))*16.0
                    +((float)(lastReadValue&0x00ff))/16.0;
        }
    }
    return result;
}

/**
 * @brief read the temperature different between hot and cold junction
 * @param value used for external output
 * @return true if success, false otherwise
 */
bool AP_MCP9600::readJunctionsDifferent(){
    bool result = false;
    //result = this->readByte(JUNCTION_TEMP_DELTA_REG,2);
    result = this->readDoubleByte(JUNCTION_TEMP_DELTA_REG);
    if(result){
        if(lastReadValue&0x8000){
            this->temperature = ((float)(lastReadValue>>8))*16.0
                    +((float)(lastReadValue&0x00ff))/16.0-4096.0;
        }
        else{
            this->temperature = ((float)(lastReadValue>>8))*16.0
                    +((float)(lastReadValue&0x00ff))/16.0;
        }
    }
    return result;
}

/**
 * @brief the value of 24-bit ADC-register will be saved 
 * in class attribute lastReadValue
 * @return 
 */
bool AP_MCP9600::readADC(){
    bool result = false;
    //result = this->readByte(RAW_ADC_DATA_REG,3);
    result = this->readTripleByte(RAW_ADC_DATA_REG);
    return result;
}

/**
 * @brief
 * @return 
 */
bool AP_MCP9600::readStatus(){
    bool result = false;
    result = this->readByte(STAT_REG);
    return result;
}

/**
 * @brief used to set thermocouple type and filter coefficient (manually via 
 * bit mask), see p29 of data sheet
 * @param setValue the bit mask
 * @return true if success, false otherwise
 */
bool AP_MCP9600::setThermoConfig(uint8_t setValue){
    bool result = this->writeByte(THERM_SENS_CFG_REG,setValue);
    return result;
}

/**
 * @brief used to read the value of Sensor Configuration Register
 * @return true if reading is successful, false otherwise
 */
bool AP_MCP9600::readThermoConfig(){
    bool result = false;
    result = this->readByte(THERM_SENS_CFG_REG);
    return result;
}

/**
 * @brief set the thermo-type used the type as character
 * @param type the type of thermocouple (written as lowercase letter)
 * @return true if success, false otherwise
 */
bool AP_MCP9600::setThermoType(char type){
    bool result = false;
    uint8_t setByte=0;
    this->thermoType=type;
    switch(type){
    case 'k':
        setByte = 0<<4;
        break;
    case 'j':
        setByte = 1<<4;
        break;
    case 't':
        setByte = 2<<4;
        break;
    case 'n':
        setByte = 3<<4;
        break;
    case 's':
        setByte = 4<<4;
        break;
    case 'e':
        setByte = 5<<4;
        break;
    case 'b':
        setByte = 6<<4;
        break;
    case 'r':
        setByte = 7<<4;
        break;
    default:
        setByte = 0<<4;
        this->thermoType='k';
        break;
    };
    this->readThermoConfig();//save the current value of other bits
    setByte|=(((uint8_t)this->lastReadValue)&0x8f);/*clear the old value and 
                                                     * set the new for 
                                                     * thermocouple type*/
    result = this->writeByte(THERM_SENS_CFG_REG,setByte);
    return result;
}

/**
 * 
 * @param setValue
 * @return 
 */
bool AP_MCP9600::setFilterCofficient(uint8_t setValue){
    bool result=false;
    uint8_t setByte=0;
    this->readThermoConfig();//save the current value of other bits
    setByte = setValue | (((uint8_t)this->lastReadValue)&0xf8);/*clear the 
                                                                * old and 
                                                                * set the 
                                                                * new value*/
    result = this->writeByte(THERM_SENS_CFG_REG,setByte);
    return result;
}

/**
 * @brief manual setting the value of device configuration reg
 * @param setValue the set value
 * @return true if success, false otherwise
 */
bool AP_MCP9600::setDeviceConfig(uint8_t setValue){
    bool result=this->writeByte(DEVICE_CFG_REG,setValue);
    return result;
}

/**
 * 
 * @return 
 */
bool AP_MCP9600::readDeviveConfig(){
    bool result = false;
    //result = this->readByte(DEVICE_CFG_REG,2);
    result = this->readByte(DEVICE_CFG_REG);
    return result;
}

/**
 * 
 * @param setValue the value accordingly the data sheet page 30
 * @return true if success, false otherwise
 */
bool AP_MCP9600::setShutdownMode(uint8_t setValue){
    bool result = false; 
    uint8_t setByte=0xfc;
    setValue = setValue%4;//caution: for setValue = 3 it has no effect
    this->readByte(DEVICE_CFG_REG);//save the value of other bits
    setByte &= (uint8_t)(this->lastReadValue);//delete the old value
    setByte |= setValue;
    result = this->writeByte(DEVICE_CFG_REG,setByte);
    return result;
}

/**
 * @brief set the sample for burst mode,1/2/4/8/16/32/64/128 samples 
 * are available. Otherwise the default value (1 sample) will be chosen
 * @param setValue the number of sample 
 * @return true if success, otherwise false
 */
bool AP_MCP9600::setBurstModeSamp(uint8_t setValue){
    bool result = false;
    uint8_t resolution=0; //the value of resolution
    uint8_t setByte=~((1<<2)|(1<<3)|(1<<4));//make the mask to reset bit 2.-4.
    this->readByte(DEVICE_CFG_REG);//save the value of other bits
    setByte &= (uint8_t)(this->lastReadValue);//delete the old value of bit 2-4
    switch(setValue){
    case 1:
        resolution=0;
        break;
    case 2:
        resolution=(1<<2);
        break;
    case 4:
        resolution=(2<<2);
        break;
    case 8:
        resolution=(3<<2);
        break;
    case 16:
        resolution=(4<<2);
        break;
    case 32:
        resolution=(5<<2);
        break;
    case 64:
        resolution=(6<<2);
        break;
    case 128:
        resolution=(7<<2);
        break;
    default:
        break;
    };
    setByte|=resolution;
    result = this->writeByte(DEVICE_CFG_REG,setByte);
    return result;
}

/**
 * @brief setting the ADC measure resolution,18/16/14/12-bit resolutions 
 * available. if another value is written, the value of 18 bit-resolution will 
 * be set as default
 * @param setValue the desired resolution  
 * @return 
 */
bool AP_MCP9600::setADCMeasureResolution(uint8_t setValue){
    bool result = false;
    uint8_t setByte=0;
    this->readByte(DEVICE_CFG_REG);//save the value of other bits
    setByte = this->lastReadValue&0x9f;//delete the old value
    switch(setValue){
    case 18:
        setByte |= 0;
        break;
    case 16:
        setByte |= 1<<5;
        break;
    case 14:
        setByte |= 2<<5;
        break;
    case 12:
        setByte |= 3<<5;
        break;
    default:
        break;
    };
    result = this->writeByte(DEVICE_CFG_REG,setByte);
    return result;
}

/**
 * @brief setting the resolution for Cold Junction
 * @param setValue 0 for 0.0625Grad C and 128 for 0.25Grad C
 * @return true if success, false otherwise
 */
bool AP_MCP9600::setColdJunctionResolution(uint8_t setValue){
    bool result=false;
    uint8_t setByte = (uint8_t)(~(1<<7));
    this->readByte(DEVICE_CFG_REG);//save the value of other bits
    //delete the old value and setting the new
    setByte &= this->lastReadValue;
    setByte |=setValue;
    result = this->writeByte(DEVICE_CFG_REG,setByte);
    return result;
}

//not implemented
bool AP_MCP9600::setAlertLimit(uint8_t alert_num,uint16_t value){
    return false;
}

//not implemented
bool AP_MCP9600::setAlertHysteresis(uint8_t alert_num,uint16_t value){
    return false;
}

/**
 * @brief manual setting the value for Alert configuration registers
 * @param alertChannel the channel to be alerted, 1-4 is available, periodic
 * @param setValue the to be set value
 * @return 
 */
bool AP_MCP9600::setAlertConfig(uint8_t alertChannel, uint8_t setValue){
    bool result = false;
    alertChannel = alertChannel%4;
    switch(alertChannel){
    case 1:
        result = this->writeByte(ALERT1_CFG_REG,setValue);
        break;
    case 2:
        result = this->writeByte(ALERT2_CFG_REG,setValue);
        break;
    case 3:
        result = this->writeByte(ALERT3_CFG_REG,setValue);
        break;
    case 0:
        result = this->writeByte(ALERT4_CFG_REG,setValue);
        break;
    default:
        break;
    }
    return result;
}

/**
 * 
 * @param alertChannel
 * @return 
 */
bool AP_MCP9600::readAlertConfig(uint8_t alertChannel){
    alertChannel = alertChannel%4;
    bool result = false;
    switch(alertChannel){
    case 1:
        result = this->readByte(ALERT1_CFG_REG);
        break;
    case 2:
        result = this->readByte(ALERT2_CFG_REG);
        break;
    case 3:
        result = this->readByte(ALERT3_CFG_REG);
        break;
    case 0:
        result = this->readByte(ALERT4_CFG_REG);
        break;
    default:
        break;
    }
    return result;
}

/**
 * @brief used to clear the interrupt-flag in register
 * @param alertChannel
 * @return 
 */
bool AP_MCP9600::clearInterruptFlag(uint8_t alertChannel){
    alertChannel = alertChannel%4;
    uint8_t clearMask =(1<<7);
    uint8_t addressOfChannel; /*TODO if the value is false, try to insert the 
                               * addresses direct to function readByte and 
                               * writeByte instead */
    bool result = false;
    switch(alertChannel){
    case 1:
        addressOfChannel = ALERT1_CFG_REG;
        break;
    case 2:
        addressOfChannel = ALERT2_CFG_REG;
        break;
    case 3:
        addressOfChannel = ALERT3_CFG_REG;
        break;
    case 0:
        addressOfChannel = ALERT4_CFG_REG;
        break;
    default:
        break;
    }
    this->readByte(addressOfChannel); //stored the values of remaining bits
    clearMask |= this->lastReadValue;
    result = this->writeByte(addressOfChannel,clearMask);
    return result;
}

//not implemented
bool AP_MCP9600::setAlertForThOrTc(uint8_t alertChannel, uint8_t setValue){
    return false;
}

//not implemented
bool AP_MCP9600::setAlertLimitDirection(unsigned char alert_num,unsigned char set_byte){
    return false;
}

//not implemented
bool AP_MCP9600::setAlertBit(unsigned char alert_num,unsigned char set_byte){
    return false;
}

//not implemented
bool AP_MCP9600::setAlertModeBit(unsigned char alert_num,unsigned char set_byte){
    return false;
}

//not implemented
bool AP_MCP9600::setAlertEnable(unsigned char alert_num,unsigned char set_byte){
    return false;
}

/**
 * convert an temperature to register-form in this Sensor
 * @param temp to converting temperature
 * @return the converted temperature
 */
uint16_t AP_MCP9600::covertTempToRegForm(float temp){
    uint8_t negative=0;
    if(temp<0){ 
        negative=1;
        temp = -temp;
    }
    //temp=abs(temp);
    uint16_t dest_temp=0;
    uint8_t temp_H=0,temp_L=0;
    uint16_t interger=(uint16_t)temp;
    float decimal=temp-interger;
    temp_H=interger/16;
    temp_L|=(interger%16)<<4;
    temp_L|=(uint8_t)(decimal/0.25)<<2;
    if(negative) temp_H|=0x80;
    dest_temp=(uint16_t)temp_H<<8|temp_L;
    return dest_temp;
}

/**
 * @brief check if byte 7 (Flag TH Updated) is updated or not
 * @return true if data is update, false otherwise
 */
bool AP_MCP9600::checkDataUpdate(){
    bool result = false;
    this->readStatus();
    if(lastReadValue&0x40){
        result = true;
    }
    return result;
}

/**
 * 
 * @return 
 */
float AP_MCP9600::getTemperature(){
    return this->temperature;
}

uint32_t AP_MCP9600::getLastReg(){
    return this->lastReadValue;
}