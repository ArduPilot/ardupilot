#pragma once

/*
 *

 * DataSheet: https://www.ti.com/lit/ds/symlink/ina3221.pdf?ts=1597369254046
 */

// The INA3221takestwo measurementsfor eachchannel:one for shuntvoltageand one for bus voltage.Eachmeasurementcan be independentlyor sequentiallymeasured,basedon the modesetting(bits2-0 in theConfigurationregister).Whenthe INA3221is in normaloperatingmode(thatis, the MODEbits of theConfigurationregisterare set to 111),the devicecontinuouslyconvertsa shunt-voltagereadingfollowedby abus-voltagereading.Thisprocedureconvertsone channel,and thencontinuesto the shuntvoltagereadingofthe nextenabledchannel,followedby the bus-voltagereadingfor that channel,and so on, untilall enabledchannelshavebeenmeasured.The programmedConfigurationregistermodesettingappliesto all channels.Any channelsthat are not enabledare bypassedin the measurementsequence,regardlessof modesetting.


// 8.3.3SoftwareResetThe INA3321featuresa softwareresetthat reinitializesthe deviceand registersettingsto defaultpower-upvalueswithouthavingto cyclepowerto the device.Use bit 15 (RST)of the Configurationregisterto performasoftwarereset.SettingRSTreinitializesall registersand settingsto the defaultpowerstatewith the exceptionofthe power-validoutputstate.If a softwareresetis issued,the INA3221holdsthe outputof the PV pin until the power-validdetectionsequencecompletes.The Power-ValidUpperLimitand Power-ValidLowerlimit registersreturnto the defaultstatewhenthe softwareresethas beenissued.Therefore,any reprogrammedlimitregistersare reset,resultingin theoriginalpower-validthresholdsvalidatingthe power-validconditions.Thisarchitecturepreventsinterruptiontocircuitryconnectedto the powervalidoutputduringa softwareresetevent.

// TheINA3221has programmableconversiontimesfor boththe shunt-and bus-voltagemeasurements.Theselectableconversiontimesfor thesemeasurementsrangefrom140μs to 8.244ms.


#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_INA3221 : public AP_BattMonitor_Backend
{
public:
    /// Constructor
    AP_BattMonitor_INA3221(AP_BattMonitor &mon,
                           AP_BattMonitor::BattMonitor_State &mon_state,
                           AP_BattMonitor_Params &params,
                           AP_BattMonitor::Type type);

    void init() override;

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    bool has_current() const override {
        return true;
    }

private:

    bool init_at_address(uint8_t addr);

    HAL_Semaphore _sem;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    const AP_BattMonitor::Type _type;
    uint8_t address;

    bool read_register(uint8_t addr, uint16_t &ret);
    bool write_register(uint8_t addr, uint16_t val);

    void _timer(void);
};
