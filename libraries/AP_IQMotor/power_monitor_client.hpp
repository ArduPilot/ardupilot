/*
  Copyright 2019 IQinetics Technologies, Inc support@iq-control.com

  This file is part of the IQ C++ API.

  This code is licensed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT for details)
*/

/*
  Name: power_monitor_client.hpp
  Last update: 3/7/2019 by Raphael Van Hoffelen
  Author: Matthew Piccoli
  Contributors: Raphael Van Hoffelen
*/

#ifndef POWER_MONITOR_CLIENT_HPP_
#define POWER_MONITOR_CLIENT_HPP_

#include "client_communication.hpp"

const uint8_t kTypePowerMonitor  =   69;

class PowerMonitorClient: public ClientAbstract{
  public:
    PowerMonitorClient(uint8_t obj_idn):
      ClientAbstract( kTypePowerMonitor, obj_idn),
      volts_(         kTypePowerMonitor, obj_idn, kSubVolts),
      amps_(          kTypePowerMonitor, obj_idn, kSubAmps),
      watts_(         kTypePowerMonitor, obj_idn, kSubWatts),
      joules_(        kTypePowerMonitor, obj_idn, kSubJoules),
      reset_joules_(  kTypePowerMonitor, obj_idn, kSubResetJoules),
      filter_fs_(     kTypePowerMonitor, obj_idn, kSubFilterFs),
      filter_fc_(     kTypePowerMonitor, obj_idn, kSubFilterFc),
      volts_raw_(     kTypePowerMonitor, obj_idn, kSubVoltsRaw),
      amps_raw_(      kTypePowerMonitor, obj_idn, kSubAmpsRaw),
      volts_gain_(    kTypePowerMonitor, obj_idn, kSubVoltsGain),
      amps_gain_(     kTypePowerMonitor, obj_idn, kSubAmpsGain),
      amps_bias_(     kTypePowerMonitor, obj_idn, kSubAmpsBias)
      {};

    // Client Entries
    // Control commands
    ClientEntry<float>    volts_;
    ClientEntry<float>    amps_;
    ClientEntry<float>    watts_;
    ClientEntry<float>    joules_;
    ClientEntryVoid       reset_joules_;
    ClientEntry<uint32_t> filter_fs_;
    ClientEntry<uint32_t> filter_fc_;
    ClientEntry<uint16_t> volts_raw_;
    ClientEntry<uint16_t> amps_raw_;
    ClientEntry<float>    volts_gain_;
    ClientEntry<float>    amps_gain_;
    ClientEntry<float>    amps_bias_;

    void ReadMsg(uint8_t* rx_data, uint8_t rx_length) override
    {
      static const uint8_t kEntryLength = kSubAmpsBias+1;
      ClientEntryAbstract* entry_array[kEntryLength] = {
        &volts_,        // 0
        &amps_,         // 1
        &watts_,        // 2
        &joules_,       // 3
        &reset_joules_, // 4
        &filter_fs_,    // 5
        &filter_fc_,    // 6
        &volts_raw_,    // 7
        &amps_raw_,     // 8
        &volts_gain_,   // 9
        &amps_gain_,    // 10
        &amps_bias_     // 11
      };

      ParseMsg(rx_data, rx_length, entry_array, kEntryLength);
    }

  private:
    static const uint8_t kSubVolts =      0;
    static const uint8_t kSubAmps =       1;
    static const uint8_t kSubWatts =      2;
    static const uint8_t kSubJoules =     3;
    static const uint8_t kSubResetJoules =4;
    static const uint8_t kSubFilterFs =   5;
    static const uint8_t kSubFilterFc =   6;
    static const uint8_t kSubVoltsRaw =   7;
    static const uint8_t kSubAmpsRaw =    8;
    static const uint8_t kSubVoltsGain =  9;
    static const uint8_t kSubAmpsGain =   10;
    static const uint8_t kSubAmpsBias =   11;
};

#endif /* POWER_MONITOR_CLIENT_HPP_ */
