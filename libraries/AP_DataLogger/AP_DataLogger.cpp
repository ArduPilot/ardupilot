/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/* 
    Data Logger - support for logging none flight related sensores to dataflash
    Author: Peter Hall - commissioned by ISS Aerospace
*/

#include "AP_DataLogger.h"

const AP_Param::GroupInfo AP_DataLogger::var_info[] = {

    // @Param: ENABLED
    // @DisplayName: Data Logger enabled
    // @Description: Data Logger enabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLED", 1, AP_DataLogger, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Data Logger type
    // @Description: Data Logger type
    // @Values: 0:None,1:Serial ascii
    // @User: Standard
    // @RebootRequired: True
    // @Bitmask: 0:Serial ascii
    AP_GROUPINFO("TYPE", 2, AP_DataLogger, _type, 0),
    AP_GROUPEND
};

// Constructor
AP_DataLogger::AP_DataLogger()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton) {
        AP_HAL::panic("Too many DataLoggers");
    }
#endif
    _singleton = this;
}

/*
 * Get the AP_DataLogger singleton
 */
AP_DataLogger *AP_DataLogger::get_singleton()
{
    return _singleton;
}

// Return true if data logger is enabled
bool AP_DataLogger::enabled() const
{
    return _enabled != 0;
}

// Initialize the datalogger object and prepare it for use
void AP_DataLogger::init()
{
    if (_enabled == 0) {
        return;
    }

    if ((_type & _DATALOGGER_ASCII) !=0) {
        _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Data_log, 0);

        if (_uart != nullptr) {
            _uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Data_log, 0));
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Data Logger: no serial port found");
        }
    }

    _logger = AP_Logger::get_singleton();
}

// Update datalogger, expected to be called at 10hz
void AP_DataLogger::update()
{
    if (_enabled == 0) {
        clear();
        return;
    }

    if (_logger == nullptr) {
        clear();
        return;
    }
    if (!_logger->should_log(0xFFFFFFFF)) { // this is all ones 32 bit bitmask, ie allways log unless disabled
        clear();
        return;
    }

    if ((_type & _DATALOGGER_ASCII) !=0) {
        update_ascii();
    } else {
        clear();
    }
}

// Write ascii data to data flash logs, delimited by \n and/or \r
void AP_DataLogger::update_ascii()
{
    if (_uart == nullptr) {
        clear();
        return;
    }

   // read any available data
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        char c = _uart->read();
        if (decode(c)) {
            // Line ending found or buffer is full. Log latest line
            _logger->Write("DTA0", "TimeUS,TimeStampUS,asciiData",
                        "ss-", "FF?", "QQZ",
                        AP_HAL::micros64(),
                        _time_stamp,
                        _term);

            // print debug
            //gcs().send_text(MAV_SEVERITY_INFO, "Data Logger: %s",_term);
            clear();
        }
    }
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded or if the buffer is full
bool AP_DataLogger::decode(char c)
{

    if (c == '\r' || c == '\n') {
        // we have got to the end of this string
        _overflow = false;
        if (_num_chars > 0) {
            return true;
        } else {
            return false;
        }
    }
    // otherwise we should add the character to the string to be logged

    if (_num_chars == 0 && !_overflow) {
        // record the timestamp of the first char in the string
        _time_stamp = AP_HAL::micros64();
    }

    _term[_num_chars] = c;
    _num_chars++;

    // if were going to overflow the buffer then start a new line
    if (_num_chars == ASCII_BUFFER) {
        _overflow = true;
        return true;
    }

    return false;
}

void AP_DataLogger::clear()
{
    // buffer is already clear
    if (_num_chars == 0) {
        return;
    }

    // clear the buffer
    for (uint8_t i=0; i<ASCII_BUFFER; i++) {
        _term[i] = '\0';
    }
    _num_chars = 0;
}

AP_DataLogger *AP_DataLogger::_singleton = nullptr;

namespace AP {
    AP_DataLogger *datalogger()
    {
        return AP_DataLogger::get_singleton();
    }
};
