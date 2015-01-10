/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "DataFlash.h"
#include <stdlib.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_Baro.h>
#include <AP_AHRS.h>

extern const AP_HAL::HAL& hal;


void DataFlash_Class::Init(const struct LogStructure *structure, uint8_t num_types)
{
    _num_types = num_types;
    _structures = structure;
    _writes_enabled = true;
}


// This function determines the number of whole or partial log files in the DataFlash
// Wholly overwritten files are (of course) lost.
uint16_t DataFlash_Block::get_num_logs(void)
{
    uint16_t lastpage;
    uint16_t last;
    uint16_t first;

    if (find_last_page() == 1) {
        return 0;
    }

    StartRead(1);

    if (GetFileNumber() == 0xFFFF) {
        return 0;
    }

    lastpage = find_last_page();
    StartRead(lastpage);
    last = GetFileNumber();
    StartRead(lastpage + 2);
    first = GetFileNumber();
    if(first > last) {
        StartRead(1);
        first = GetFileNumber();
    }

    if (last == first) {
        return 1;
    }

    return (last - first + 1);
}


// This function starts a new log file in the DataFlash
uint16_t DataFlash_Block::start_new_log(void)
{
    uint16_t last_page = find_last_page();

    StartRead(last_page);
    //Serial.print("last page: ");	Serial.println(last_page);
    //Serial.print("file #: ");	Serial.println(GetFileNumber());
    //Serial.print("file page: ");	Serial.println(GetFilePage());

    if(find_last_log() == 0 || GetFileNumber() == 0xFFFF) {
        SetFileNumber(1);
        StartWrite(1);
        //Serial.println("start log from 0");
        log_write_started = true;
        return 1;
    }

    uint16_t new_log_num;

    // Check for log of length 1 page and suppress
    if(GetFilePage() <= 1) {
        new_log_num = GetFileNumber();
        // Last log too short, reuse its number
        // and overwrite it
        SetFileNumber(new_log_num);
        StartWrite(last_page);
    } else {
        new_log_num = GetFileNumber()+1;
        if (last_page == 0xFFFF) {
            last_page=0;
        }
        SetFileNumber(new_log_num);
        StartWrite(last_page + 1);
    }
    log_write_started = true;
    return new_log_num;
}

// This function finds the first and last pages of a log file
// The first page may be greater than the last page if the DataFlash has been filled and partially overwritten.
void DataFlash_Block::get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page)
{
    uint16_t num = get_num_logs();
    uint16_t look;

    if (df_BufferIdx != 0) {
        FinishWrite();
        hal.scheduler->delay(100);
    }

    if(num == 1)
    {
        StartRead(df_NumPages);
        if (GetFileNumber() == 0xFFFF)
        {
            start_page = 1;
            end_page = find_last_page_of_log((uint16_t)log_num);
        } else {
            end_page = find_last_page_of_log((uint16_t)log_num);
            start_page = end_page + 1;
        }

    } else {
        if(log_num==1) {
            StartRead(df_NumPages);
            if(GetFileNumber() == 0xFFFF) {
                start_page = 1;
            } else {
                start_page = find_last_page() + 1;
            }
        } else {
            if(log_num == find_last_log() - num + 1) {
                start_page = find_last_page() + 1;
            } else {
                look = log_num-1;
                do {
                    start_page = find_last_page_of_log(look) + 1;
                    look--;
                } while (start_page <= 0 && look >=1);
            }
        }
    }
    if (start_page == df_NumPages+1 || start_page == 0) {
        start_page = 1;
    }
    end_page = find_last_page_of_log(log_num);
    if (end_page == 0) {
        end_page = start_page;
    }
}

// find log size and time
void DataFlash_Block::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc)
{
    uint16_t start, end;
    get_log_boundaries(log_num, start, end);
    if (end >= start) {
        size = (end + 1 - start) * (uint32_t)df_PageSize;
    } else {
        size = (df_NumPages + end - start) * (uint32_t)df_PageSize;
    }
    time_utc = 0;
}

bool DataFlash_Block::check_wrapped(void)
{
    StartRead(df_NumPages);
    if(GetFileNumber() == 0xFFFF)
        return 0;
    else
        return 1;
}


// This funciton finds the last log number
uint16_t DataFlash_Block::find_last_log(void)
{
    uint16_t last_page = find_last_page();
    StartRead(last_page);
    return GetFileNumber();
}

// This function finds the last page of the last file
uint16_t DataFlash_Block::find_last_page(void)
{
    uint16_t look;
    uint16_t bottom = 1;
    uint16_t top = df_NumPages;
    uint32_t look_hash;
    uint32_t bottom_hash;
    uint32_t top_hash;

    StartRead(bottom);
    bottom_hash = ((int32_t)GetFileNumber()<<16) | GetFilePage();

    while(top-bottom > 1) {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (int32_t)GetFileNumber()<<16 | GetFilePage();
        if (look_hash >= 0xFFFF0000) look_hash = 0;

        if(look_hash < bottom_hash) {
            // move down
            top = look;
        } else {
            // move up
            bottom = look;
            bottom_hash = look_hash;
        }
    }

    StartRead(top);
    top_hash = ((int32_t)GetFileNumber()<<16) | GetFilePage();
    if (top_hash >= 0xFFFF0000) {
        top_hash = 0;
    }
    if (top_hash > bottom_hash) {
        return top;
    }

    return bottom;
}

// This function finds the last page of a particular log file
uint16_t DataFlash_Block::find_last_page_of_log(uint16_t log_number)
{
    uint16_t look;
    uint16_t bottom;
    uint16_t top;
    uint32_t look_hash;
    uint32_t check_hash;

    if(check_wrapped())
    {
        StartRead(1);
        bottom = GetFileNumber();
        if (bottom > log_number)
        {
            bottom = find_last_page();
            top = df_NumPages;
        } else {
            bottom = 1;
            top = find_last_page();
        }
    } else {
        bottom = 1;
        top = find_last_page();
    }

    check_hash = (int32_t)log_number<<16 | 0xFFFF;

    while(top-bottom > 1)
    {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (int32_t)GetFileNumber()<<16 | GetFilePage();
        if (look_hash >= 0xFFFF0000) look_hash = 0;

        if(look_hash > check_hash) {
            // move down
            top = look;
        } else {
            // move up
            bottom = look;
        }
    }

    StartRead(top);
    if (GetFileNumber() == log_number) return top;

    StartRead(bottom);
    if (GetFileNumber() == log_number) return bottom;

    return -1;
}

#define PGM_UINT8(addr) pgm_read_byte((const prog_char *)addr)

#ifndef DATAFLASH_NO_CLI
/*
  read and print a log entry using the format strings from the given structure
 */
void DataFlash_Class::_print_log_entry(uint8_t msg_type,
                                       void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                                       AP_HAL::BetterStream *port)
{
    uint8_t i;
    for (i=0; i<_num_types; i++) {
        if (msg_type == PGM_UINT8(&_structures[i].msg_type)) {
            break;
        }
    }
    if (i == _num_types) {
        port->printf_P(PSTR("UNKN, %u\n"), (unsigned)msg_type);
        return;
    }
    uint8_t msg_len = PGM_UINT8(&_structures[i].msg_len) - 3;
    uint8_t pkt[msg_len];
    ReadBlock(pkt, msg_len);
    port->printf_P(PSTR("%S, "), _structures[i].name);
    for (uint8_t ofs=0, fmt_ofs=0; ofs<msg_len; fmt_ofs++) {
        char fmt = PGM_UINT8(&_structures[i].format[fmt_ofs]);
        switch (fmt) {
        case 'b': {
            port->printf_P(PSTR("%d"), (int)pkt[ofs]);
            ofs += 1;
            break;
        }
        case 'B': {
            port->printf_P(PSTR("%u"), (unsigned)pkt[ofs]);
            ofs += 1;
            break;
        }
        case 'h': {
            int16_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf_P(PSTR("%d"), (int)v);
            ofs += sizeof(v);
            break;
        }
        case 'H': {
            uint16_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf_P(PSTR("%u"), (unsigned)v);
            ofs += sizeof(v);
            break;
        }
        case 'i': {
            int32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf_P(PSTR("%ld"), (long)v);
            ofs += sizeof(v);
            break;
        }
        case 'I': {
            uint32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf_P(PSTR("%lu"), (unsigned long)v);
            ofs += sizeof(v);
            break;
        }
        case 'f': {
            float v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf_P(PSTR("%f"), v);
            ofs += sizeof(v);
            break;
        }
        case 'c': {
            int16_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf_P(PSTR("%.2f"), 0.01f*v);
            ofs += sizeof(v);
            break;
        }
        case 'C': {
            uint16_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf_P(PSTR("%.2f"), 0.01f*v);
            ofs += sizeof(v);
            break;
        }
        case 'e': {
            int32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf_P(PSTR("%.2f"), 0.01f*v);
            ofs += sizeof(v);
            break;
        }
        case 'E': {
            uint32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf_P(PSTR("%.2f"), 0.01f*v);
            ofs += sizeof(v);
            break;
        }
        case 'L': {
            int32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            print_latlon(port, v);
            ofs += sizeof(v);
            break;
        }
        case 'n': {
            char v[5];
            memcpy(&v, &pkt[ofs], sizeof(v));
            v[sizeof(v)-1] = 0;
            port->printf_P(PSTR("%s"), v);
            ofs += sizeof(v)-1;
            break;
        }
        case 'N': {
            char v[17];
            memcpy(&v, &pkt[ofs], sizeof(v));
            v[sizeof(v)-1] = 0;
            port->printf_P(PSTR("%s"), v);
            ofs += sizeof(v)-1;
            break;
        }
        case 'Z': {
            char v[65];
            memcpy(&v, &pkt[ofs], sizeof(v));
            v[sizeof(v)-1] = 0;
            port->printf_P(PSTR("%s"), v);
            ofs += sizeof(v)-1;
            break;
        }
        case 'M': {
            print_mode(port, pkt[ofs]);
            ofs += 1;
            break;
        }
        default:
            ofs = msg_len;
            break;
        }
        if (ofs < msg_len) {
            port->printf_P(PSTR(", "));
        }
    }
    port->println();
}


/*
  print FMT specifiers for log dumps where we have wrapped in the
  dataflash and so have no formats. This assumes the log being dumped
  using the same log formats as the current formats, but it is better
  than falling back to old defaults in the GCS
 */
void DataFlash_Block::_print_log_formats(AP_HAL::BetterStream *port)
{
    for (uint8_t i=0; i<_num_types; i++) {
        const struct LogStructure *s = &_structures[i];
        port->printf_P(PSTR("FMT, %u, %u, %S, %S, %S\n"),
                       (unsigned)PGM_UINT8(&s->msg_type),
                       (unsigned)PGM_UINT8(&s->msg_len),
                       s->name, s->format, s->labels);
    }
}

/*
  Read the log and print it on port
*/
void DataFlash_Block::LogReadProcess(uint16_t log_num,
                                     uint16_t start_page, uint16_t end_page,
                                     void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                                     AP_HAL::BetterStream *port)
{
    uint8_t log_step = 0;
    uint16_t page = start_page;
    bool first_entry = true;

    if (df_BufferIdx != 0) {
        FinishWrite();
        hal.scheduler->delay(100);
    }

    StartRead(start_page);

	while (true) {
		uint8_t data;
        ReadBlock(&data, 1);

		// This is a state machine to read the packets
		switch(log_step) {
			case 0:
				if (data == HEAD_BYTE1) {
					log_step++;
                }
				break;

			case 1:
				if (data == HEAD_BYTE2) {
					log_step++;
                } else {
					log_step = 0;
				}
				break;

			case 2:
				log_step = 0;
                if (first_entry && data != LOG_FORMAT_MSG) {
                    _print_log_formats(port);
                }
                first_entry = false;
                _print_log_entry(data, print_mode, port);
                break;
		}
        uint16_t new_page = GetPage();
        if (new_page != page) {
            if (new_page == end_page+1 || new_page == start_page) {
                return;
            }
            page = new_page;
        }
	}
}

/*
  dump header information from all log pages
 */
void DataFlash_Block::DumpPageInfo(AP_HAL::BetterStream *port)
{
    for (uint16_t count=1; count<=df_NumPages; count++) {
        StartRead(count);
        port->printf_P(PSTR("DF page, log file #, log page: %u,\t"), (unsigned)count);
        port->printf_P(PSTR("%u,\t"), (unsigned)GetFileNumber());
        port->printf_P(PSTR("%u\n"), (unsigned)GetFilePage());
    }
}

/*
  show information about the device
 */
void DataFlash_Block::ShowDeviceInfo(AP_HAL::BetterStream *port)
{
    if (!CardInserted()) {
        port->println_P(PSTR("No dataflash inserted"));
        return;
    }
    ReadManufacturerID();
    port->printf_P(PSTR("Manufacturer: 0x%02x   Device: 0x%04x\n"),
                    (unsigned)df_manufacturer,
                    (unsigned)df_device);
    port->printf_P(PSTR("NumPages: %u  PageSize: %u\n"),
                   (unsigned)df_NumPages+1,
                   (unsigned)df_PageSize);
}

/*
  list available log numbers
 */
void DataFlash_Block::ListAvailableLogs(AP_HAL::BetterStream *port)
{
    uint16_t num_logs = get_num_logs();
    int16_t last_log_num = find_last_log();
    uint16_t log_start = 0;
    uint16_t log_end = 0;

    if (num_logs == 0) {
        port->printf_P(PSTR("\nNo logs\n\n"));
        return;
    }
    port->printf_P(PSTR("\n%u logs\n"), (unsigned)num_logs);

    for (uint16_t i=num_logs; i>=1; i--) {
        uint16_t last_log_start = log_start, last_log_end = log_end;
        uint16_t temp = last_log_num - i + 1;
        get_log_boundaries(temp, log_start, log_end);
        port->printf_P(PSTR("Log %u,    start %u,   end %u\n"),
                       (unsigned)temp,
                       (unsigned)log_start,
                       (unsigned)log_end);
        if (last_log_start == log_start && last_log_end == log_end) {
            // we are printing bogus logs
            break;
        }
    }
    port->println();
}
#endif // DATAFLASH_NO_CLI

// This function starts a new log file in the DataFlash, and writes
// the format of supported messages in the log, plus all parameters
uint16_t DataFlash_Class::StartNewLog(void)
{
    uint16_t ret;
    ret = start_new_log();
    if (ret == 0xFFFF) {
        // don't write out parameters if we fail to open the log
        return ret;
    }
    // write log formats so the log is self-describing
    for (uint8_t i=0; i<_num_types; i++) {
        Log_Write_Format(&_structures[i]);
        // avoid corrupting the APM1/APM2 dataflash by writing too fast
        hal.scheduler->delay(10);
    }

    // and all current parameters
    Log_Write_Parameters();
    return ret;
}

// add new logging formats to the log. Used by libraries that want to
// add their own log messages
void DataFlash_Class::AddLogFormats(const struct LogStructure *structures, uint8_t num_types)
{
    // write new log formats
    for (uint8_t i=0; i<num_types; i++) {
        Log_Write_Format(&structures[i]);
    }
}

/*
  write a structure format to the log
 */
void DataFlash_Class::Log_Fill_Format(const struct LogStructure *s, struct log_Format &pkt)
{
    memset(&pkt, 0, sizeof(pkt));
    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_FORMAT_MSG;
    pkt.type = PGM_UINT8(&s->msg_type);
    pkt.length = PGM_UINT8(&s->msg_len);
    strncpy_P(pkt.name, s->name, sizeof(pkt.name));
    strncpy_P(pkt.format, s->format, sizeof(pkt.format));
    strncpy_P(pkt.labels, s->labels, sizeof(pkt.labels));
}

/*
  write a structure format to the log
 */
void DataFlash_Class::Log_Write_Format(const struct LogStructure *s)
{
    struct log_Format pkt;
    Log_Fill_Format(s, pkt);
    WriteBlock(&pkt, sizeof(pkt));
}


/*
  write a parameter to the log
 */
void DataFlash_Class::Log_Write_Parameter(const char *name, float value)
{
    struct log_Parameter pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMETER_MSG),
        name  : {},
        value : value
    };
    strncpy(pkt.name, name, sizeof(pkt.name));
    WriteBlock(&pkt, sizeof(pkt));
}

/*
  write a parameter to the log
 */
void DataFlash_Class::Log_Write_Parameter(const AP_Param *ap,
                                          const AP_Param::ParamToken &token,
                                          enum ap_var_type type)
{
    char name[16];
    ap->copy_name_token(token, &name[0], sizeof(name), true);
    Log_Write_Parameter(name, ap->cast_to_float(type));
}

/*
  write all parameters to the log - used when starting a new log so
  the log file has a full record of the parameters
 */
void DataFlash_Class::Log_Write_Parameters(void)
{
    AP_Param::ParamToken token;
    AP_Param *ap;
    enum ap_var_type type;

    for (ap=AP_Param::first(&token, &type);
         ap;
         ap=AP_Param::next_scalar(&token, &type)) {
        Log_Write_Parameter(ap, token, type);
        // slow down the parameter dump to prevent saturating
        // the dataflash write bandwidth
        hal.scheduler->delay(1);
    }
}



// Write an GPS packet
void DataFlash_Class::Log_Write_GPS(const AP_GPS &gps, uint8_t i, int32_t relative_alt)
{
    if (i == 0) {
        const struct Location &loc = gps.location(i);
        struct log_GPS pkt = {
            LOG_PACKET_HEADER_INIT(LOG_GPS_MSG),
            status        : (uint8_t)gps.status(i),
            gps_week_ms   : gps.time_week_ms(i),
            gps_week      : gps.time_week(i),
            num_sats      : gps.num_sats(i),
            hdop          : gps.get_hdop(i),
            latitude      : loc.lat,
            longitude     : loc.lng,
            rel_altitude  : relative_alt,
            altitude      : loc.alt,
            ground_speed  : (uint32_t)(gps.ground_speed(i) * 100),
            ground_course : gps.ground_course_cd(i),
            vel_z         : gps.velocity(i).z,
            apm_time      : hal.scheduler->millis()
        };
        WriteBlock(&pkt, sizeof(pkt));
    }
#if HAL_CPU_CLASS > HAL_CPU_CLASS_16
    if (i > 0) {
        const struct Location &loc2 = gps.location(i);
        struct log_GPS2 pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_GPS2_MSG),
            status        : (uint8_t)gps.status(i),
            gps_week_ms   : gps.time_week_ms(i),
            gps_week      : gps.time_week(i),
            num_sats      : gps.num_sats(i),
            hdop          : gps.get_hdop(i),
            latitude      : loc2.lat,
            longitude     : loc2.lng,
            altitude      : loc2.alt,
            ground_speed  : (uint32_t)(gps.ground_speed(i)*100),
            ground_course : gps.ground_course_cd(i),
            vel_z         : gps.velocity(i).z,
            apm_time      : hal.scheduler->millis(),
            dgps_numch    : 0,
            dgps_age      : 0
        };
        WriteBlock(&pkt2, sizeof(pkt2));
    }
#endif
}

// Write an RCIN packet
void DataFlash_Class::Log_Write_RCIN(void)
{
    uint32_t timestamp = hal.scheduler->millis();
    struct log_RCIN pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RCIN_MSG),
        timestamp     : timestamp,
        chan1         : hal.rcin->read(0),
        chan2         : hal.rcin->read(1),
        chan3         : hal.rcin->read(2),
        chan4         : hal.rcin->read(3),
        chan5         : hal.rcin->read(4),
        chan6         : hal.rcin->read(5),
        chan7         : hal.rcin->read(6),
        chan8         : hal.rcin->read(7),
        chan9         : hal.rcin->read(8),
        chan10        : hal.rcin->read(9),
        chan11        : hal.rcin->read(10),
        chan12        : hal.rcin->read(11),
        chan13        : hal.rcin->read(12),
        chan14        : hal.rcin->read(13)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an SERVO packet
void DataFlash_Class::Log_Write_RCOUT(void)
{
    struct log_RCOUT pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RCOUT_MSG),
        timestamp     : hal.scheduler->millis(),
        chan1         : hal.rcout->read(0),
        chan2         : hal.rcout->read(1),
        chan3         : hal.rcout->read(2),
        chan4         : hal.rcout->read(3),
        chan5         : hal.rcout->read(4),
        chan6         : hal.rcout->read(5),
        chan7         : hal.rcout->read(6),
        chan8         : hal.rcout->read(7),
        chan9         : hal.rcout->read(8),
        chan10        : hal.rcout->read(9),
        chan11        : hal.rcout->read(10),
        chan12        : hal.rcout->read(11)
    };
    WriteBlock(&pkt, sizeof(pkt));
    Log_Write_ESC();
}

// Write a BARO packet
void DataFlash_Class::Log_Write_Baro(AP_Baro &baro)
{
    uint32_t now = hal.scheduler->millis();
    struct log_BARO pkt = {
        LOG_PACKET_HEADER_INIT(LOG_BARO_MSG),
        timestamp     : now,
        altitude      : baro.get_altitude(0),
        pressure	  : baro.get_pressure(0),
        temperature   : (int16_t)(baro.get_temperature(0) * 100),
        climbrate     : baro.get_climb_rate()
    };
    WriteBlock(&pkt, sizeof(pkt));
#if BARO_MAX_INSTANCES > 1
    if (baro.num_instances() > 1 && baro.healthy(1)) {
        struct log_BARO pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_BAR2_MSG),
            timestamp     : now,
            altitude      : baro.get_altitude(1),
            pressure	  : baro.get_pressure(1),
            temperature   : (int16_t)(baro.get_temperature(1) * 100),
            climbrate     : baro.get_climb_rate()
        };
        WriteBlock(&pkt2, sizeof(pkt2));        
    }
#endif
}

// Write an raw accel/gyro data packet
void DataFlash_Class::Log_Write_IMU(const AP_InertialSensor &ins)
{
    uint32_t tstamp = hal.scheduler->millis();
    const Vector3f &gyro = ins.get_gyro(0);
    const Vector3f &accel = ins.get_accel(0);
    struct log_IMU pkt = {
        LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
        timestamp : tstamp,
        gyro_x  : gyro.x,
        gyro_y  : gyro.y,
        gyro_z  : gyro.z,
        accel_x : accel.x,
        accel_y : accel.y,
        accel_z : accel.z,
        gyro_error  : ins.get_gyro_error_count(0),
        accel_error : ins.get_accel_error_count(0)
    };
    WriteBlock(&pkt, sizeof(pkt));
    if (ins.get_gyro_count() < 2 && ins.get_accel_count() < 2) {
        return;
    }
#if INS_MAX_INSTANCES > 1
    const Vector3f &gyro2 = ins.get_gyro(1);
    const Vector3f &accel2 = ins.get_accel(1);
    struct log_IMU pkt2 = {
        LOG_PACKET_HEADER_INIT(LOG_IMU2_MSG),
        timestamp : tstamp,
        gyro_x  : gyro2.x,
        gyro_y  : gyro2.y,
        gyro_z  : gyro2.z,
        accel_x : accel2.x,
        accel_y : accel2.y,
        accel_z : accel2.z,
        gyro_error  : ins.get_gyro_error_count(1),
        accel_error : ins.get_accel_error_count(1)
    };
    WriteBlock(&pkt2, sizeof(pkt2));
    if (ins.get_gyro_count() < 3 && ins.get_accel_count() < 3) {
        return;
    }
    const Vector3f &gyro3 = ins.get_gyro(2);
    const Vector3f &accel3 = ins.get_accel(2);
    struct log_IMU pkt3 = {
        LOG_PACKET_HEADER_INIT(LOG_IMU3_MSG),
        timestamp : tstamp,
        gyro_x  : gyro3.x,
        gyro_y  : gyro3.y,
        gyro_z  : gyro3.z,
        accel_x : accel3.x,
        accel_y : accel3.y,
        accel_z : accel3.z,
        gyro_error  : ins.get_gyro_error_count(2),
        accel_error : ins.get_accel_error_count(3)
    };
    WriteBlock(&pkt3, sizeof(pkt3));
#endif
}

// Write a text message to the log
void DataFlash_Class::Log_Write_Message(const char *message)
{
    struct log_Message pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MESSAGE_MSG),
        msg  : {}
    };
    strncpy(pkt.msg, message, sizeof(pkt.msg));
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a text message to the log
void DataFlash_Class::Log_Write_Message_P(const prog_char_t *message)
{
    struct log_Message pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MESSAGE_MSG),
        msg  : {}
    };
    strncpy_P(pkt.msg, message, sizeof(pkt.msg));
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a POWR packet
void DataFlash_Class::Log_Write_Power(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    struct log_POWR pkt = {
        LOG_PACKET_HEADER_INIT(LOG_POWR_MSG),
        time_ms : hal.scheduler->millis(),
        Vcc     : (uint16_t)(hal.analogin->board_voltage() * 100),
        Vservo  : (uint16_t)(hal.analogin->servorail_voltage() * 100),
        flags   : hal.analogin->power_status_flags()
    };
    WriteBlock(&pkt, sizeof(pkt));
#endif
}

// Write an AHRS2 packet
void DataFlash_Class::Log_Write_AHRS2(AP_AHRS &ahrs)
{
    Vector3f euler;
    struct Location loc;
    if (!ahrs.get_secondary_attitude(euler) || !ahrs.get_secondary_position(loc)) {
        return;
    }
    struct log_AHRS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AHR2_MSG),
        time_ms : hal.scheduler->millis(),
        roll  : (int16_t)(degrees(euler.x)*100),
        pitch : (int16_t)(degrees(euler.y)*100),
        yaw   : (uint16_t)(wrap_360_cd(degrees(euler.z)*100)),
        alt   : loc.alt*1.0e-2f,
        lat   : loc.lat,
        lng   : loc.lng
    };
    WriteBlock(&pkt, sizeof(pkt));
}

#if AP_AHRS_NAVEKF_AVAILABLE
void DataFlash_Class::Log_Write_EKF(AP_AHRS_NavEKF &ahrs, bool optFlowEnabled)
{
	// Write first EKF packet
    Vector3f euler;
    Vector3f posNED;
    Vector3f velNED;
    Vector3f dAngBias;
    Vector3f dVelBias;
    Vector3f gyroBias;
    ahrs.get_NavEKF().getEulerAngles(euler);
    ahrs.get_NavEKF().getVelNED(velNED);
    ahrs.get_NavEKF().getPosNED(posNED);
    ahrs.get_NavEKF().getGyroBias(gyroBias);
    struct log_EKF1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_EKF1_MSG),
        time_ms : hal.scheduler->millis(),
        roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
        pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
        yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
        velN    : (float)(velNED.x), // velocity North (m/s)
        velE    : (float)(velNED.y), // velocity East (m/s)
        velD    : (float)(velNED.z), // velocity Down (m/s)
        posN    : (float)(posNED.x), // metres North
        posE    : (float)(posNED.y), // metres East
        posD    : (float)(posNED.z), // metres Down
        gyrX    : (int16_t)(100*degrees(gyroBias.x)), // cd/sec, displayed as deg/sec due to format string
        gyrY    : (int16_t)(100*degrees(gyroBias.y)), // cd/sec, displayed as deg/sec due to format string
        gyrZ    : (int16_t)(100*degrees(gyroBias.z)) // cd/sec, displayed as deg/sec due to format string
    };
    WriteBlock(&pkt, sizeof(pkt));

	// Write second EKF packet
    float ratio;
    float az1bias, az2bias;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    ahrs.get_NavEKF().getIMU1Weighting(ratio);
    ahrs.get_NavEKF().getAccelZBias(az1bias, az2bias);
    ahrs.get_NavEKF().getWind(wind);
    ahrs.get_NavEKF().getMagNED(magNED);
    ahrs.get_NavEKF().getMagXYZ(magXYZ);
    struct log_EKF2 pkt2 = {
        LOG_PACKET_HEADER_INIT(LOG_EKF2_MSG),
        time_ms : hal.scheduler->millis(),
        Ratio   : (int8_t)(100*ratio),
        AZ1bias : (int8_t)(100*az1bias),
        AZ2bias : (int8_t)(100*az2bias),
        windN   : (int16_t)(100*wind.x),
        windE   : (int16_t)(100*wind.y),
        magN    : (int16_t)(magNED.x),
        magE    : (int16_t)(magNED.y),
        magD    : (int16_t)(magNED.z),
        magX    : (int16_t)(magXYZ.x),
        magY    : (int16_t)(magXYZ.y),
        magZ    : (int16_t)(magXYZ.z)
    };
    WriteBlock(&pkt2, sizeof(pkt2));

	// Write third EKF packet
	Vector3f velInnov;
	Vector3f posInnov;
	Vector3f magInnov;
	float tasInnov;
    ahrs.get_NavEKF().getInnovations(velInnov, posInnov, magInnov, tasInnov);
    struct log_EKF3 pkt3 = {
        LOG_PACKET_HEADER_INIT(LOG_EKF3_MSG),
        time_ms : hal.scheduler->millis(),
        innovVN : (int16_t)(100*velInnov.x),
        innovVE : (int16_t)(100*velInnov.y),
        innovVD : (int16_t)(100*velInnov.z),
        innovPN : (int16_t)(100*posInnov.x),
        innovPE : (int16_t)(100*posInnov.y),
        innovPD : (int16_t)(100*posInnov.z),
        innovMX : (int16_t)(magInnov.x),
        innovMY : (int16_t)(magInnov.y),
        innovMZ : (int16_t)(magInnov.z),
        innovVT : (int16_t)(100*tasInnov)
    };
    WriteBlock(&pkt3, sizeof(pkt3));
	
	// Write fourth EKF packet
    float velVar;
    float posVar;
    float hgtVar;
	Vector3f magVar;
	float tasVar;
    Vector2f offset;
    uint8_t faultStatus, timeoutStatus;
    nav_filter_status solutionStatus;
    ahrs.get_NavEKF().getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
    ahrs.get_NavEKF().getFilterFaults(faultStatus);
    ahrs.get_NavEKF().getFilterTimeouts(timeoutStatus);
    ahrs.get_NavEKF().getFilterStatus(solutionStatus);
    struct log_EKF4 pkt4 = {
        LOG_PACKET_HEADER_INIT(LOG_EKF4_MSG),
        time_ms : hal.scheduler->millis(),
        sqrtvarV : (int16_t)(100*velVar),
        sqrtvarP : (int16_t)(100*posVar),
        sqrtvarH : (int16_t)(100*hgtVar),
        sqrtvarMX : (int16_t)(100*magVar.x),
        sqrtvarMY : (int16_t)(100*magVar.y),
        sqrtvarMZ : (int16_t)(100*magVar.z),
        sqrtvarVT : (int16_t)(100*tasVar),
        offsetNorth : (int8_t)(offset.x),
        offsetEast : (int8_t)(offset.y),
        faults : (uint8_t)(faultStatus),
        timeouts : (uint8_t)(timeoutStatus),
        solution : (uint16_t)(solutionStatus.value)
    };
    WriteBlock(&pkt4, sizeof(pkt4));


    // Write fifth EKF packet
    if (optFlowEnabled) {
        float normInnov; // normalised innovation variance ratio for optical flow observations fused by the main nav filter
        float gndOffset; // estimated vertical position of the terrain relative to the nav filter zero datum
        float flowInnovX, flowInnovY; // optical flow LOS rate vector innovations from the main nav filter
        float auxFlowInnov; // optical flow LOS rate innovation from terrain offset estimator
        float HAGL; // height above ground level
        float rngInnov; // range finder innovations
        float range; // measured range
        float gndOffsetErr; // filter ground offset state error
        ahrs.get_NavEKF().getFlowDebug(normInnov, gndOffset, flowInnovX, flowInnovY, auxFlowInnov, HAGL, rngInnov, range, gndOffsetErr);
        struct log_EKF5 pkt5 = {
            LOG_PACKET_HEADER_INIT(LOG_EKF5_MSG),
            time_ms : hal.scheduler->millis(),
            normInnov : (uint8_t)(min(100*normInnov,255)),
            FIX : (int16_t)(1000*flowInnovX),
            FIY : (int16_t)(1000*flowInnovY),
            AFI : (int16_t)(1000*auxFlowInnov),
            HAGL : (int16_t)(100*HAGL),
            offset : (int16_t)(100*gndOffset),
            RI : (int16_t)(100*rngInnov),
            meaRng : (uint16_t)(100*range),
            errHAGL : (uint16_t)(100*gndOffsetErr)
         };
        WriteBlock(&pkt5, sizeof(pkt5));
    }
}
#endif

// Write a command processing packet
void DataFlash_Class::Log_Write_MavCmd(uint16_t cmd_total, const mavlink_mission_item_t& mav_cmd)
{
    struct log_Cmd pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CMD_MSG),
        time_ms         : hal.scheduler->millis(),
        command_total   : (uint16_t)cmd_total,
        sequence        : (uint16_t)mav_cmd.seq,
        command         : (uint16_t)mav_cmd.command,
        param1          : (float)mav_cmd.param1,
        param2          : (float)mav_cmd.param2,
        param3          : (float)mav_cmd.param3,
        param4          : (float)mav_cmd.param4,
        latitude        : (float)mav_cmd.x,
        longitude       : (float)mav_cmd.y,
        altitude        : (float)mav_cmd.z
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void DataFlash_Class::Log_Write_Radio(const mavlink_radio_t &packet) 
{
    struct log_Radio pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RADIO_MSG),
        time_ms      : hal.scheduler->millis(),
        rssi         : packet.rssi,
        remrssi      : packet.remrssi,
        txbuf        : packet.txbuf,
        noise        : packet.noise,
        remnoise     : packet.remnoise,
        rxerrors     : packet.rxerrors,
        fixed        : packet.fixed
    };
    WriteBlock(&pkt, sizeof(pkt)); 
}

// Write a Camera packet
void DataFlash_Class::Log_Write_Camera(const AP_AHRS &ahrs, const AP_GPS &gps, const Location &current_loc)
{
    int32_t altitude, altitude_rel;
    if (current_loc.flags.relative_alt) {
        altitude = current_loc.alt+ahrs.get_home().alt;
        altitude_rel = current_loc.alt;
    } else {
        altitude = current_loc.alt;
        altitude_rel = current_loc.alt - ahrs.get_home().alt;
    }

    struct log_Camera pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CAMERA_MSG),
        gps_time    : gps.time_week_ms(),
        gps_week    : gps.time_week(),
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        altitude    : altitude,
        altitude_rel: altitude_rel,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write ESC status messages
void DataFlash_Class::Log_Write_ESC(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    static int _esc_status_sub = -1;
    struct esc_status_s esc_status;

    if (_esc_status_sub == -1) {
        // subscribe to ORB topic on first call
        _esc_status_sub = orb_subscribe(ORB_ID(esc_status));  
    } 

    // check for new ESC status data
    bool esc_updated = false;
    orb_check(_esc_status_sub, &esc_updated);
    if (esc_updated && (OK == orb_copy(ORB_ID(esc_status), _esc_status_sub, &esc_status))) {
        if (esc_status.esc_count > 8) {
            esc_status.esc_count = 8;
        }
        uint32_t time_ms = hal.scheduler->millis();
        for (uint8_t i = 0; i < esc_status.esc_count; i++) {
            // skip logging ESCs with a esc_address of zero, and this
            // are probably not populated. The Pixhawk itself should
            // be address zero
            if (esc_status.esc[i].esc_address != 0) {
                struct log_Esc pkt = {
                    LOG_PACKET_HEADER_INIT((uint8_t)(LOG_ESC1_MSG + i)),
                    time_ms     : time_ms,
                    rpm         : (int16_t)(esc_status.esc[i].esc_rpm/10),
                    voltage     : (int16_t)(esc_status.esc[i].esc_voltage*100.f + .5f),
                    current     : (int16_t)(esc_status.esc[i].esc_current*100.f + .5f),
                    temperature : (int16_t)(esc_status.esc[i].esc_temperature*100.f + .5f)
                };

                WriteBlock(&pkt, sizeof(pkt));
            }
        }
    }
#endif // CONFIG_HAL_BOARD
}
