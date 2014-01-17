/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "DataFlash.h"
#include <stdlib.h>
#include <AP_Param.h>
#include <AP_Math.h>

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

// This function starts a new log file in the DataFlash, and writes
// the format of supported messages in the log, plus all parameters
uint16_t DataFlash_Class::StartNewLog(void)
{
    uint16_t ret;
    ret = start_new_log();

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
void DataFlash_Class::Log_Write_GPS(const GPS *gps, int32_t relative_alt)
{
    struct log_GPS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GPS_MSG),
    	status        : (uint8_t)gps->status(),
    	gps_week_ms   : gps->time_week_ms,
    	gps_week      : gps->time_week,
        num_sats      : gps->num_sats,
        hdop          : gps->hdop,
        latitude      : gps->latitude,
        longitude     : gps->longitude,
        rel_altitude  : relative_alt,
        altitude      : gps->altitude_cm,
        ground_speed  : gps->ground_speed_cm,
        ground_course : gps->ground_course_cd,
        vel_z         : gps->velocity_down(),
        apm_time      : hal.scheduler->millis()
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an RCIN packet
void DataFlash_Class::Log_Write_RCIN(void)
{
    struct log_RCIN pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RCIN_MSG),
        timestamp     : hal.scheduler->millis(),
        chan1         : hal.rcin->read(0),
        chan2         : hal.rcin->read(1),
        chan3         : hal.rcin->read(2),
        chan4         : hal.rcin->read(3),
        chan5         : hal.rcin->read(4),
        chan6         : hal.rcin->read(5),
        chan7         : hal.rcin->read(6),
        chan8         : hal.rcin->read(7)
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
        chan8         : hal.rcout->read(7)
    };
    WriteBlock(&pkt, sizeof(pkt));
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
        accel_z : accel.z
    };
    WriteBlock(&pkt, sizeof(pkt));
    if (ins.get_gyro_count() < 2 && ins.get_accel_count() < 2) {
        return;
    }
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
        accel_z : accel2.z
    };
    WriteBlock(&pkt2, sizeof(pkt2));
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
