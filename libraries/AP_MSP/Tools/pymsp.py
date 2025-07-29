#!/usr/bin/env python3

# flake8: noqa

"""
 author: Alex Apostoli
 based on https://github.com/hkm95/python-multiwii
 which is under GPLv3
"""

import struct
import time
import sys
import re

class MSPItem:
    def __init__(self, name, fmt, fields):
        self.name = name
        self.format = fmt
        self.fields = fields
        if not isinstance(self.format, list):
            self.format = [self.format]
            self.fields = [self.fields]
        self.values = {}

    def parse(self, msp, dataSize):
        '''parse data'''
        ofs = msp.p
        for i in range(len(self.format)):
            fmt = self.format[i]
            fields = self.fields[i].split(',')
            if fmt[0] == '{':
                # we have a repeat count from an earlier variable
                right = fmt.find('}')
                vname = fmt[1:right]
                count = self.values[vname]
                fmt = "%u%s" % (count, fmt[right+1:])
            if fmt[0].isdigit():
                repeat = int(re.search(r'\d+', fmt).group())
            else:
                repeat = None
            fmt = "<" + fmt
            fmt_size = struct.calcsize(fmt)
            if dataSize < fmt_size:
                raise Exception("Format %s needs %u bytes got %u for %s" % (self.name, fmt_size, dataSize, fmt))
            values = list(struct.unpack(fmt, msp.inBuf[ofs:ofs+fmt_size]))
            if repeat is not None:
                for i in range(len(fields)):
                    self.values[fields[i]] = []
                    for j in range(repeat):
                        self.values[fields[i]].append(values[j*len(fields)])
            else:
                for i in range(len(fields)):
                    self.values[fields[i]] = values[i]
            dataSize -= fmt_size
            ofs += fmt_size
        msp.by_name[self.name] = self
        #print("Got %s" % self.name)

class PyMSP:
    """ Multiwii Serial Protocol """
    OSD_RSSI_VALUE              = 0
    OSD_MAIN_BATT_VOLTAGE       = 1
    OSD_CROSSHAIRS              = 2
    OSD_ARTIFICIAL_HORIZON      = 3
    OSD_HORIZON_SIDEBARS        = 4
    OSD_ITEM_TIMER_1            = 5
    OSD_ITEM_TIMER_2            = 6
    OSD_FLYMODE                 = 7
    OSD_CRAFT_NAME              = 8
    OSD_THROTTLE_POS            = 9
    OSD_VTX_CHANNEL             = 10
    OSD_CURRENT_DRAW            = 11
    OSD_MAH_DRAWN               = 12
    OSD_GPS_SPEED               = 13
    OSD_GPS_SATS                = 14
    OSD_ALTITUDE                = 15
    OSD_ROLL_PIDS               = 16
    OSD_PITCH_PIDS              = 17
    OSD_YAW_PIDS                = 18
    OSD_POWER                   = 19
    OSD_PIDRATE_PROFILE         = 20
    OSD_WARNINGS                = 21
    OSD_AVG_CELL_VOLTAGE        = 22
    OSD_GPS_LON                 = 23
    OSD_GPS_LAT                 = 24
    OSD_DEBUG                   = 25
    OSD_PITCH_ANGLE             = 26
    OSD_ROLL_ANGLE              = 27
    OSD_MAIN_BATT_USAGE         = 28
    OSD_DISARMED                = 29
    OSD_HOME_DIR                = 30
    OSD_HOME_DIST               = 31
    OSD_NUMERICAL_HEADING       = 32
    OSD_NUMERICAL_VARIO         = 33
    OSD_COMPASS_BAR             = 34
    OSD_ESC_TMP                 = 35
    OSD_ESC_RPM                 = 36
    OSD_REMAINING_TIME_ESTIMATE = 37
    OSD_RTC_DATETIME            = 38
    OSD_ADJUSTMENT_RANGE        = 39
    OSD_CORE_TEMPERATURE        = 40
    OSD_ANTI_GRAVITY            = 41
    OSD_G_FORCE                 = 42
    OSD_MOTOR_DIAG              = 43
    OSD_LOG_STATUS              = 44
    OSD_FLIP_ARROW              = 45
    OSD_LINK_QUALITY            = 46
    OSD_FLIGHT_DIST             = 47
    OSD_STICK_OVERLAY_LEFT      = 48
    OSD_STICK_OVERLAY_RIGHT     = 49
    OSD_DISPLAY_NAME            = 50
    OSD_ESC_RPM_FREQ            = 51
    OSD_RATE_PROFILE_NAME       = 52
    OSD_PID_PROFILE_NAME        = 53
    OSD_PROFILE_NAME            = 54
    OSD_RSSI_DBM_VALUE          = 55
    OSD_RC_CHANNELS             = 56
    OSD_CAMERA_FRAME            = 57

    MSP_NAME                 =10
    MSP_OSD_CONFIG           =84
    MSP_IDENT                =100
    MSP_STATUS               =101
    MSP_RAW_IMU              =102
    MSP_SERVO                =103
    MSP_MOTOR                =104
    MSP_RC                   =105
    MSP_RAW_GPS              =106
    MSP_COMP_GPS             =107
    MSP_ATTITUDE             =108
    MSP_ALTITUDE             =109
    MSP_ANALOG               =110
    MSP_RC_TUNING            =111
    MSP_PID                  =112
    MSP_BOX                  =113
    MSP_MISC                 =114
    MSP_MOTOR_PINS           =115
    MSP_BOXNAMES             =116
    MSP_PIDNAMES             =117
    MSP_WP                   =118
    MSP_BOXIDS               =119
    MSP_SERVO_CONF           =120
    MSP_NAV_STATUS           =121
    MSP_NAV_CONFIG           =122
    MSP_MOTOR_3D_CONFIG      =124
    MSP_RC_DEADBAND          =125
    MSP_SENSOR_ALIGNMENT     =126
    MSP_LED_STRIP_MODECOLOR  =127
    MSP_VOLTAGE_METERS       =128
    MSP_CURRENT_METERS       =129
    MSP_BATTERY_STATE        =130
    MSP_MOTOR_CONFIG         =131
    MSP_GPS_CONFIG           =132
    MSP_COMPASS_CONFIG       =133
    MSP_ESC_SENSOR_DATA      =134
    MSP_GPS_RESCUE           =135
    MSP_GPS_RESCUE_PIDS      =136
    MSP_VTXTABLE_BAND        =137
    MSP_VTXTABLE_POWERLEVEL  =138
    MSP_MOTOR_TELEMETRY      =139

    MSP_SET_RAW_RC           =200
    MSP_SET_RAW_GPS          =201
    MSP_SET_PID              =202
    MSP_SET_BOX              =203
    MSP_SET_RC_TUNING        =204
    MSP_ACC_CALIBRATION      =205
    MSP_MAG_CALIBRATION      =206
    MSP_SET_MISC             =207
    MSP_RESET_CONF           =208
    MSP_SET_WP               =209
    MSP_SELECT_SETTING       =210
    MSP_SET_HEAD             =211
    MSP_SET_SERVO_CONF       =212
    MSP_SET_MOTOR            =214
    MSP_SET_NAV_CONFIG       =215
    MSP_SET_MOTOR_3D_CONFIG  =217
    MSP_SET_RC_DEADBAND      =218
    MSP_SET_RESET_CURR_PID   =219
    MSP_SET_SENSOR_ALIGNMENT =220
    MSP_SET_LED_STRIP_MODECOLOR=221
    MSP_SET_MOTOR_CONFIG     =222
    MSP_SET_GPS_CONFIG       =223
    MSP_SET_COMPASS_CONFIG   =224
    MSP_SET_GPS_RESCUE       =225
    MSP_SET_GPS_RESCUE_PIDS  =226
    MSP_SET_VTXTABLE_BAND    =227
    MSP_SET_VTXTABLE_POWERLEVEL=228


    MSP_BIND                 =241
    MSP_RTC                  =247

    MSP_EEPROM_WRITE         =250

    MSP_DEBUGMSG             =253
    MSP_DEBUG                =254

    IDLE = 0
    HEADER_START = 1
    HEADER_M = 2
    HEADER_ARROW = 3
    HEADER_SIZE = 4
    HEADER_CMD = 5
    HEADER_ERR = 6

    PIDITEMS = 10

    MESSAGES = {
        MSP_RAW_GPS:   MSPItem('RAW_GPS', "BBiihH", "fix,numSat,Lat,Lon,Alt,Speed"),
        MSP_IDENT:     MSPItem('IDENT', "BBBI", "version,multiType,MSPVersion,multiCapability"),
        MSP_STATUS:    MSPItem('STATUS', "HHHI", "cycleTime,i2cError,present,mode"),
        MSP_RAW_IMU:   MSPItem('RAW_IMU', "hhhhhhhhh", "AccX,AccY,AccZ,GyrX,GyrY,GyrZ,MagX,MagY,MagZ"),
        MSP_SERVO:     MSPItem('SERVO', "8h", "servo"),
        MSP_MOTOR:     MSPItem('MOTOR', "8h", "motor"),
        MSP_RC:        MSPItem('RC', "8h", "rc"),
        MSP_COMP_GPS:  MSPItem('COMP_GPS', "HhB", "distanceToHome,directionToHome,update"),
        MSP_ATTITUDE:  MSPItem('ATTITUDE', "hhh", "roll,pitch,yaw"),
        MSP_ALTITUDE:  MSPItem('ALTITUDE', "ih", "alt,vspeed"),
        MSP_RC_TUNING: MSPItem('RC_TUNING', "BBBBBBB", "RC_Rate,RC_Expo,RollPitchRate,YawRate,DynThrPID,ThrottleMID,ThrottleExpo"),
        MSP_BATTERY_STATE: MSPItem('BATTERY_STATE', "BHBHhBh", "cellCount,capacity,voltage,mah,current,state,voltage_cv"),
        MSP_RTC:       MSPItem('RTC', "HBBBBBH", "year,mon,mday,hour,min,sec,millis"),
        MSP_OSD_CONFIG: MSPItem("OSD_CONFIG",
                                ["BBBBHBBH",
                                 "{osd_item_count}H",
                                 "B", "{stats_item_count}H",
                                 "B", "{timer_count}H",
                                 "HBIBBB"],
                                ["feature,video_system,units,rssi_alarm,cap_alarm,unused1,osd_item_count,alt_alarm",
                                 "osd_items",
                                 "stats_item_count", "stats_items",
                                 "timer_count", "timer_items",
                                 "legacy_warnings,warnings_count,enabled_warnings,profiles,selected_profile,osd_overlay"]),
        MSP_PID:       MSPItem("PID", "8PID", "P,I,D"),
        MSP_MISC:      MSPItem("MISC", "HHHHHII","intPowerTrigger,conf1,conf2,conf3,conf4,conf5,conf6"),
        MSP_MOTOR_PINS: MSPItem("MOTOR_PINS", "8H","MP"),
        MSP_ANALOG:    MSPItem("ANALOG", "BHHHH", "dV,consumed_mah,rssi,current,volt"),
        MSP_STATUS:    MSPItem("STATUS", "HHHIBHHBBIB", "task_delta,i2c_err_count,sensor_status,mode_flags,nop_1,system_load,gyro_time,nop_2,nop_3,armed,extra"),
        MSP_ESC_SENSOR_DATA:    MSPItem('ESC', "BH", "temp1,rpm1"),
        }

    def __init__(self):

        self.msp_name = {
            'name':None
            }
        self.msp_osd_config = {}

        self.inBuf = bytearray([0] * 255)
        self.p = 0
        self.c_state = self.IDLE
        self.err_rcvd = False
        self.checksum = 0
        self.cmd = 0
        self.offset=0
        self.dataSize=0
        self.servo = []
        self.mot = []
        self.RCChan = []
        self.byteP = []
        self.byteI = []
        self.byteD = []
        self.confINF = []
        self.byteMP = []

        self.confP = []
        self.confI = []
        self.confD = []

        # parsed messages, indexed by name
        self.by_name = {}

    def get(self, fieldname):
        '''get a field from a parsed message by Message.Field name'''
        a = fieldname.split('.')
        msgName = a[0]
        fieldName = a[1]
        if not msgName in self.by_name:
            # default to zero for simplicity of display
            return 0
        msg = self.by_name[msgName]
        if not fieldName in msg.values:
            raise Exception("Unknown field %s" % fieldName)
        return msg.values[fieldName]

    def read32(self):
        '''signed 32 bit number'''
        value, = struct.unpack("<i", self.inBuf[self.p:self.p+4])
        self.p += 4
        return value

    def read32u(self):
        '''unsigned 32 bit number'''
        value, = struct.unpack("<I", self.inBuf[self.p:self.p+4])
        self.p += 4
        return value

    def read16(self):
        '''signed 16 bit number'''
        value, = struct.unpack("<h", self.inBuf[self.p:self.p+2])
        self.p += 2
        return value

    def read16u(self):
        '''unsigned 16 bit number'''
        value, = struct.unpack("<H", self.inBuf[self.p:self.p+2])
        self.p += 2
        return value

    def read8(self):
        '''unsigned 8 bit number'''
        value, = struct.unpack("<B", self.inBuf[self.p:self.p+1])
        self.p += 1
        return value

    def requestMSP (self, msp, payload = [], payloadinbytes = False):

        if msp < 0:
            return 0
        checksum = 0
        bf = ['$', 'M', '<']

        pl_size = 2 * ((len(payload)) & 0xFF)
        bf.append(pl_size)
        checksum ^= (pl_size&0xFF)

        bf.append(msp&0xFF)
        checksum ^= (msp&0xFF)
        if payload > 0:
            if (payloadinbytes == False):
                for c in struct.pack('<%dh' % ((pl_size) / 2), *payload):
                    checksum ^= (ord(c) & 0xFF)
            else:
                for c in struct.pack('<%Bh' % ((pl_size) / 2), *payload):
                    checksum ^= (ord(c) & 0xFF)
        bf = bf + payload
        bf.append(checksum)
        return bf

    def evaluateCommand(self, cmd, dataSize):
        if cmd in self.MESSAGES:
            # most messages are parsed from the MESSAGES list
            self.MESSAGES[cmd].parse(self, dataSize)
        elif cmd == self.MSP_NAME:
            s = bytearray()
            for i in range(0,dataSize,1):
                b = self.read8()
                if b == 0:
                    break
                s.append(b)
            self.msp_name['name'] = s.decode("utf-8")

        elif cmd == self.MSP_ACC_CALIBRATION:
            x = None
        elif cmd == self.MSP_MAG_CALIBRATION:
            x = None
        elif cmd == self.MSP_BOX:
            x = None
        elif cmd == self.MSP_BOXNAMES:
            x = None
        elif cmd == self.MSP_PIDNAMES:
            x = None
        elif cmd == self.MSP_SERVO_CONF:
            x = None
        elif cmd == self.MSP_DEBUGMSG:
            x = None
        elif cmd == self.MSP_DEBUG:
            x = None
        else:
            print("Unhandled command ", cmd, dataSize)

    def parseMspData(self, buf):
        for c in buf:
            self.parseMspByte(c)

    def parseMspByte(self, c):
        if sys.version_info.major >= 3:
            cc = chr(c)
            ci = c
        else:
            cc = c
            ci = ord(c)
        if self.c_state == self.IDLE:
            if cc == '$':
                self.c_state = self.HEADER_START
            else:
                self.c_state = self.IDLE
        elif self.c_state == self.HEADER_START:
            if cc == 'M':
                self.c_state = self.HEADER_M
            else:
                self.c_state = self.IDLE
        elif self.c_state == self.HEADER_M:
            if cc == '>':
                self.c_state = self.HEADER_ARROW
            elif cc == '!':
                self.c_state = self.HEADER_ERR
            else:
                self.c_state = self.IDLE

        elif self.c_state == self.HEADER_ARROW or self.c_state == self.HEADER_ERR:
            self.err_rcvd = (self.c_state == self.HEADER_ERR)
            #print (struct.unpack('<B',c)[0])
            self.dataSize = ci
            # reset index variables
            self.p = 0
            self.offset = 0
            self.checksum = 0
            self.checksum ^= ci
            # the command is to follow
            self.c_state = self.HEADER_SIZE
        elif self.c_state == self.HEADER_SIZE:
            #print (struct.unpack('<B',c)[0])
            self.cmd = ci
            self.checksum ^= ci
            self.c_state = self.HEADER_CMD
        elif self.c_state == self.HEADER_CMD and self.offset < self.dataSize:
            #print (struct.unpack('<B',c)[0])
            self.checksum ^= ci
            self.inBuf[self.offset] = ci
            self.offset += 1
        elif self.c_state == self.HEADER_CMD and self.offset >= self.dataSize:
            # compare calculated and transferred checksum
            if ((self.checksum&0xFF) == ci):
                if self.err_rcvd:
                    print("Vehicle didn't understand the request type")
                else:
                    self.evaluateCommand(self.cmd, self.dataSize)
            else:
                print('"invalid checksum for command "+((int)(cmd&0xFF))+": "+(checksum&0xFF)+" expected, got "+(int)(c&0xFF))')

            self.c_state = self.IDLE

    def setPID(self):
        self.sendRequestMSP(self.requestMSP(self.MSP_PID))
        self.receiveData(self.MSP_PID)
        time.sleep(0.04)
        payload = []
        for i in range(0, self.PIDITEMS, 1):
            self.byteP[i] = int((round(self.confP[i] * 10)))
            self.byteI[i] = int((round(self.confI[i] * 1000)))
            self.byteD[i] = int((round(self.confD[i])))


        # POS - 4 POSR - 5 NAVR - 6

        self.byteP[4] = int((round(self.confP[4] * 100.0)))
        self.byteI[4] = int((round(self.confI[4] * 100.0)))
        self.byteP[5] = int((round(self.confP[5] * 10.0)))
        self.byteI[5] = int((round(self.confI[5] * 100.0)))
        self.byteD[5] = int((round(self.confD[5] * 10000.0))) / 10

        self.byteP[6] = int((round(self.confP[6] * 10.0)))
        self.byteI[6] = int((round(self.confI[6] * 100.0)))
        self.byteD[6] = int((round(self.confD[6] * 10000.0))) / 10

        for i in range(0, self.PIDITEMS, 1):
            payload.append(self.byteP[i])
            payload.append(self.byteI[i])
            payload.append(self.byteD[i])
        self.sendRequestMSP(self.requestMSP(self.MSP_SET_PID, payload, True), True)


    def arm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,2000,1000]
            self.sendRequestMSP(self.requestMSP(self.MSP_SET_RAW_RC,data))
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()

    def disarm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,1000,1000]
            self.sendRequestMSP(self.requestMSP(self.MSP_SET_RAW_RC,data))
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()


    def receiveIMU(self, duration):
        timer = 0
        start = time.time()
        while timer < duration:
            self.sendRequestMSP(self.requestMSP(self.MSP_RAW_IMU))
            self.receiveData(self.MSP_RAW_IMU)
            if self.msp_raw_imu['accx'] > 32768:  # 2^15 ...to check if negative number is received
                self.msp_raw_imu['accx'] -= 65536 # 2^16 ...converting into 2's complement
            if self.msp_raw_imu['accy'] > 32768:
                self.msp_raw_imu['accy'] -= 65536
            if self.msp_raw_imu['accz'] > 32768:
                self.msp_raw_imu['accz'] -= 65536
            if self.msp_raw_imu['gyrx'] > 32768:
                self.msp_raw_imu['gyrx'] -= 65536
            if self.msp_raw_imu['gyry'] > 32768:
                self.msp_raw_imu['gyry'] -= 65536
            if self.msp_raw_imu['gyrz'] > 32768:
                self.msp_raw_imu['gyrz'] -= 65536
            print("size: %d, accx: %f, accy: %f, accz: %f, gyrx: %f, gyry: %f, gyrz: %f  " %(self.msp_raw_imu['size'], self.msp_raw_imu['accx'], self.msp_raw_imu['accy'], self.msp_raw_imu['accz'], self.msp_raw_imu['gyrx'], self.msp_raw_imu['gyry'], self.msp_raw_imu['gyrz']))
            time.sleep(0.04)
            timer = timer + (time.time() - start)
            start = time.time()


    def calibrateIMU(self):
        self.sendRequestMSP(self.requestMSP(self.MSP_ACC_CALIBRATION))
        time.sleep(0.01)
