#!/usr/bin/env python

"""
 author: Alex Apostoli
 based on https://github.com/hkm95/python-multiwii
 which is under GPLv3
"""

import struct
import time

class MSPItem:
    def __init__(self, name, fmt, fields):
        self.name = name
        self.format = fmt
        self.fields = fields.split(',')
        self.values = {}
        self.fmt_size = struct.calcsize(self.format)

    def parse(self, msp, dataSize):
        '''parse data'''
        if dataSize < self.fmt_size:
            raise Exception("Format %s needs %u bytes got %u" % (self.name, self.fmt_size, dataSize))
        values = list(struct.unpack(self.format, msp.inBuf[msp.p:msp.p+self.fmt_size]))
        for i in range(len(self.fields)):
            self.values[self.fields[i]] = values[i]
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
        MSP_RAW_GPS:   MSPItem('RAW_GPS', "<BBiihH", "fix,numSat,Lat,Lon,Alt,Speed"),
        MSP_IDENT:     MSPItem('IDENT', "<BBBI", "version,multiType,MSPVersion,multiCapability"),
        MSP_STATUS:    MSPItem('STATUS', "<HHHI", "cycleTime,i2cError,present,mode"),
        MSP_RAW_IMU:   MSPItem('RAW_IMU', "<hhhhhhhhh", "AccX,AccY,AccZ,GyrX,GyrY,GyrZ,MagX,MagY,MagZ"),
        MSP_SERVO:     MSPItem('SERVO', "<8h", "servo"),
        MSP_MOTOR:     MSPItem('MOTOR', "<8h", "motor"),
        MSP_RC:        MSPItem('RC', "<8h", "rc"),
        MSP_COMP_GPS:  MSPItem('COMP_GPS', "<HhB", "distanceToHome,directionToHome,update"),
        MSP_ATTITUDE:  MSPItem('ATTITUDE', "<hhh", "roll,pitch,yaw"),
        MSP_ALTITUDE:  MSPItem('ALTITUDE', "<ih", "alt,vspeed"),
        MSP_RC_TUNING: MSPItem('RC_TUNING', "<BBBBBBB", "RC_Rate,RC_Expo,RollPitchRate,YawRate,DynThrPID,ThrottleMID,ThrottleExpo"),
        MSP_BATTERY_STATE: MSPItem('BATTERY_STATE', "<BHBHh", "cellCount,capacity,voltage,mah,current"),
        MSP_RTC:       MSPItem('RTC', "<HBBBBBH", "year,mon,mday,hour,min,sec,millis"),
        }

    def __init__(self):

        self.msp_name = {
            'name':None 
            }
        self.msp_osd_config = {
            'feature':None,                # 8
            'video_system':None,           # 8
            'units':None,                  # 8
            'rssi_alarm':None,             # 8
            'cap_alarm':None,              # 16
            'unusaed_1':None,              # 8
            'osd_item_count':None,         # 8
            'alt_alarm':None,              # 16
            'osd_items': [None] * 60,   # x16
            'stats_item_count':None,       # 8
            'stats_items': [None] * 30, # x16
            'timer_count':None,            # 8
            'timer_items': [None] * 10, # 16
            'legacy_warnings':None,        # 16
            'warnings_count':None,         # 8
            'enabled_warnings':None,       # 32
            'profiles':None,               # 8
            'selected_profile':None,       # 8
            'osd_overlay':None,            # 8   
        }

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
            # default to zero for simplicty of display
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
        value, = struct.unpack("<h", str(self.inBuf[self.p:self.p+2]))
        self.p += 2
        return value

    def read16u(self):
        '''unsigned 16 bit number'''
        value, = struct.unpack("<H", str(self.inBuf[self.p:self.p+2]))
        self.p += 2
        return value
    
    def read8(self):
        '''unsigned 8 bit number'''
        value, = struct.unpack("<B", str(self.inBuf[self.p:self.p+1]))
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
            s = ''
            for i in range(0,dataSize,1):
                b = self.read8()
                if b == 0:
                    break
                s += chr(b)
            self.msp_name['name'] = s

        elif cmd == self.MSP_ANALOG:
            x = None

        elif cmd == self.MSP_ACC_CALIBRATION:
            x = None

        elif cmd == self.MSP_MAG_CALIBRATION:
            x = None

        elif cmd == self.MSP_PID:
            for i in range(0, 8, 1):
                self.byteP[i] = (self.read8())
                self.byteI[i] = (self.read8())
                self.byteD[i] = (self.read8())
                if (i != 4) and (i != 5) and (i != 6):
                    self.confP[i] = (float(self.byteP[i])/10.0)
                    self.confI[i] = (float(self.byteI[i])/1000.0)
                    self.confD[i] = (float(self.byteD[i]))
            self.confP[4] = (float(self.byteP[4]) / 100.0)
            self.confI[4] = (float(self.byteI[4]) / 100.0)
            self.confD[4] = (float(self.byteD[4]) / 1000.0)
            self.confP[5] = (float(self.byteP[5]) / 10.0)
            self.confI[5] = (float(self.byteI[5]) / 100.0)
            self.confD[5] = (float(self.byteD[5]) / 1000.0)
            self.confP[6] = (float(self.byteP[6]) / 10.0)
            self.confI[6] = (float(self.byteI[6]) / 100.0)
            self.confD[6] = (float(self.byteD[6]) / 1000.0)
        elif cmd == self.MSP_BOX:
            x = None

        elif cmd == self.MSP_BOXNAMES:
            x = None

        elif cmd == self.MSP_PIDNAMES:
            x = None

        elif cmd == self.MSP_SERVO_CONF:
            x = None

        elif cmd == self.MSP_MISC:
            self.msp_misc['intPowerTrigger'] = self.read16u()
            for i in range(0,4,1):
                self.MConf[i] = self.read16u()
            self.MConf[4] = self.read32u()
            self.MConf[5] = self.read32u()

        elif cmd == self.MSP_MOTOR_PINS:
            for i in range(0, 8, 1):
                self.byteMP.append(self.read16())

        elif cmd == self.MSP_DEBUGMSG:
            x = None
        elif cmd == self.MSP_DEBUG:
            x = None
        elif cmd == self.MSP_OSD_CONFIG:
            self.msp_osd_config['feature'] = int(self.read8())                # 8
            self.msp_osd_config['video_system'] = self.read8()           # 8
            self.msp_osd_config['units'] = self.read8()                  # 8
            self.msp_osd_config['rssi_alarm'] = self.read8()             # 8
            self.msp_osd_config['cap_alarm'] = self.read16()             # 16
            self.msp_osd_config['unusaed_1'] = self.read8()              # 8
            self.msp_osd_config['osd_item_count'] = self.read8()         # 8
            self.msp_osd_config['alt_alarm'] = self.read16()             # 16
            for i in range(0, self.msp_osd_config['osd_item_count'], 1):
                self.msp_osd_config['osd_items'][i] = self.read16u()      # x 16
            self.msp_osd_config['stats_item_count'] = self.read8()       # 8
            for i in range(0, self.msp_osd_config['stats_item_count'], 1):
                self.msp_osd_config['stats_items'][i] = self.read16u()    # x 16
            self.msp_osd_config['timer_count'] = self.read8()            # 8
            for i in range(0, self.msp_osd_config['timer_count'], 1):
                self.msp_osd_config['timer_items'][i] = self.read16u()    # x 16
            self.msp_osd_config['legacy_warnings'] = self.read16u()       # 16
            self.msp_osd_config['warnings_count'] = self.read8()         # 8
            self.msp_osd_config['enabled_warnings'] = self.read32u()      # 32
            self.msp_osd_config['profiles'] = self.read8()               # 8
            self.msp_osd_config['selected_profile'] = self.read8()       # 8
            self.msp_osd_config['osd_overlay'] = self.read8()
                        # 8
        else:
            print("Unhandled command ", cmd, dataSize)

    def parseMspData(self, buf):
        for c in buf:
            self.parseMspByte(c)

    def parseMspByte(self, c):
        if self.c_state == self.IDLE:
            if c == '$':
                self.c_state = self.HEADER_START
            else:
                self.c_state = self.IDLE
        elif self.c_state == self.HEADER_START:
            if c == 'M':
                self.c_state = self.HEADER_M
            else:
                self.c_state = self.IDLE
        elif self.c_state == self.HEADER_M:
            if c == '>':
                self.c_state = self.HEADER_ARROW
            elif c == '!':
                self.c_state = self.HEADER_ERR
            else:
                self.c_state = self.IDLE
    
        elif self.c_state == self.HEADER_ARROW or self.c_state == self.HEADER_ERR:
            self.err_rcvd = (self.c_state == self.HEADER_ERR)
            #print (struct.unpack('<B',c)[0])
            self.dataSize = ((struct.unpack('<B',c)[0])&0xFF)
            # reset index variables
            self.p = 0
            self.offset = 0
            self.checksum = 0
            self.checksum ^= ((struct.unpack('<B',c)[0])&0xFF)
            # the command is to follow
            self.c_state = self.HEADER_SIZE
        elif self.c_state == self.HEADER_SIZE:
            #print (struct.unpack('<B',c)[0])
            self.cmd = ((struct.unpack('<B',c)[0])&0xFF)
            self.checksum ^= ((struct.unpack('<B',c)[0])&0xFF)
            self.c_state = self.HEADER_CMD
        elif self.c_state == self.HEADER_CMD and self.offset < self.dataSize:
            #print (struct.unpack('<B',c)[0])
            self.checksum ^= ((struct.unpack('<B',c)[0])&0xFF)
            self.inBuf[self.offset] = c
            self.offset += 1
        elif self.c_state == self.HEADER_CMD and self.offset >= self.dataSize:
            # compare calculated and transferred checksum
            #print "Final step..."
            if ((self.checksum&0xFF) == ((struct.unpack('<B',c)[0])&0xFF)):
                if self.err_rcvd:
                    print "Copter didn't understand the request type"
                else:
                    self.evaluateCommand(self.cmd, self.dataSize)
            else:
                print '"invalid checksum for command "+((int)(cmd&0xFF))+": "+(checksum&0xFF)+" expected, got "+(int)(c&0xFF))'

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
        print "Payload:..."
        print payload
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
            print "size: %d, accx: %f, accy: %f, accz: %f, gyrx: %f, gyry: %f, gyrz: %f  " %(self.msp_raw_imu['size'], self.msp_raw_imu['accx'], self.msp_raw_imu['accy'], self.msp_raw_imu['accz'], self.msp_raw_imu['gyrx'], self.msp_raw_imu['gyry'], self.msp_raw_imu['gyrz'])
            time.sleep(0.04)
            timer = timer + (time.time() - start)
            start = time.time()


    def calibrateIMU(self):
        self.sendRequestMSP(self.requestMSP(self.MSP_ACC_CALIBRATION))
        time.sleep(0.01)
