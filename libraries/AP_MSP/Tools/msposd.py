#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
A tool to view MSP telemetry, simulating a DJI FPV display
This is used for testing changes to the ArduPilot implementation of MSP telemetry for FPV

Originally started by Alex Apostoli
based on https://github.com/hkm95/python-multiwii
'''

import pygame
import threading
import time
import sys
import pymsp
import argparse
import socket
import serial
import errno
import math

parser = argparse.ArgumentParser(description='ArduPilot MSP FPV viewer')
parser.add_argument('--port', default="tcp:localhost:5763", help="port to listen on, can be serial port or tcp:IP:port")
parser.add_argument('--font', help="font to use", default="freesans")
parser.add_argument('--list-fonts', action="store_true", default=False, help="show list of system fonts")

args = parser.parse_args()

def connect(port):
    '''connect to port, returning a receive function'''
    try:
        if port.startswith("tcp:"):
            print("Connecting to TCP socket %s" % port)
            a = port[4:].split(':')
            port = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            port.connect((a[0],int(a[1])))
            port.setblocking(0)
            return port.recv
        print("Connecting to serial port %s" % port)
        port = serial.Serial(args.port, 115200, timeout=0.01)
        return port.read
    except Exception as ex:
        return None


recv_func = None
last_read_s = time.time()

msp = pymsp.PyMSP()

pygame.init()

# define the RGB value for white,
#  green, blue colour .
white = (255, 255, 255)
green = (0, 255, 0)
blue = (0, 0, 128)
black = (0, 0 ,0)

# window size
FontWidth = 25
FontHeight = 25

WindowWidth = 27 * FontWidth
WindowHeight = 16 * FontHeight

# create the display surface object
# of specific dimension..e(X, Y).
display_surface = pygame.display.set_mode((WindowWidth,WindowHeight))

# set the pygame window name
pygame.display.set_caption('MSP Display')

def item_to_pos(item):
    '''map MSP item to a X,Y tuple or None'''
    if item is None or item >= msp.get('OSD_CONFIG.osd_item_count'):
        return None
    pos = msp.get("OSD_CONFIG.osd_items")[item]
    if pos < 2048:
        return None
    pos_y = (pos-2048) // 32
    pos_x = (pos-2048) % 32
    return (pos_x, pos_y)

def display_text(item, message, x_offset=0):
    '''display a string message for an item'''
    XY = item_to_pos(item)
    if XY is None:
        return
    (X,Y) = XY
    text = font.render(message, True, white, black)
    textRect = text.get_rect()

    slen = len(message)
    px = X * FontWidth + x_offset
    py = Y * FontHeight
    textRect.center = (px+textRect.width//2, py+textRect.height//2)
    display_surface.blit(text, textRect)

def draw_gauge(x, y, width, height, perc):
    '''draw an horizontal gauge'''
    pygame.draw.rect(display_surface, (255, 255, 255), (x, y, width, height), 1)
    pygame.draw.rect(display_surface, (255, 255, 255), (x+2, y+2, int((width-4)*perc/100), height-4))

def draw_batt_icon(x, y):
    pygame.draw.rect(display_surface, (255, 255, 255), (x+1, y, 6, 3))
    pygame.draw.rect(display_surface, (255, 255, 255), (x, y+3, 8, 14))

def draw_triangle(x, y, r, angle):
    a = angle -90
    ra = math.radians(a)
    x1 = int(x + r * math.cos(ra))
    y1 = int(y + r * math.sin(ra))

    ra = math.radians(a + 140)
    x2 = int(x + r * math.cos(ra))
    y2 = int(y + r * math.sin(ra))

    ra = math.radians(a - 140)
    x3 = int(x + r * math.cos(ra))
    y3 = int(y + r * math.sin(ra))

    ra = math.radians(angle - 270)
    x4 = int(x + r * 0.5 * math.cos(ra))
    y4 = int(y + r * 0.5 *math.sin(ra))

    pygame.draw.line(display_surface, (255, 255, 255), (x1, y1), (x2, y2), 2)
    pygame.draw.line(display_surface, (255, 255, 255), (x1, y1), (x3, y3), 2)
    pygame.draw.line(display_surface, (255, 255, 255), (x2, y2), (x4, y4), 2)
    pygame.draw.line(display_surface, (255, 255, 255), (x3, y3), (x4, y4), 2)

def display_battbar():
    XY = item_to_pos(msp.OSD_MAIN_BATT_USAGE)
    if XY is None:
        return
    (X,Y) = XY
    px = X * FontWidth
    py = Y * FontHeight
    perc = 100
    if msp.get('BATTERY_STATE.capacity') > 0:
        perc = 100 - 100*(float(msp.get('BATTERY_STATE.mah'))/msp.get('BATTERY_STATE.capacity'))
    draw_gauge(px, py, 100, 12, max(0,perc))

def display_homedir():
    XY = item_to_pos(msp.OSD_HOME_DIR)
    if XY is None:
        return
    (X,Y) = XY
    px = X * FontWidth
    py = Y * FontHeight
    yaw = msp.get("ATTITUDE.yaw")                       #deg
    home_angle = msp.get('COMP_GPS.directionToHome')    #deg
    draw_triangle(px+35, py+4, 10, home_angle - yaw)
    display_text(msp.OSD_HOME_DIR, "Hdir")

def display_batt_voltage():
    XY = item_to_pos(msp.OSD_MAIN_BATT_VOLTAGE)
    if XY is None:
        return
    (X,Y) = XY
    px = X * FontWidth
    py = Y * FontHeight
    display_text(msp.OSD_MAIN_BATT_VOLTAGE, "%.2fV" % (msp.get('BATTERY_STATE.voltage')*0.1), 12)
    draw_batt_icon(px,py-6)

def display_cell_voltage():
    XY = item_to_pos(msp.OSD_AVG_CELL_VOLTAGE)
    if XY is None:
        return
    (X,Y) = XY
    px = X * FontWidth
    py = Y * FontHeight
    display_text(msp.OSD_AVG_CELL_VOLTAGE, "%.02fv" % (0 if msp.get('BATTERY_STATE.cellCount')==0 else msp.get('BATTERY_STATE.voltage_cv')/msp.get('BATTERY_STATE.cellCount')*0.01), 12)
    draw_batt_icon(px,py-6)

def display_all():
    '''display all items'''

    '''
    _osd_item_settings[OSD_RSSI_VALUE] = &osd->screen[0].rssi;
    _osd_item_settings[OSD_MAIN_BATT_VOLTAGE] = &osd->screen[0].bat_volt;
    _osd_item_settings[OSD_CROSSHAIRS] = &osd->screen[0].crosshair;
    _osd_item_settings[OSD_ARTIFICIAL_HORIZON] = &osd->screen[0].horizon;
    _osd_item_settings[OSD_HORIZON_SIDEBARS] = &osd->screen[0].sidebars;
    _osd_item_settings[OSD_CRAFT_NAME] = &osd->screen[0].message;
    _osd_item_settings[OSD_FLYMODE] = &osd->screen[0].fltmode;
    _osd_item_settings[OSD_CURRENT_DRAW] = &osd->screen[0].current;
    _osd_item_settings[OSD_MAH_DRAWN] = &osd->screen[0].batused;
    _osd_item_settings[OSD_GPS_SPEED] = &osd->screen[0].gspeed;
    _osd_item_settings[OSD_GPS_SATS] = &osd->screen[0].sats;
    _osd_item_settings[OSD_ALTITUDE] = &osd->screen[0].altitude;
    _osd_item_settings[OSD_POWER] = &osd->screen[0].power;
    _osd_item_settings[OSD_AVG_CELL_VOLTAGE] = &osd->screen[0].cell_volt;
    _osd_item_settings[OSD_GPS_LON] = &osd->screen[0].gps_longitude;
    _osd_item_settings[OSD_GPS_LAT] = &osd->screen[0].gps_latitude;
    _osd_item_settings[OSD_PITCH_ANGLE] = &osd->screen[0].pitch_angle;
    _osd_item_settings[OSD_ROLL_ANGLE] = &osd->screen[0].roll_angle;
    _osd_item_settings[OSD_MAIN_BATT_USAGE] = &osd->screen[0].batt_bar;
    _osd_item_settings[OSD_DISARMED] = &osd->screen[0].arming;
    _osd_item_settings[OSD_HOME_DIR] = &osd->screen[0].home_dir;
    _osd_item_settings[OSD_HOME_DIST] = &osd->screen[0].home_dist;
    _osd_item_settings[OSD_NUMERICAL_HEADING] = &osd->screen[0].heading;
    _osd_item_settings[OSD_NUMERICAL_VARIO] = &osd->screen[0].vspeed;
    _osd_item_settings[OSD_ESC_TMP] = &osd->screen[0].blh_temp;
    _osd_item_settings[OSD_RTC_DATETIME] = &osd->screen[0].clk;
    '''

    display_text(msp.OSD_POWER, "%03dW" % (0 if msp.get('BATTERY_STATE.voltage')==0 or msp.get('BATTERY_STATE.current')==0 else 0.1*msp.get('BATTERY_STATE.voltage')/msp.get('BATTERY_STATE.voltage')*msp.get('BATTERY_STATE.current')))
    display_battbar()
    display_cell_voltage()
    display_batt_voltage()
    display_text(msp.OSD_GPS_LAT, "Lat:%.07f" % (msp.get('RAW_GPS.Lat')*0.0000001))
    display_text(msp.OSD_GPS_LON, "Lon:%.07f" % (msp.get('RAW_GPS.Lon')*0.0000001))
    display_text(msp.OSD_RTC_DATETIME, "%04d-%02d-%02d %02d:%02d:%02d" % (msp.get('RTC.year'),msp.get('RTC.mon'),msp.get('RTC.mday'),msp.get('RTC.hour'),msp.get('RTC.min'),msp.get('RTC.sec')))
    display_text(msp.OSD_DISARMED, "%s" % ("" if msp.get('STATUS.mode_flags')&0X01==1 else "DISARMED"))
    display_homedir()
    display_text(msp.OSD_HOME_DIST, "Dist: %dm" % (msp.get('COMP_GPS.distanceToHome')))
    display_text(msp.OSD_RSSI_VALUE, "Rssi:%02d" % (msp.get('ANALOG.rssi') * 0.097))
    display_text(msp.OSD_GPS_SPEED, "%.1fm/s" % (msp.get('RAW_GPS.Speed')*0.01))
    display_text(msp.OSD_GPS_SATS, "Sats:%u" % msp.get("RAW_GPS.numSat"))
    display_text(msp.OSD_ROLL_ANGLE, "Roll:%.1f" % (msp.get("ATTITUDE.roll")*0.1))
    display_text(msp.OSD_PITCH_ANGLE, "Pitch:%.1f" % (msp.get("ATTITUDE.pitch")*0.1))
    display_text(msp.OSD_CURRENT_DRAW, "%02.2fA" % (msp.get('BATTERY_STATE.current')*0.01))
    display_text(msp.OSD_ALTITUDE, "%.1fm" % (msp.get("ALTITUDE.alt")*0.01))
    display_text(msp.OSD_NUMERICAL_VARIO, u"↕ %.01fm/s" % (msp.get('ALTITUDE.vspeed')*0.01))
    display_text(msp.OSD_MAH_DRAWN, "%umAh" % (msp.get('BATTERY_STATE.mah')))
    display_text(msp.OSD_CRAFT_NAME, "%s" % (msp.msp_name['name']))

def receive_data():
    '''receive any data from socket'''
    global recv_func, last_read_s
    while recv_func is None:
        time.sleep(0.5)
        recv_func = connect(args.port)
    try:
        buf = recv_func(100)
        if not buf:
            if time.time() - last_read_s > 1:
                recv_func = None
            return
        last_read_s = time.time()
    except IOError as e:
        if e.errno == errno.EWOULDBLOCK:
            return
        recv_func = None
        return
    msp.parseMspData(buf)

if args.list_fonts:
    print(sorted(pygame.font.get_fonts()))
font = pygame.font.SysFont(args.font, 12)

def run():
    last_display_t = time.time()

    while True:
        receive_data()
        now = time.time()

        if now - last_display_t > 0.1:
            # display at 10Hz
            last_display_t = now
            display_surface.fill(black)
            display_all()
            pygame.display.update()
            time.sleep(0.01)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

try:
    run()
except KeyboardInterrupt:
    pass
