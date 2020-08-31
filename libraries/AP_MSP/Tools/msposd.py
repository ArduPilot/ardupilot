#!/usr/bin/env python
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

parser = argparse.ArgumentParser(description='ArduPilot MSP FPV viewer')
parser.add_argument('--port', default="tcp:localhost:5763", help="port to listen on, can be serial port or tcp:IP:port")

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

WindowWidth = 32 * FontWidth
WindowHeight = 16 * FontHeight
  
# create the display surface object 
# of specific dimension..e(X, Y). 
display_surface = pygame.display.set_mode((WindowWidth,WindowHeight))

# set the pygame window name
pygame.display.set_caption('MSP Display')

def item_to_pos(item):
    '''map MSP item to a X,Y tuple or None'''
    if item is None or item >= msp.msp_osd_config.get('osd_item_count',0):
        return None
    pos = msp.msp_osd_config['osd_items'][item]
    if pos < 2048:
        return None
    pos_y = (pos-2048) // 32
    pos_x = (pos-2048) % 32
    return (pos_x, pos_y)


def display_text(item, message):
    '''display a string message for an item'''
    XY = item_to_pos(item)
    if XY is None:
        return
    (X,Y) = XY
    text = font.render(message, True, white, black)
    textRect = text.get_rect()

    slen = len(message)
    px = X * FontWidth
    py = Y * FontHeight
    textRect.center = (px+textRect.width//2, py+textRect.height//2)
    display_surface.blit(text, textRect)

def display_all():
    '''display all items'''
    display_text(msp.OSD_GPS_SPEED, "%.1fm/s" % (msp.get('RAW_GPS.Speed')*0.01))
    display_text(msp.OSD_GPS_SATS, "%uS" % msp.get("RAW_GPS.numSat"))
    display_text(msp.OSD_ROLL_ANGLE, "Roll:%.1f" % (msp.get("ATTITUDE.roll")*0.1))
    display_text(msp.OSD_PITCH_ANGLE, "Pitch:%.1f" % (msp.get("ATTITUDE.pitch")*0.1))
    display_text(msp.OSD_CURRENT_DRAW, "%.2fA" % (msp.get('BATTERY_STATE.current')*0.01))
    display_text(msp.OSD_MAIN_BATT_VOLTAGE, "%.2fV" % (msp.get('BATTERY_STATE.voltage')*0.1))
    display_text(msp.OSD_ALTITUDE, "%.1fm" % (msp.get("ALTITUDE.alt")*0.01))
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

font = pygame.font.Font('freesansbold.ttf', 12)

last_display_t = time.time()

# infinite loop 
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
        if event.type == pygame.QUIT : 
            pygame.quit()
            quit() 
