#!/usr/bin/env python3
'''
Model for data downloaded from Aerobridge
'''

from dataclasses import dataclass

@dataclass
class FirmwareVersionAndHash:
    '''A class to hold information about Firmware version and hash from the board'''
    flight_sw_version: str
    flight_sw_git_hash: str

@dataclass
class HardwareUID:
    '''A class to hold information about Hardware version of the drone'''
    hardware_uid : str
    
