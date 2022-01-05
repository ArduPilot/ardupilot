from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import math
import time

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def adds_alphabet_mission(aLocation, aSize):
    """
    アルファベット軌跡の定義
    """
    Alphabet_Mission_Plan = {
        "A" : [    #アルファベット軌跡Aに関するウェイポイントリスト
            get_location_metres(aLocation,0,0),
            get_location_metres(aLocation,-2,0),
            get_location_metres(aLocation,-7,13),
            get_location_metres(aLocation,-8,13),
            get_location_metres(aLocation,-11,5),
            get_location_metres(aLocation,-7,5),
            get_location_metres(aLocation,-11,5),
            get_location_metres(aLocation,-13,0),
            get_location_metres(aLocation,-15,0),
            # 目標WP到達確認用
            get_location_metres(aLocation,-15,0)
        ],

        "B" : [    #アルファベット軌跡Bに関するウェイポイントリスト
            get_location_metres(aLocation,0,0),
            get_location_metres(aLocation,-3,0),
            get_location_metres(aLocation,-3,13),
            get_location_metres(aLocation,-10,13),
            get_location_metres(aLocation,-11,12),
            get_location_metres(aLocation,-12,12),
            get_location_metres(aLocation,-13,11),
            get_location_metres(aLocation,-13,9),
            get_location_metres(aLocation,-12,8),
            get_location_metres(aLocation,-11,8),
            get_location_metres(aLocation,-10,7),
            get_location_metres(aLocation,-7,7),
            get_location_metres(aLocation,-7,6),
            get_location_metres(aLocation,-10,6),
            get_location_metres(aLocation,-11,5),
            get_location_metres(aLocation,-12,5),
            get_location_metres(aLocation,-14,4),
            get_location_metres(aLocation,-13,2),
            get_location_metres(aLocation,-12,1),
            get_location_metres(aLocation,-11,1),
            get_location_metres(aLocation,-8,0),
            get_location_metres(aLocation,-15,0),
            # 目標WP到達確認用
            get_location_metres(aLocation,-15,0),
        ]
    }


    cmds = vehicle.commands
    print(" Clear any existing commands")
    cmds.clear() 
    

    print(" Define Alphabet Flight Trajectory")
    print(" Enter 'A' or 'B'.  (other characters are under implementation...)")
     
    
    Command_Alph = input()

    for x in Alphabet_Mission_Plan[Command_Alph]:
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, x.lat, x.lon, 20))
    # #add dummy waypoint "Last Number" at point "Post Last Number" (lets us know when have reached destination)
    # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point9.lat, point9.lon, 20))    

    print(" Upload new commands to vehicle")
    cmds.upload()

# 機体と接続
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True,timeout=60)
# ミッションの生成
adds_alphabet_mission(vehicle.location.global_frame,50)

####################
# Arm->目標高度到達
####################
while not vehicle.is_armable:
    print ("Waiting for vehicle to initialize...")
    time.sleep(1)

print ("Arming motors")

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

targetAltitude = 20
print("Take Off")
vehicle.simple_takeoff(targetAltitude)
while True:
    print("Altitude:",vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= targetAltitude *0.95:
        print("Reached target altitude")
        break
    time.sleep(1)


####################
# ミッションの開始
####################

# Waypointの数を取得
Number_of_WP = len(vehicle.commands)
print("Alphabet Flight is started. ")
vehicle.mode = VehicleMode("AUTO")

while(True):
    # Waypointをすべて実行するまで待機
    if vehicle.commands.next == Number_of_WP:
        break

print("Alphabet Flight is completed. ")

# 着陸
vehicle.mode = VehicleMode( 'LAND' )
print("Landing...")
