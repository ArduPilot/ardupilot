#!/usr/bin/env python3
"""
gps_interference_demo.py
-------------------------------------------
ArduPilot SITL 自動飛行腳本

步驟：
1. 起飛 50 m
2. 北移 100 m
3. 東移 100 m
4. 關閉 GPS (模擬干擾)
5. 飛回起點
6. 開啟 GPS、降落
"""
from dronekit import connect, VehicleMode, LocationGlobalRelative
from math import cos, radians, sqrt
import time, argparse, sys

# ---------- 參數 ----------
ALT_TAKEOFF = 50      # 起飛高度（m）
LEG_NORTH   = 100     # 北移距離（m）
LEG_EAST    = 100     # 東移距離（m）
SPEED       = 5       # m/s 巡航速度
GPS_DISABLE_TIME = 15 # 關閉 GPS 後持續時間（s）
# -------------------------

def log(msg): print(f"[SCRIPT] {msg}")

def meters_to_lat(m):  return  m / 111320.0        # 1° 緯度 ≈ 111.32km
def meters_to_lon(m, lat_deg): return m / (111320.0 * cos(radians(lat_deg)))

def distance_meters(a, b):
    dlat = (a.lat - b.lat) * 111320.0
    dlon = (a.lon - b.lon) * 111320.0 * cos(radians(a.lat))
    dalt = a.alt - b.alt
    return sqrt(dlat**2 + dlon**2 + dalt**2)

def arm_and_takeoff(vehicle, target_alt):
    log("切 GUIDED")
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        time.sleep(0.5)

    log("解鎖 (arm)")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)

    log(f"起飛至 {target_alt} m")
    vehicle.simple_takeoff(target_alt)
    while True:
        if vehicle.location.global_relative_frame.alt >= target_alt * 0.95:
            break
        time.sleep(0.5)
    log("到達目標高度")

def go_to_rel(vehicle, north_m=0, east_m=0, alt_m=None):
    cur = vehicle.location.global_relative_frame
    tgt_lat = cur.lat + meters_to_lat(north_m)
    tgt_lon = cur.lon + meters_to_lon(east_m, cur.lat)
    tgt_alt = alt_m if alt_m is not None else cur.alt
    tgt = LocationGlobalRelative(tgt_lat, tgt_lon, tgt_alt)
    vehicle.simple_goto(tgt, groundspeed=SPEED)
    # 等待接近 (≤5 m)
    while distance_meters(vehicle.location.global_relative_frame, tgt) > 5:
        time.sleep(0.5)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='127.0.0.1:14550',
                        help='MAVLink connection string')
    args = parser.parse_args()

    log(f"連線到 {args.connect} …")
    vehicle = connect(args.connect, wait_ready=True)

    home = vehicle.location.global_relative_frame
    log(f"Home 位置：{home.lat:.6f}, {home.lon:.6f}")

    # 1) 起飛
    arm_and_takeoff(vehicle, ALT_TAKEOFF)

    # 2) 北移
    log("向北飛 100 m …")
    go_to_rel(vehicle, north_m=LEG_NORTH)

    # 3) 東移
    log("向東飛 100 m …")
    go_to_rel(vehicle, east_m=LEG_EAST)

    # 4) 關閉 GPS 模擬
    log("關閉 GPS，模擬干擾 …")
    vehicle.parameters['SIM_GPS_ENABLE'] = 0
    time.sleep(GPS_DISABLE_TIME)

    # 5) 飛回起點（仍在 GPS 關閉狀態，依慣性導航）
    log("飛回起點 …")
    vehicle.simple_goto(home, groundspeed=SPEED)
    while distance_meters(vehicle.location.global_relative_frame, home) > 5:
        time.sleep(0.5)

    # 重新開啟 GPS
    vehicle.parameters['SIM_GPS_DISABLE'] = 0
    log("GPS 已重新啟用")

    # 6) 降落
    log("切 LAND")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        time.sleep(0.5)
    log("降落完成並解除鎖定")

    vehicle.close()
    log("腳本結束")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用戶中斷，關閉腳本")
        sys.exit(0)

