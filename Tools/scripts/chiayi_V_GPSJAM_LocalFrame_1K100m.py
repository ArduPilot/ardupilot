from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

takeoff_altitude = 100.0
gps_waypoints = [
    LocationGlobalRelative(23.457822, 120.276008, 0.0),
    LocationGlobalRelative(23.463822, 120.282008, 100.0),
]

def arm_and_takeoff(takeoff_altitude):
    print("起飛中...")
    while not vehicle.is_armable:
        print(" 等待初始化...")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" 等待解鎖中...")
        time.sleep(1)
    vehicle.simple_takeoff(takeoff_altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" 高度：{alt:.2f} m")
        if alt >= takeoff_altitude * 0.95:
            print(" 到達目標高度")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * 1.113195e5)**2 + (dlong * 1.113195e5)**2)

def fly_to_waypoint(location, tolerance=1.5):
    print(f"飛到：{location.lat}, {location.lon}, {location.alt}")
    vehicle.simple_goto(location)
    while True:
        current_location = vehicle.location.global_relative_frame
        dist = get_distance_metres(current_location, location)
        print(f" 距離目標約 {dist:.1f} 公尺")
        if dist <= tolerance:
            print(" 到達航點")
            break
        time.sleep(2)

def condition_yaw(heading, relative=True):
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        0,
        1,
        is_relative,
        0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def wait_for_yaw(target_yaw, tolerance=1):
    while True:
        current_yaw = vehicle.heading
        diff = abs(current_yaw - target_yaw) % 360
        diff = diff if diff <= 180 else 360 - diff
        print(f"目前機頭: {current_yaw}°, 目標機頭: {target_yaw}°, 差距: {diff}°")
        if diff <= tolerance:
            break
        time.sleep(5)

def send_local_velocity_to_target(target_location, speed=5.0):
    """
    使用 Local NED Frame (EKF) 控制無人機以速度方式飛回 target_location。
    """
    print("以 LOCAL NED 速度控制飛回第一個航點...")

    # 取得當前位置
    current_location = vehicle.location.global_relative_frame
    dist_m = get_distance_metres(current_location, target_location)
    print(f" ➤ 計算距離目標約 {dist_m:.2f} 公尺")

    # 使用速度估算所需時間
    duration = int(dist_m / speed) + 1

    # 計算方向向量
    dlat = target_location.lat - current_location.lat
    dlong = target_location.lon - current_location.lon
    norm = math.sqrt(dlat**2 + dlong**2)
    if norm == 0:
        print(" ✅ 已在目標點，無需移動。")
        return
    unit_lat = dlat / norm
    unit_lon = dlong / norm

    vx = unit_lat * speed * 1.113195e5
    vy = unit_lon * speed * 1.113195e5
    vz = 0

    # 傳送 MAVLink 速度控制訊息
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # 只啟用速度控制
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0,
        0
    )

    for _ in range(duration):
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(1)

    print(" ✅ 已嘗試飛回第一個航點。")

# ================= 主程式開始 ====================

print("連線中...")
vehicle = connect('udp:127.0.0.1:14551', wait_ready=True,
                  source_system=200, source_component=0)

arm_and_takeoff(takeoff_altitude)

for waypoint in gps_waypoints:
    fly_to_waypoint(waypoint)

print("啟用 GPS 干擾模擬 (SIM_GPS1_JAM=1)...")
vehicle.parameters['SIM_GPS1_JAM'] = 1
time.sleep(1)

# 等待 GPS fix_type 降低
print("檢查 GPS 是否失效中...")
timeout = 15
for i in range(timeout):
    fix = vehicle.gps_0.fix_type
    print(f" GPS fix_type = {fix}")
    if fix <= 1:
        print(" ✅ GPS 信號已失效")
        break
    time.sleep(1)
else:
    print(" ⚠️ GPS 干擾可能未生效，fix_type 未降到 1 以下")

print("開始轉機頭180度...")
condition_yaw(180, relative=True)
wait_for_yaw(222)

# ===== 此處改為使用 Local Frame 飛回第一點 =====
print("完成轉向，開始LOCAL FRAME位移（返回第一點）...")
send_local_velocity_to_target(gps_waypoints[0], speed=5.0)

print("開始降落...")
vehicle.mode = VehicleMode("LAND")
while vehicle.location.global_relative_frame.alt > 0.1:
    print(f" 高度：{vehicle.location.global_relative_frame.alt:.2f}")
    time.sleep(1)

print("關閉 GPS 干擾模擬 (SIM_GPS1_JAM=0)...")
vehicle.parameters['SIM_GPS1_JAM'] = 0

print("完成任務並關閉連線")
vehicle.close()

