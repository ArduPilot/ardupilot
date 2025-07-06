from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

takeoff_altitude = 50.0
gps_waypoints = [
    LocationGlobalRelative(23.457822, 120.276008, 0.0),
    LocationGlobalRelative(23.458822, 120.277008, 50.0),
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
    return math.sqrt((dlat*1.113195e5)**2 + (dlong*1.113195e5)**2)

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
    """
    控制機頭朝向 (heading 單位：度)
    relative=True 表示相對當前方向旋轉，False表示絕對方位
    """
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,       # confirmation
        heading, # 目標角度 (degrees)
        0,       # 速度 deg/s (0 表示用默認速度)
        1,       # 方向：1順時針，-1逆時針
        is_relative,
        0, 0, 0  # 其他參數保留
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def wait_for_yaw(target_yaw, tolerance=1):
    """
    等待飛機機頭角度接近 target_yaw (度)，容忍度 tolerance 度
    """
    while True:
        current_yaw = vehicle.heading  # vehicle.heading 是0~360度
        diff = abs(current_yaw - target_yaw) % 360
        diff = diff if diff <= 180 else 360 - diff
        print(f"目前機頭: {current_yaw}°, 目標機頭: {target_yaw}°, 差距: {diff}°")
        if diff <= tolerance:
            break
        time.sleep(5)

def send_body_velocity_ned(vx, vy, vz, yaw_rate=0, duration=10):
    """
    以 BODY FRAME（NED）座標系下的速度控制無人機移動。

    vx, vy, vz 為 NED 速度向量 (m/s)，例如 vx=5 表示以每秒5公尺向前移動。
    yaw_rate 單位為度/秒。
    duration 表示持續時間（秒）。
    """
    print(f"以 BODY FRAME 速度移動：VX={vx}m/s, VY={vy}m/s, VZ={vz}m/s，Yaw_rate={yaw_rate}度/秒")
    yaw_rate_rad = math.radians(yaw_rate)

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # BODY NED 座標
        0b0000111111000111,  # type_mask: 只啟用速度控制
        0, 0, 0,  # x, y, z position (忽略)
        vx, vy, vz,  # velocity in m/s
        0, 0, 0,  # acceleration (忽略)
        0,  # yaw (忽略)
        yaw_rate_rad  # yaw rate in rad/s
    )

    for _ in range(duration):
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(1)


print("連線中...")
vehicle = connect('udp:127.0.0.1:14551', wait_ready=True,
                  source_system=200, source_component=0)

arm_and_takeoff(takeoff_altitude)

for waypoint in gps_waypoints:
    fly_to_waypoint(waypoint)


print("啟用 GPS 干擾模擬 (SIM_GPS1_JAM=1)...")
vehicle.parameters['SIM_GPS1_JAM'] = 1
time.sleep(1)

# 等待 GPS fix_type 降低，確認干擾生效
print("檢查 GPS 是否失效中...")
timeout = 15  # 最多等 15 秒
for i in range(timeout):
    fix = vehicle.gps_0.fix_type
    print(f" GPS fix_type = {fix}")
    if fix <= 1:
        print(" ✅ GPS 信號已失效")
        break
    time.sleep(1)
else:
    print(" ⚠️ GPS 干擾可能未生效，fix_type 未降到 1 以下")


# 呼叫範例
print("開始轉機頭180度...")
condition_yaw(180, relative=True)
wait_for_yaw(222)

print("完成轉向，開始BODY FRAME位移（速度控制）...")
send_body_velocity_ned(5, 0, 0, yaw_rate=0, duration=32)

print("開始降落...")
vehicle.mode = VehicleMode("LAND")
while vehicle.location.global_relative_frame.alt > 0.1:
    print(f" 高度：{vehicle.location.global_relative_frame.alt:.2f}")
    time.sleep(1)

print("關閉 GPS 干擾模擬 (SIM_GPS1_JAM=0)...")
vehicle.parameters['SIM_GPS1_JAM'] = 0

print("完成任務並關閉連線")
vehicle.close()

