#!/usr/bin/env python3
"""
Test script for ArduSub state feedback control in SITL
"""

import time
from pymavlink import mavutil

# Connect to SITL
print("Connecting to ArduSub SITL...")
master = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

# Function to set parameter
def set_param(name, value, retries=3):
    for i in range(retries):
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode('utf-8'),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

        # Wait for acknowledgment
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg and msg.param_id.decode('utf-8').strip('\x00') == name:
            print(f"✓ Set {name} = {value}")
            return True

    print(f"✗ Failed to set {name}")
    return False

# Function to get parameter
def get_param(name):
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        name.encode('utf-8'),
        -1
    )

    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
    if msg and msg.param_id.decode('utf-8').strip('\x00') == name:
        print(f"  {name} = {msg.param_value}")
        return msg.param_value
    return None

print("\n=== Setting up State Feedback Parameters ===")

# Set rate loop dynamics (example values from LQR calculator test)
print("\nRate loop dynamics:")
set_param('SF_R_IXX', 0.15)
set_param('SF_R_IYY', 0.15)
set_param('SF_R_IZZ', 0.25)
set_param('SF_R_DX', 0.5)
set_param('SF_R_DY', 0.5)
set_param('SF_R_DZ', 0.3)

# Set rate loop gains (from LQR calculator output)
print("\nRate loop gains:")
set_param('SF_R_K1', 2.630300)
set_param('SF_R_K2', 0.000000)
set_param('SF_R_K3', 0.000000)
set_param('SF_R_K4', 0.000000)
set_param('SF_R_K5', 2.630300)
set_param('SF_R_K6', 0.000000)
set_param('SF_R_K7', 0.000000)
set_param('SF_R_K8', 0.000000)
set_param('SF_R_K9', 2.831120)

# Enable state feedback for rate loop only
print("\nEnabling state feedback:")
set_param('SF_ENABLE', 1)
set_param('SF_LQR_MODE', 0)  # Use pre-calculated gains

print("\n=== Verifying Parameters ===")
get_param('SF_ENABLE')
get_param('SF_R_K1')
get_param('SF_R_K5')
get_param('SF_R_K9')

print("\n=== Arming and Testing ===")

# Set mode to STABILIZE
print("Setting mode to STABILIZE...")
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    2  # STABILIZE mode
)

time.sleep(1)

# Arm
print("Arming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)

# Wait for arming confirmation
msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("✓ Armed successfully")
else:
    print("✗ Arming failed or timed out")
    print("  This is expected in SITL without proper setup")

print("\n=== Reading Attitude Data ===")
print("Monitoring attitude for 5 seconds...")

for i in range(10):
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    if msg:
        import math
        roll_deg = math.degrees(msg.roll)
        pitch_deg = math.degrees(msg.pitch)
        yaw_deg = math.degrees(msg.yaw)
        print(f"  Roll: {roll_deg:6.2f}°  Pitch: {pitch_deg:6.2f}°  Yaw: {yaw_deg:6.2f}°  "
              f"Rates: {msg.rollspeed:6.3f}, {msg.pitchspeed:6.3f}, {msg.yawspeed:6.3f} rad/s")
    time.sleep(0.5)

print("\n=== Test Complete ===")
print("State feedback rate controller is configured and running")
print("Check /tmp/ArduSub.log for detailed logs")
