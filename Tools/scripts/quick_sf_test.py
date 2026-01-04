#!/usr/bin/env python3
"""
Quick validation test for State Feedback Control
Verifies that parameters load correctly and controllers respond
"""

import time
from pymavlink import mavutil
import sys

def test_state_feedback():
    """Quick test of state feedback functionality"""
    print("="*60)
    print("ArduSub State Feedback - Quick Validation Test")
    print("="*60)

    # Connect
    print("\n1. Connecting to SITL...")
    try:
        master = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
        master.wait_heartbeat(timeout=10)
        print(f"   ✓ Connected to system {master.target_system}")
    except Exception as e:
        print(f"   ✗ Connection failed: {e}")
        return False

    def set_param(name, value):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode('utf-8'), value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg and msg.param_id.decode('utf-8').strip('\x00') == name:
            return True
        return False

    def get_param(name):
        master.mav.param_request_read_send(
            master.target_system, master.target_component,
            name.encode('utf-8'), -1
        )
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg:
            return msg.param_value
        return None

    # Test parameter access
    print("\n2. Testing parameter system...")
    params_to_test = [
        ('SF_ENABLE', 0),
        ('SF_LQR_MODE', 0),
        ('SF_R_IXX', 0.15),
        ('SF_R_K1', 3.162)
    ]

    for name, value in params_to_test:
        if set_param(name, value):
            readback = get_param(name)
            if readback is not None and abs(readback - value) < 0.001:
                print(f"   ✓ {name} = {value:.3f} (verified)")
            else:
                print(f"   ✗ {name} verification failed")
                return False
        else:
            print(f"   ✗ Failed to set {name}")
            return False

    # Test mode switching
    print("\n3. Testing control mode switching...")
    for mode, enable_val in [('PID', 0), ('Rate SF', 1), ('Attitude SF', 2)]:
        if set_param('SF_ENABLE', enable_val):
            print(f"   ✓ Switched to {mode} mode (SF_ENABLE={enable_val})")
        else:
            print(f"   ✗ Failed to switch to {mode} mode")
            return False
        time.sleep(0.5)

    # Monitor attitude data
    print("\n4. Monitoring attitude data (5s)...")
    start_time = time.time()
    sample_count = 0

    while time.time() - start_time < 5.0:
        msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if msg:
            sample_count += 1
            if sample_count % 20 == 0:
                print(f"   Roll: {msg.roll*57.3:6.2f}°  Pitch: {msg.pitch*57.3:6.2f}°  " +
                      f"Yaw: {msg.yaw*57.3:6.2f}°")

    if sample_count > 0:
        print(f"   ✓ Received {sample_count} attitude messages")
    else:
        print("   ✗ No attitude data received")
        return False

    # Final check
    print("\n5. Verifying state feedback parameters...")
    final_params = ['SF_R_K1', 'SF_R_K5', 'SF_R_K9', 'SF_A_K1']
    all_ok = True

    for name in final_params:
        val = get_param(name)
        if val is not None:
            print(f"   ✓ {name} = {val:.3f}")
        else:
            print(f"   ✗ {name} not found")
            all_ok = False

    return all_ok

if __name__ == '__main__':
    print("\nStarting quick validation test...")
    print("Make sure SITL is running on tcp:127.0.0.1:5760\n")

    try:
        success = test_state_feedback()

        print("\n" + "="*60)
        if success:
            print("SUCCESS: State Feedback system validated ✓")
            print("="*60)
            print("\nThe system is ready for full evaluation.")
            print("Run: python3 evaluate_state_feedback.py --test-mode all")
            sys.exit(0)
        else:
            print("FAILED: Validation encountered errors ✗")
            print("="*60)
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
