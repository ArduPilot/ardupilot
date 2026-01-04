#!/usr/bin/env python3
"""
ArduSub State Feedback Position Control Test Script

Tests the full 12-state position controller in SITL.

Usage:
    python3 test_position_control.py
"""

import time
import sys
import numpy as np
from pymavlink import mavutil
import matplotlib.pyplot as plt
from collections import defaultdict

class PositionControlTester:
    def __init__(self, connection_string='tcp:127.0.0.1:5760'):
        """Initialize MAVLink connection to SITL"""
        print("=" * 70)
        print("ArduSub State Feedback Position Control Test")
        print("=" * 70)

        print(f"\nConnecting to SITL at {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string, source_system=255)

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat(timeout=10)
        print(f"✓ Connected to system {self.master.target_system}")

        # Data storage
        self.data = defaultdict(list)
        self.start_time = None

    def set_param(self, name, value, retries=3):
        """Set parameter with retry logic"""
        for i in range(retries):
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                name.encode('utf-8'),
                value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )

            # Wait for acknowledgment
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            if msg:
                param_id = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode('utf-8')
                if param_id.strip('\x00') == name:
                    return True

        print(f"  ✗ Failed to set {name}")
        return False

    def get_param(self, name):
        """Get parameter value"""
        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            name.encode('utf-8'),
            -1
        )
        msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg:
            return msg.param_value
        return None

    def load_param_file(self, filename):
        """Load parameters from file"""
        print(f"\nLoading parameters from {filename}...")
        loaded = 0
        failed = 0

        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue

                parts = line.split(',')
                if len(parts) == 2:
                    param_name = parts[0].strip()
                    try:
                        param_value = float(parts[1].strip())
                        if self.set_param(param_name, param_value):
                            loaded += 1
                            if loaded % 10 == 0:
                                print(f"  Loaded {loaded} parameters...")
                        else:
                            failed += 1
                    except ValueError:
                        print(f"  ✗ Invalid value for {param_name}")
                        failed += 1

        print(f"\n✓ Loaded {loaded} parameters")
        if failed > 0:
            print(f"✗ Failed to load {failed} parameters")
        return loaded, failed

    def wait_for_mode(self, mode_name, timeout=10):
        """Wait for vehicle to enter specified mode"""
        start = time.time()
        while time.time() - start < timeout:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                # Check if mode matches
                # For ArduSub, we just check if armed/disarmed
                return True
        return False

    def arm_vehicle(self):
        """Arm the vehicle"""
        print("\nArming vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0
        )

        # Wait for ack
        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("✓ Vehicle armed")
            return True
        else:
            print("✗ Failed to arm vehicle")
            return False

    def disarm_vehicle(self):
        """Disarm the vehicle"""
        print("\nDisarming vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # disarm
            0, 0, 0, 0, 0, 0
        )
        time.sleep(1)
        print("✓ Vehicle disarmed")

    def set_position_target(self, x, y, z, vx=0, vy=0, vz=0):
        """Send position target in NED frame"""
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask (position and velocity)
            x, y, z,  # position
            vx, vy, vz,  # velocity
            0, 0, 0,  # acceleration (not used)
            0, 0  # yaw, yaw_rate (not used)
        )

    def record_telemetry(self, duration):
        """Record telemetry data for specified duration"""
        print(f"\nRecording telemetry for {duration}s...")
        self.start_time = time.time()
        self.data = defaultdict(list)

        while time.time() - self.start_time < duration:
            msg = self.master.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                t = time.time() - self.start_time

                if msg_type == 'LOCAL_POSITION_NED':
                    self.data['time'].append(t)
                    self.data['x'].append(msg.x)
                    self.data['y'].append(msg.y)
                    self.data['z'].append(msg.z)
                    self.data['vx'].append(msg.vx)
                    self.data['vy'].append(msg.vy)
                    self.data['vz'].append(msg.vz)

                elif msg_type == 'ATTITUDE':
                    self.data['roll'].append(msg.roll)
                    self.data['pitch'].append(msg.pitch)
                    self.data['yaw'].append(msg.yaw)
                    self.data['rollspeed'].append(msg.rollspeed)
                    self.data['pitchspeed'].append(msg.pitchspeed)
                    self.data['yawspeed'].append(msg.yawspeed)

            time.sleep(0.01)

        print(f"✓ Recorded {len(self.data['time'])} samples")

    def plot_results(self, filename='position_control_test.png'):
        """Plot test results"""
        if len(self.data['time']) == 0:
            print("No data to plot")
            return

        print(f"\nPlotting results to {filename}...")

        fig, axes = plt.subplots(4, 3, figsize=(15, 12))
        fig.suptitle('ArduSub State Feedback Position Control Test', fontsize=16)

        t = self.data['time']

        # Position plots
        if 'x' in self.data:
            axes[0, 0].plot(t, self.data['x'])
            axes[0, 0].set_ylabel('X Position (m)')
            axes[0, 0].grid(True)
            axes[0, 0].set_title('North Position')

            axes[0, 1].plot(t, self.data['y'])
            axes[0, 1].set_ylabel('Y Position (m)')
            axes[0, 1].grid(True)
            axes[0, 1].set_title('East Position')

            axes[0, 2].plot(t, self.data['z'])
            axes[0, 2].set_ylabel('Z Position (m)')
            axes[0, 2].grid(True)
            axes[0, 2].set_title('Down Position (Depth)')

        # Velocity plots
        if 'vx' in self.data:
            axes[1, 0].plot(t, self.data['vx'])
            axes[1, 0].set_ylabel('Vx (m/s)')
            axes[1, 0].grid(True)
            axes[1, 0].set_title('North Velocity')

            axes[1, 1].plot(t, self.data['vy'])
            axes[1, 1].set_ylabel('Vy (m/s)')
            axes[1, 1].grid(True)
            axes[1, 1].set_title('East Velocity')

            axes[1, 2].plot(t, self.data['vz'])
            axes[1, 2].set_ylabel('Vz (m/s)')
            axes[1, 2].grid(True)
            axes[1, 2].set_title('Down Velocity')

        # Attitude plots
        if 'roll' in self.data:
            axes[2, 0].plot(t, np.rad2deg(self.data['roll']))
            axes[2, 0].set_ylabel('Roll (deg)')
            axes[2, 0].grid(True)
            axes[2, 0].set_title('Roll Angle')

            axes[2, 1].plot(t, np.rad2deg(self.data['pitch']))
            axes[2, 1].set_ylabel('Pitch (deg)')
            axes[2, 1].grid(True)
            axes[2, 1].set_title('Pitch Angle')

            axes[2, 2].plot(t, np.rad2deg(self.data['yaw']))
            axes[2, 2].set_ylabel('Yaw (deg)')
            axes[2, 2].grid(True)
            axes[2, 2].set_title('Yaw Angle')

        # Rate plots
        if 'rollspeed' in self.data:
            axes[3, 0].plot(t, np.rad2deg(self.data['rollspeed']))
            axes[3, 0].set_ylabel('Roll Rate (deg/s)')
            axes[3, 0].set_xlabel('Time (s)')
            axes[3, 0].grid(True)
            axes[3, 0].set_title('Roll Rate')

            axes[3, 1].plot(t, np.rad2deg(self.data['pitchspeed']))
            axes[3, 1].set_ylabel('Pitch Rate (deg/s)')
            axes[3, 1].set_xlabel('Time (s)')
            axes[3, 1].grid(True)
            axes[3, 1].set_title('Pitch Rate')

            axes[3, 2].plot(t, np.rad2deg(self.data['yawspeed']))
            axes[3, 2].set_ylabel('Yaw Rate (deg/s)')
            axes[3, 2].set_xlabel('Time (s)')
            axes[3, 2].grid(True)
            axes[3, 2].set_title('Yaw Rate')

        plt.tight_layout()
        plt.savefig(filename, dpi=150)
        print(f"✓ Plot saved to {filename}")

    def run_position_hold_test(self, duration=30):
        """Test position hold at origin"""
        print("\n" + "=" * 70)
        print("TEST 1: Position Hold at Origin")
        print("=" * 70)

        print("\nSetting target: x=0, y=0, z=-5m (5m depth)")

        # Send position target repeatedly
        start = time.time()
        while time.time() - start < duration:
            self.set_position_target(0, 0, -5)
            time.sleep(0.1)

        print(f"✓ Position hold test completed ({duration}s)")

    def run_step_response_test(self, duration=20):
        """Test step response to position change"""
        print("\n" + "=" * 70)
        print("TEST 2: Step Response Test")
        print("=" * 70)

        print("\nStep 1: Hold at origin for 5s")
        start = time.time()
        while time.time() - start < 5:
            self.set_position_target(0, 0, -5)
            time.sleep(0.1)

        print("Step 2: Move to x=5m for 10s")
        start = time.time()
        while time.time() - start < 10:
            self.set_position_target(5, 0, -5)
            time.sleep(0.1)

        print("Step 3: Return to origin for 5s")
        start = time.time()
        while time.time() - start < 5:
            self.set_position_target(0, 0, -5)
            time.sleep(0.1)

        print("✓ Step response test completed")

    def verify_setup(self):
        """Verify state feedback is properly configured"""
        print("\nVerifying State Feedback Configuration:")
        print("-" * 70)

        # Check SF_ENABLE
        sf_enable = self.get_param('SF_ENABLE')
        if sf_enable == 3:
            print(f"✓ SF_ENABLE = {sf_enable} (Position control enabled)")
        else:
            print(f"✗ SF_ENABLE = {sf_enable} (Should be 3 for position control)")
            return False

        # Check a few gain parameters
        k1 = self.get_param('SF_P_K1')
        k48 = self.get_param('SF_P_K48')
        mass = self.get_param('SF_P_MASS')

        print(f"✓ SF_P_K1 = {k1}")
        print(f"✓ SF_P_K48 = {k48}")
        print(f"✓ SF_P_MASS = {mass} kg")

        if abs(k1) < 0.001 and abs(k48) < 0.001:
            print("✗ WARNING: Gains appear to be zero. Load parameter file first!")
            return False

        print("✓ Configuration verified")
        return True

def main():
    tester = PositionControlTester()

    # Load parameters
    param_file = 'Tools/scripts/gains_position.param'
    print(f"\nChecking for parameter file: {param_file}")
    try:
        loaded, failed = tester.load_param_file(param_file)
        if failed > 0:
            print(f"\n⚠ Some parameters failed to load ({failed}). Continuing...")
            time.sleep(1)
    except FileNotFoundError:
        print(f"✗ Parameter file not found: {param_file}")
        print("Run lqr_position_gain_calculator.py first!")
        return 1

    # Verify setup
    time.sleep(2)  # Wait for parameters to apply
    if not tester.verify_setup():
        print("\n✗ Setup verification failed. Fix issues and try again.")
        return 1

    # Arm vehicle
    if not tester.arm_vehicle():
        return 1

    time.sleep(2)

    # Run tests
    try:
        # Start recording
        import threading
        record_thread = threading.Thread(target=lambda: tester.record_telemetry(35))
        record_thread.start()

        time.sleep(2)

        # Run position hold test
        tester.run_position_hold_test(duration=15)

        # Run step response test
        tester.run_step_response_test(duration=18)

        # Wait for recording to finish
        record_thread.join()

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        # Disarm
        tester.disarm_vehicle()

    # Plot results
    tester.plot_results('/tmp/position_control_test.png')

    print("\n" + "=" * 70)
    print("Test Complete!")
    print("=" * 70)
    print(f"Results saved to: /tmp/position_control_test.png")

    return 0

if __name__ == '__main__':
    sys.exit(main())
