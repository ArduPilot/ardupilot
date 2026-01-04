#!/usr/bin/env python3
"""
State Feedback Control Evaluation for ArduSub
Tests and compares PID vs State Feedback control performance in SITL

Usage:
    python3 evaluate_state_feedback.py --test-mode [all|rate|attitude]
"""

import time
import argparse
import numpy as np
from pymavlink import mavutil
import matplotlib.pyplot as plt
from collections import defaultdict

class SITLEvaluator:
    def __init__(self, connection_string='tcp:127.0.0.1:5760'):
        """Initialize MAVLink connection to SITL"""
        print(f"Connecting to ArduSub SITL at {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string, source_system=255)

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(f"Heartbeat received from system {self.master.target_system}")

        # Data storage for analysis
        self.test_data = defaultdict(list)
        self.results = {}

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
            if msg and msg.param_id.decode('utf-8').strip('\x00') == name:
                print(f"  ✓ {name} = {value}")
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
        if msg and msg.param_id.decode('utf-8').strip('\x00') == name:
            return msg.param_value
        return None

    def configure_state_feedback_rate(self):
        """Configure rate loop state feedback parameters"""
        print("\n=== Configuring Rate Loop State Feedback ===")

        # System dynamics
        self.set_param('SF_R_IXX', 0.15)
        self.set_param('SF_R_IYY', 0.15)
        self.set_param('SF_R_IZZ', 0.25)
        self.set_param('SF_R_DX', 0.5)
        self.set_param('SF_R_DY', 0.5)
        self.set_param('SF_R_DZ', 0.3)

        # LQR gains (from calculator: Q=[10,10,10], R=[1,1,1])
        self.set_param('SF_R_K1', 2.630300)
        self.set_param('SF_R_K2', 0.0)
        self.set_param('SF_R_K3', 0.0)
        self.set_param('SF_R_K4', 0.0)
        self.set_param('SF_R_K5', 2.630300)
        self.set_param('SF_R_K6', 0.0)
        self.set_param('SF_R_K7', 0.0)
        self.set_param('SF_R_K8', 0.0)
        self.set_param('SF_R_K9', 2.831120)

        self.set_param('SF_LQR_MODE', 0)

    def configure_state_feedback_attitude(self):
        """Configure attitude loop state feedback parameters"""
        print("\n=== Configuring Attitude Loop State Feedback ===")

        # Attitude gains (from calculator: Q=[100,100,100,10,10,10], R=[1,1,1])
        # Approximate values - would be calculated offline
        gains = [
            ('SF_A_K1', 9.74),   ('SF_A_K2', 0.0),   ('SF_A_K3', 0.0),
            ('SF_A_K4', 3.06),   ('SF_A_K5', 0.0),   ('SF_A_K6', 0.0),
            ('SF_A_K7', 0.0),    ('SF_A_K8', 9.74),  ('SF_A_K9', 0.0),
            ('SF_A_K10', 0.0),   ('SF_A_K11', 3.06), ('SF_A_K12', 0.0),
            ('SF_A_K13', 0.0),   ('SF_A_K14', 0.0),  ('SF_A_K15', 9.82),
            ('SF_A_K16', 0.0),   ('SF_A_K17', 0.0),  ('SF_A_K18', 3.53)
        ]

        for name, value in gains:
            self.set_param(name, value)

    def set_mode(self, mode_name):
        """Set flight mode"""
        mode_mapping = {
            'STABILIZE': 2,
            'ALTHOLD': 3,
            'AUTO': 4,
            'GUIDED': 5,
            'MANUAL': 6,
            'POSHOLD': 16
        }

        if mode_name not in mode_mapping:
            print(f"Unknown mode: {mode_name}")
            return False

        mode_id = mode_mapping[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        time.sleep(0.5)
        return True

    def arm(self):
        """Arm the vehicle"""
        print("Arming vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(1)

    def collect_attitude_data(self, duration=10.0, label="test"):
        """Collect attitude data for specified duration"""
        print(f"Collecting data for {duration}s...")

        start_time = time.time()
        data = {
            'time': [],
            'roll': [], 'pitch': [], 'yaw': [],
            'rollspeed': [], 'pitchspeed': [], 'yawspeed': []
        }

        while time.time() - start_time < duration:
            msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
            if msg:
                elapsed = time.time() - start_time
                data['time'].append(elapsed)
                data['roll'].append(np.degrees(msg.roll))
                data['pitch'].append(np.degrees(msg.pitch))
                data['yaw'].append(np.degrees(msg.yaw))
                data['rollspeed'].append(msg.rollspeed)
                data['pitchspeed'].append(msg.pitchspeed)
                data['yawspeed'].append(msg.yawspeed)

        self.test_data[label] = data
        return data

    def analyze_step_response(self, data, axis='roll'):
        """Analyze step response characteristics"""
        values = np.array(data[axis])
        time_data = np.array(data['time'])

        if len(values) < 10:
            return None

        # Find steady state (last 20% of data)
        steady_start = int(len(values) * 0.8)
        steady_state = np.mean(values[steady_start:])

        # Find rise time (10% to 90%)
        target_10 = 0.1 * steady_state
        target_90 = 0.9 * steady_state

        idx_10 = np.where(values >= target_10)[0]
        idx_90 = np.where(values >= target_90)[0]

        rise_time = None
        if len(idx_10) > 0 and len(idx_90) > 0:
            rise_time = time_data[idx_90[0]] - time_data[idx_10[0]]

        # Find overshoot
        peak = np.max(values)
        overshoot = ((peak - steady_state) / steady_state * 100) if steady_state != 0 else 0

        # Calculate settling time (within 5% of steady state)
        tolerance = 0.05 * abs(steady_state)
        settled = np.where(np.abs(values - steady_state) <= tolerance)[0]
        settling_time = time_data[settled[0]] if len(settled) > 0 else None

        # RMS error
        rms_error = np.sqrt(np.mean((values - steady_state)**2))

        return {
            'steady_state': steady_state,
            'rise_time': rise_time,
            'overshoot': overshoot,
            'settling_time': settling_time,
            'rms_error': rms_error,
            'peak': peak
        }

    def test_stabilize_mode(self, control_mode):
        """Test STABILIZE mode performance"""
        print(f"\n=== Testing STABILIZE Mode ({control_mode}) ===")

        # Set control mode
        if control_mode == 'PID':
            self.set_param('SF_ENABLE', 0)
        elif control_mode == 'Rate SF':
            self.set_param('SF_ENABLE', 1)
        elif control_mode == 'Attitude SF':
            self.set_param('SF_ENABLE', 2)

        # Set mode and collect data
        self.set_mode('STABILIZE')
        time.sleep(1)

        # Collect baseline
        label = f'stabilize_{control_mode.lower().replace(" ", "_")}'
        data = self.collect_attitude_data(duration=15.0, label=label)

        # Analyze
        results = {}
        for axis in ['roll', 'pitch', 'yaw']:
            analysis = self.analyze_step_response(data, axis)
            if analysis:
                results[axis] = analysis
                print(f"\n{axis.upper()} Analysis:")
                print(f"  Steady State: {analysis['steady_state']:.2f}°")
                print(f"  Rise Time: {analysis['rise_time']:.3f}s" if analysis['rise_time'] else "  Rise Time: N/A")
                print(f"  Overshoot: {analysis['overshoot']:.2f}%")
                print(f"  RMS Error: {analysis['rms_error']:.3f}°")

        return results

    def generate_comparison_report(self):
        """Generate comparison plots and metrics"""
        print("\n=== Generating Performance Comparison ===")

        # Plot attitude data for all tests
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        axes_names = ['roll', 'pitch', 'yaw']

        for label, data in self.test_data.items():
            for i, axis in enumerate(axes_names):
                axes[i].plot(data['time'], data[axis], label=label, linewidth=2)
                axes[i].set_ylabel(f'{axis.capitalize()} (deg)')
                axes[i].grid(True)
                axes[i].legend()

        axes[-1].set_xlabel('Time (s)')
        fig.suptitle('ArduSub State Feedback vs PID Control Comparison', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig('/tmp/state_feedback_comparison.png', dpi=150)
        print("  Saved comparison plot to /tmp/state_feedback_comparison.png")

    def run_evaluation(self, test_modes=['rate', 'attitude']):
        """Run complete evaluation"""
        print("\n" + "="*60)
        print("ArduSub State Feedback Control Evaluation")
        print("="*60)

        # Configure parameters
        if 'rate' in test_modes or 'attitude' in test_modes:
            self.configure_state_feedback_rate()

        if 'attitude' in test_modes:
            self.configure_state_feedback_attitude()

        # Arm vehicle
        self.arm()

        # Test different control modes
        results = {}

        # Test 1: PID Baseline
        print("\n--- Test 1: PID Baseline ---")
        results['PID'] = self.test_stabilize_mode('PID')
        time.sleep(2)

        # Test 2: Rate State Feedback
        if 'rate' in test_modes:
            print("\n--- Test 2: Rate State Feedback ---")
            results['Rate SF'] = self.test_stabilize_mode('Rate SF')
            time.sleep(2)

        # Test 3: Attitude State Feedback
        if 'attitude' in test_modes:
            print("\n--- Test 3: Attitude State Feedback ---")
            results['Attitude SF'] = self.test_stabilize_mode('Attitude SF')
            time.sleep(2)

        # Generate report
        self.generate_comparison_report()

        # Print summary
        print("\n" + "="*60)
        print("EVALUATION SUMMARY")
        print("="*60)
        print("\nPerformance Metrics (Roll Axis):")
        print(f"{'Control Mode':<20} {'RMS Error':<12} {'Overshoot':<12} {'Rise Time'}")
        print("-" * 60)

        for mode, data in results.items():
            if 'roll' in data:
                roll = data['roll']
                rise_str = f"{roll['rise_time']:.3f}s" if roll['rise_time'] else "N/A"
                print(f"{mode:<20} {roll['rms_error']:<12.3f} {roll['overshoot']:<12.2f}% {rise_str}")

        print("\n✓ Evaluation complete!")
        return results

def main():
    parser = argparse.ArgumentParser(description='Evaluate ArduSub State Feedback Control')
    parser.add_argument('--test-mode', choices=['all', 'rate', 'attitude'], default='all',
                       help='Which controllers to test')
    parser.add_argument('--connection', default='tcp:127.0.0.1:5760',
                       help='MAVLink connection string')

    args = parser.parse_args()

    # Determine test modes
    test_modes = []
    if args.test_mode in ['all', 'rate']:
        test_modes.append('rate')
    if args.test_mode in ['all', 'attitude']:
        test_modes.append('attitude')

    # Run evaluation
    try:
        evaluator = SITLEvaluator(connection_string=args.connection)
        results = evaluator.run_evaluation(test_modes=test_modes)
        return 0
    except KeyboardInterrupt:
        print("\n\nEvaluation interrupted by user")
        return 1
    except Exception as e:
        print(f"\n\nError during evaluation: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    exit(main())
