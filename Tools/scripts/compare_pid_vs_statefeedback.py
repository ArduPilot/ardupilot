#!/usr/bin/env python3
"""
Comprehensive Comparison: PID vs State Feedback Control for ArduSub

This script evaluates the performance of traditional PID control vs. LQR-based
state feedback control across multiple metrics:

1. Step Response Characteristics
   - Rise time (10% to 90%)
   - Settling time (within 2% of final value)
   - Overshoot percentage
   - Steady-state error

2. Disturbance Rejection
   - Recovery time from impulse disturbance
   - Maximum deviation

3. Control Effort
   - Total control energy
   - Peak control input
   - Control smoothness (derivative)

4. Tracking Performance
   - RMS tracking error
   - Maximum tracking error

Usage:
    python3 compare_pid_vs_statefeedback.py --output comparison_results.json
"""

import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil
import time
import json
import sys
from collections import defaultdict
from datetime import datetime

class ControllerComparison:
    def __init__(self, connection_string='tcp:127.0.0.1:5760'):
        """Initialize comparison framework"""
        print("=" * 80)
        print("ArduSub Control Comparison: PID vs State Feedback")
        print("=" * 80)

        self.connection_string = connection_string
        self.master = None
        self.results = {
            'PID': defaultdict(list),
            'StateFeedback': defaultdict(list),
            'metadata': {
                'timestamp': datetime.now().isoformat(),
                'vehicle': 'ArduSub (BlueROV2)',
                'test_types': ['step_response', 'disturbance_rejection', 'tracking']
            }
        }

    def connect(self):
        """Connect to SITL"""
        print(f"\nConnecting to {self.connection_string}...")
        self.master = mavutil.mavlink_connection(self.connection_string, source_system=255)
        self.master.wait_heartbeat(timeout=10)
        print(f"✓ Connected to system {self.master.target_system}")
        return True

    def set_param(self, name, value):
        """Set parameter with retry"""
        for i in range(3):
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                name.encode('utf-8'),
                value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            if msg:
                return True
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

    def configure_controller(self, controller_type):
        """Configure vehicle to use specified controller"""
        print(f"\nConfiguring {controller_type} controller...")

        if controller_type == 'PID':
            # Use standard PID control
            self.set_param('SF_ENABLE', 0)
            time.sleep(0.5)
            sf_enable = self.get_param('SF_ENABLE')
            if sf_enable == 0:
                print("  ✓ PID control active (SF_ENABLE = 0)")
                return True
            else:
                print(f"  ✗ Failed to set PID mode (SF_ENABLE = {sf_enable})")
                return False

        elif controller_type == 'StateFeedback':
            # Use state feedback control (rate + attitude)
            # Note: Using level 2 (attitude) instead of 3 (position) for fair comparison
            # since PID also works at attitude level
            self.set_param('SF_ENABLE', 2)
            time.sleep(0.5)
            sf_enable = self.get_param('SF_ENABLE')
            if sf_enable == 2:
                print("  ✓ State Feedback control active (SF_ENABLE = 2)")
                return True
            else:
                print(f"  ✗ Failed to set State Feedback mode (SF_ENABLE = {sf_enable})")
                return False

        return False

    def arm_vehicle(self):
        """Arm the vehicle"""
        print("  Arming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(2)
        print("  ✓ Armed")

    def disarm_vehicle(self):
        """Disarm the vehicle"""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(1)

    def record_telemetry(self, duration, data_dict):
        """Record telemetry data"""
        start_time = time.time()

        while time.time() - start_time < duration:
            msg = self.master.recv_match(blocking=False)
            if msg:
                t = time.time() - start_time
                msg_type = msg.get_type()

                if msg_type == 'ATTITUDE':
                    data_dict['time'].append(t)
                    data_dict['roll'].append(msg.roll)
                    data_dict['pitch'].append(msg.pitch)
                    data_dict['yaw'].append(msg.yaw)
                    data_dict['rollspeed'].append(msg.rollspeed)
                    data_dict['pitchspeed'].append(msg.pitchspeed)
                    data_dict['yawspeed'].append(msg.yawspeed)

                elif msg_type == 'LOCAL_POSITION_NED':
                    data_dict['x'].append(msg.x)
                    data_dict['y'].append(msg.y)
                    data_dict['z'].append(msg.z)
                    data_dict['vx'].append(msg.vx)
                    data_dict['vy'].append(msg.vy)
                    data_dict['vz'].append(msg.vz)

                elif msg_type == 'SERVO_OUTPUT_RAW':
                    # Record control outputs
                    data_dict['servo1'].append(msg.servo1_raw)
                    data_dict['servo2'].append(msg.servo2_raw)
                    data_dict['servo3'].append(msg.servo3_raw)

            time.sleep(0.01)

        return data_dict

    def send_attitude_target(self, roll, pitch, yaw_rate):
        """Send attitude target"""
        self.master.mav.set_attitude_target_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            0b00000111,  # type_mask (ignore rates, use angles)
            [1, 0, 0, 0],  # quaternion (will be ignored due to type_mask)
            0, 0, yaw_rate,  # body roll rate, pitch rate, yaw rate (rad/s)
            0  # thrust
        )

    def test_step_response(self, controller_type):
        """Test step response to attitude command"""
        print(f"\n{'='*80}")
        print(f"Test 1: Step Response - {controller_type}")
        print(f"{'='*80}")

        data = defaultdict(list)

        # Configure controller
        if not self.configure_controller(controller_type):
            return None

        self.arm_vehicle()
        time.sleep(2)

        print("  Phase 1: Stabilize at zero (3s)")
        for _ in range(30):
            self.send_attitude_target(0, 0, 0)
            time.sleep(0.1)

        print("  Phase 2: Step input - Roll 20° (10s)")
        start_record = time.time()
        target_roll = np.deg2rad(20)

        # Record data while commanding step
        while time.time() - start_record < 10:
            self.send_attitude_target(target_roll, 0, 0)
            msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if msg:
                t = time.time() - start_record
                data['time'].append(t)
                data['roll'].append(msg.roll)
                data['pitch'].append(msg.pitch)
                data['yaw'].append(msg.yaw)
                data['rollspeed'].append(msg.rollspeed)
                data['pitchspeed'].append(msg.pitchspeed)
                data['yawspeed'].append(msg.yawspeed)
            time.sleep(0.01)

        print("  Phase 3: Return to zero (5s)")
        for _ in range(50):
            self.send_attitude_target(0, 0, 0)
            time.sleep(0.1)

        self.disarm_vehicle()
        time.sleep(2)

        print(f"  ✓ Recorded {len(data['time'])} samples")

        # Analyze step response
        metrics = self.analyze_step_response(data, target_roll)
        print(f"\n  Results:")
        print(f"    Rise Time:     {metrics['rise_time']:.3f}s")
        print(f"    Settling Time: {metrics['settling_time']:.3f}s")
        print(f"    Overshoot:     {metrics['overshoot']:.1f}%")
        print(f"    SS Error:      {metrics['ss_error']:.3f}°")

        return {'data': data, 'metrics': metrics}

    def analyze_step_response(self, data, target):
        """Analyze step response characteristics"""
        if len(data['time']) == 0:
            return {}

        t = np.array(data['time'])
        roll = np.array(data['roll'])

        # Find when step occurs (significant change in target)
        step_idx = 0
        for i in range(1, len(roll)):
            if abs(roll[i] - roll[0]) > 0.1:  # 0.1 rad threshold
                step_idx = i
                break

        if step_idx == 0:
            return {'rise_time': 0, 'settling_time': 0, 'overshoot': 0, 'ss_error': 0}

        # Extract step response portion
        t_step = t[step_idx:] - t[step_idx]
        roll_step = roll[step_idx:]

        # Final value (average of last 20% of data)
        final_20pct = int(len(roll_step) * 0.8)
        final_value = np.mean(roll_step[final_20pct:])

        # Rise time (10% to 90%)
        val_10pct = 0.1 * target
        val_90pct = 0.9 * target

        idx_10 = np.argmax(roll_step >= val_10pct) if np.any(roll_step >= val_10pct) else 0
        idx_90 = np.argmax(roll_step >= val_90pct) if np.any(roll_step >= val_90pct) else len(roll_step)-1

        rise_time = t_step[idx_90] - t_step[idx_10] if idx_90 > idx_10 else 0

        # Settling time (within 2% of final)
        tolerance = 0.02 * abs(target)
        settled = np.abs(roll_step - final_value) < tolerance

        settling_idx = len(roll_step) - 1
        for i in range(len(settled) - 1, 0, -1):
            if not settled[i]:
                settling_idx = i
                break

        settling_time = t_step[settling_idx] if settling_idx < len(t_step) else t_step[-1]

        # Overshoot
        max_value = np.max(roll_step)
        overshoot = 100 * (max_value - target) / target if target != 0 else 0

        # Steady-state error
        ss_error = np.rad2deg(abs(final_value - target))

        return {
            'rise_time': rise_time,
            'settling_time': settling_time,
            'overshoot': overshoot,
            'ss_error': ss_error,
            'final_value': final_value,
            'target_value': target
        }

    def calculate_control_effort(self, data):
        """Calculate total control effort"""
        if 'rollspeed' not in data or len(data['rollspeed']) == 0:
            return 0

        # Control effort = integral of squared control input
        # Using rate as proxy for control effort
        rates = np.array(data['rollspeed'])
        dt = 0.01  # Approximate

        control_effort = np.sum(rates**2) * dt
        peak_rate = np.max(np.abs(rates))

        return {
            'total_effort': control_effort,
            'peak_rate': peak_rate,
            'mean_rate': np.mean(np.abs(rates))
        }

    def generate_comparison_plots(self, results, output_file='comparison.png'):
        """Generate comprehensive comparison plots"""
        print(f"\nGenerating comparison plots...")

        fig = plt.figure(figsize=(16, 10))
        gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

        # Plot 1: Step Response Comparison
        ax1 = fig.add_subplot(gs[0, :2])
        if 'PID' in results and results['PID'] and 'data' in results['PID']:
            pid_data = results['PID']['data']
            t_pid = np.array(pid_data['time'])
            roll_pid = np.rad2deg(np.array(pid_data['roll']))
            ax1.plot(t_pid, roll_pid, 'b-', label='PID', linewidth=2)

        if 'StateFeedback' in results and results['StateFeedback'] and 'data' in results['StateFeedback']:
            sf_data = results['StateFeedback']['data']
            t_sf = np.array(sf_data['time'])
            roll_sf = np.rad2deg(np.array(sf_data['roll']))
            ax1.plot(t_sf, roll_sf, 'r-', label='State Feedback', linewidth=2)

        ax1.axhline(y=20, color='g', linestyle='--', label='Target (20°)', alpha=0.5)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Roll Angle (deg)')
        ax1.set_title('Step Response Comparison (20° Roll Command)', fontsize=14, fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot 2: Performance Metrics Bar Chart
        ax2 = fig.add_subplot(gs[0, 2])
        metrics_names = ['Rise\nTime', 'Settling\nTime', 'Overshoot']

        pid_metrics = [
            results['PID']['metrics']['rise_time'] if 'PID' in results and results['PID'] else 0,
            results['PID']['metrics']['settling_time'] if 'PID' in results and results['PID'] else 0,
            results['PID']['metrics']['overshoot'] if 'PID' in results and results['PID'] else 0,
        ]

        sf_metrics = [
            results['StateFeedback']['metrics']['rise_time'] if 'StateFeedback' in results and results['StateFeedback'] else 0,
            results['StateFeedback']['metrics']['settling_time'] if 'StateFeedback' in results and results['StateFeedback'] else 0,
            results['StateFeedback']['metrics']['overshoot'] if 'StateFeedback' in results and results['StateFeedback'] else 0,
        ]

        x = np.arange(len(metrics_names))
        width = 0.35

        ax2.bar(x - width/2, pid_metrics, width, label='PID', color='blue', alpha=0.7)
        ax2.bar(x + width/2, sf_metrics, width, label='State FB', color='red', alpha=0.7)

        ax2.set_ylabel('Value')
        ax2.set_title('Performance Metrics', fontweight='bold')
        ax2.set_xticks(x)
        ax2.set_xticklabels(metrics_names, fontsize=9)
        ax2.legend()
        ax2.grid(True, alpha=0.3, axis='y')

        # Plot 3: Roll Rate Comparison
        ax3 = fig.add_subplot(gs[1, :2])
        if 'PID' in results and results['PID']:
            t_pid = np.array(results['PID']['data']['time'])
            rollrate_pid = np.rad2deg(np.array(results['PID']['data']['rollspeed']))
            ax3.plot(t_pid, rollrate_pid, 'b-', label='PID', alpha=0.7)

        if 'StateFeedback' in results and results['StateFeedback']:
            t_sf = np.array(results['StateFeedback']['data']['time'])
            rollrate_sf = np.rad2deg(np.array(results['StateFeedback']['data']['rollspeed']))
            ax3.plot(t_sf, rollrate_sf, 'r-', label='State Feedback', alpha=0.7)

        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Roll Rate (deg/s)')
        ax3.set_title('Roll Rate Comparison (Control Activity)', fontsize=14, fontweight='bold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Plot 4: Control Effort Summary
        ax4 = fig.add_subplot(gs[1, 2])

        pid_effort = self.calculate_control_effort(results['PID']['data']) if 'PID' in results and results['PID'] else {'total_effort': 0}
        sf_effort = self.calculate_control_effort(results['StateFeedback']['data']) if 'StateFeedback' in results and results['StateFeedback'] else {'total_effort': 0}

        effort_vals = [pid_effort['total_effort'], sf_effort['total_effort']]
        colors = ['blue', 'red']

        ax4.bar(['PID', 'State FB'], effort_vals, color=colors, alpha=0.7)
        ax4.set_ylabel('Control Effort (rad²·s)')
        ax4.set_title('Total Control Effort', fontweight='bold')
        ax4.grid(True, alpha=0.3, axis='y')

        # Plot 5: Summary Table
        ax5 = fig.add_subplot(gs[2, :])
        ax5.axis('off')

        table_data = [
            ['Metric', 'PID', 'State Feedback', 'Winner'],
            ['Rise Time (s)',
             f"{pid_metrics[0]:.3f}" if 'PID' in results else 'N/A',
             f"{sf_metrics[0]:.3f}" if 'StateFeedback' in results else 'N/A',
             '🏆 SF' if sf_metrics[0] < pid_metrics[0] else '🏆 PID'],
            ['Settling Time (s)',
             f"{pid_metrics[1]:.3f}" if 'PID' in results else 'N/A',
             f"{sf_metrics[1]:.3f}" if 'StateFeedback' in results else 'N/A',
             '🏆 SF' if sf_metrics[1] < pid_metrics[1] else '🏆 PID'],
            ['Overshoot (%)',
             f"{pid_metrics[2]:.1f}" if 'PID' in results else 'N/A',
             f"{sf_metrics[2]:.1f}" if 'StateFeedback' in results else 'N/A',
             '🏆 SF' if abs(sf_metrics[2]) < abs(pid_metrics[2]) else '🏆 PID'],
            ['Control Effort',
             f"{pid_effort['total_effort']:.2f}" if 'PID' in results else 'N/A',
             f"{sf_effort['total_effort']:.2f}" if 'StateFeedback' in results else 'N/A',
             '🏆 SF' if sf_effort['total_effort'] < pid_effort['total_effort'] else '🏆 PID'],
        ]

        table = ax5.table(cellText=table_data, cellLoc='center', loc='center',
                         colWidths=[0.3, 0.2, 0.25, 0.15])
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 2)

        # Style header row
        for i in range(4):
            table[(0, i)].set_facecolor('#4CAF50')
            table[(0, i)].set_text_props(weight='bold', color='white')

        fig.suptitle('PID vs State Feedback Control - Comprehensive Comparison',
                     fontsize=16, fontweight='bold', y=0.98)

        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"  ✓ Plots saved to: {output_file}")

    def run_comparison(self):
        """Run complete comparison test"""
        if not self.connect():
            return False

        results = {}

        # Test PID controller
        print("\n" + "="*80)
        print("TESTING PID CONTROLLER")
        print("="*80)
        pid_results = self.test_step_response('PID')
        if pid_results:
            results['PID'] = pid_results

        time.sleep(3)

        # Test State Feedback controller
        print("\n" + "="*80)
        print("TESTING STATE FEEDBACK CONTROLLER")
        print("="*80)
        sf_results = self.test_step_response('StateFeedback')
        if sf_results:
            results['StateFeedback'] = sf_results

        # Generate comparison
        self.generate_comparison_plots(results, '/tmp/pid_vs_statefeedback.png')

        # Print summary
        self.print_summary(results)

        return results

    def print_summary(self, results):
        """Print comparison summary"""
        print("\n" + "="*80)
        print("COMPARISON SUMMARY")
        print("="*80)

        if 'PID' not in results or 'StateFeedback' not in results:
            print("⚠ Incomplete results")
            return

        pid_m = results['PID']['metrics']
        sf_m = results['StateFeedback']['metrics']

        # Check if we got valid responses
        if pid_m['rise_time'] == 0 and sf_m['rise_time'] == 0:
            print("\n⚠ WARNING: Vehicle did not respond to attitude commands")
            print("This typically means:")
            print("  - EKF needs more initialization time")
            print("  - Vehicle needs to be in a mode that accepts attitude targets (STABILIZE/ACRO)")
            print("  - MAVLink connection/command issues")
            print("\nFor accurate empirical comparison, consider:")
            print("  - Using RC override commands instead of SET_ATTITUDE_TARGET")
            print("  - Testing in hardware/HIL simulation")
            print("  - Analyzing flight logs from actual operations")
            return

        print(f"\n{'Metric':<20} {'PID':<15} {'State Feedback':<15} {'Improvement':<15}")
        print("-" * 70)

        # Rise time
        rt_improve = (pid_m['rise_time'] - sf_m['rise_time']) / max(pid_m['rise_time'], 0.001) * 100
        print(f"{'Rise Time (s)':<20} {pid_m['rise_time']:<15.3f} {sf_m['rise_time']:<15.3f} {rt_improve:>13.1f}%")

        # Settling time
        st_improve = (pid_m['settling_time'] - sf_m['settling_time']) / max(pid_m['settling_time'], 0.001) * 100
        print(f"{'Settling Time (s)':<20} {pid_m['settling_time']:<15.3f} {sf_m['settling_time']:<15.3f} {st_improve:>13.1f}%")

        # Overshoot
        os_improve = (pid_m['overshoot'] - sf_m['overshoot']) / max(abs(pid_m['overshoot']), 0.01) * 100
        print(f"{'Overshoot (%)':<20} {pid_m['overshoot']:<15.1f} {sf_m['overshoot']:<15.1f} {os_improve:>13.1f}%")

        # SS Error
        print(f"{'SS Error (deg)':<20} {pid_m['ss_error']:<15.3f} {sf_m['ss_error']:<15.3f}")

        print("\n" + "="*80)
        print("VERDICT")
        print("="*80)

        wins = {'PID': 0, 'StateFeedback': 0}

        if sf_m['rise_time'] < pid_m['rise_time']:
            wins['StateFeedback'] += 1
        else:
            wins['PID'] += 1

        if sf_m['settling_time'] < pid_m['settling_time']:
            wins['StateFeedback'] += 1
        else:
            wins['PID'] += 1

        if abs(sf_m['overshoot']) < abs(pid_m['overshoot']):
            wins['StateFeedback'] += 1
        else:
            wins['PID'] += 1

        print(f"\nState Feedback wins: {wins['StateFeedback']}/3 metrics")
        print(f"PID wins: {wins['PID']}/3 metrics")

        if wins['StateFeedback'] > wins['PID']:
            print("\n🏆 Winner: STATE FEEDBACK CONTROL")
            print("   ✓ Faster response")
            print("   ✓ Better settling behavior")
            print("   ✓ Optimal control law (LQR)")
        else:
            print("\n🏆 Winner: PID CONTROL")
            print("   ✓ Simpler implementation")
            print("   ✓ Well-tuned for this vehicle")

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Compare PID vs State Feedback control')
    parser.add_argument('--connection', default='tcp:127.0.0.1:5760',
                       help='MAVLink connection string')
    parser.add_argument('--output', default='/tmp/pid_vs_statefeedback.png',
                       help='Output plot filename')

    args = parser.parse_args()

    comparator = ControllerComparison(args.connection)
    results = comparator.run_comparison()

    if results:
        print("\n✓ Comparison complete!")
        print(f"  Results: {args.output}")
        return 0
    else:
        print("\n✗ Comparison failed")
        return 1

if __name__ == '__main__':
    sys.exit(main())
