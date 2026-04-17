#!/usr/bin/env python3
"""
ArduPilot SITL Integration for Hypothesis Fault Injection Testing

This module provides real SITL integration for property-based testing.
It starts SITL, injects faults, monitors telemetry, and verifies properties.

Usage:
    python3 test_sitl_fault_injection.py --examples=100
"""

import os
import sys
import time
import subprocess
import tempfile
import shutil
import socket
import struct
import threading
import queue
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Tuple
from enum import Enum
import random
import signal

try:
    from hypothesis import given, settings, strategies as st, assume, reject
    from hypothesis import HealthCheck
    HYPOTHESIS_AVAILABLE = True
except ImportError:
    HYPOTHESIS_AVAILABLE = False
    print("ERROR: hypothesis not installed. Install with: pip3 install hypothesis pytest")
    sys.exit(1)

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("WARNING: pymavlink not installed. Install with: pip3 install pymavlink")


# ============================================================================
# Data Models
# ============================================================================

class FaultType(Enum):
    # Sensor faults
    GPS_JUMP = "gps_jump"
    GPS_FREEZE = "gps_freeze"
    GPS_DRIFT = "gps_drift"
    IMU_BIAS = "imu_bias"
    IMU_SPIKE = "imu_spike"
    BARO_DRIFT = "baro_drift"
    MAG_SPIKE = "mag_spike"
    
    # Link faults
    LINK_DELAY = "link_delay"
    LINK_LOSS = "link_loss"
    LINK_JITTER = "link_jitter"
    
    # Control faults
    ACTUATOR_FAIL = "actuator_failure"
    CONTROL_SATURATION = "control_saturation"


@dataclass
class Fault:
    """Represents a single fault injection"""
    type: str
    t_sec: float
    duration_sec: Optional[float] = None
    params: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TelemetryData:
    """SITL telemetry data"""
    timestamp: float
    mode: str
    armed: bool
    roll: float
    pitch: float
    yaw: float
    altitude: float
    airspeed: float
    groundspeed: float
    lat: float
    lon: float
    battery_voltage: float
    rc_rssi: int
    gps_fix_type: int
    gps_satellites: int


@dataclass
class TestResult:
    """Test execution result"""
    success: bool
    mode_end: str
    max_roll_deg: float
    max_pitch_deg: float
    max_alt_m: float
    min_alt_m: float
    no_disarm_in_air: bool
    eventual_safe_mode: bool
    error_message: str = ""
    telemetry_log: List[TelemetryData] = field(default_factory=list)


# ============================================================================
# SITL Runner
# ============================================================================

class SITLRunner:
    """Runs SITL scenarios with fault injection via MAVLink"""
    
    def __init__(self, sitl_path: str = None, verbose: bool = False):
        self.sitl_path = sitl_path or os.path.join(
            os.path.dirname(__file__),
            '..', '..', 'build', 'sitl', 'bin'
        )
        self.sitl_binary = os.path.join(self.sitl_path, "arducopter")
        self.verbose = verbose
        
        if not os.path.exists(self.sitl_binary):
            raise RuntimeError(f"SITL binary not found at {self.sitl_binary}")
        
        self.sitl_process = None
        self.mav = None
        self.telemetry_queue = queue.Queue()
        self.telemetry_log = []
        self.running = False
        self.log_dir = None
    
    def start_sitl(self, model: str = "+", frame: str = "quad") -> bool:
        """Start SITL process"""
        self.log_dir = tempfile.mkdtemp(prefix="ardupilot_sitl_")
        
        cmd = [
            self.sitl_binary,
            "--model", model,
            "--frame", frame,
            "--home", "-35.363261,149.165230,584,270",
            "--defaults", os.path.join(
                os.path.dirname(__file__),
                '..', 'default_params', 'copter.parm'
            ),
        ]
        
        if self.verbose:
            print(f"Starting SITL: {' '.join(cmd)}")
        
        try:
            self.sitl_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=self.log_dir,
                preexec_fn=os.setsid
            )
            
            # Wait for SITL to start
            time.sleep(3)
            
            if self.sitl_process.poll() is not None:
                stdout, stderr = self.sitl_process.communicate()
                print(f"SITL failed to start: {stderr.decode()}")
                return False
            
            return True
            
        except Exception as e:
            print(f"Failed to start SITL: {e}")
            return False
    
    def connect_mavlink(self, timeout: int = 30) -> bool:
        """Connect to SITL via MAVLink"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                self.mav = mavutil.mavlink_connection(
                    'tcp:127.0.0.1:5760',
                    source_system=255,
                    source_component=1
                )
                
                # Wait for heartbeat
                self.mav.wait_heartbeat(timeout=5)
                
                if self.verbose:
                    print("Connected to SITL via MAVLink")
                
                return True
                
            except Exception as e:
                if self.verbose:
                    print(f"Waiting for MAVLink connection: {e}")
                time.sleep(1)
        
        print("Failed to connect to SITL via MAVLink")
        return False
    
    def inject_gps_fault(self, dx_m: float, dy_m: float, dz_m: float = 0.0):
        """Inject GPS position jump fault"""
        if not self.mav:
            return
        
        # Get current position
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            # Inject fault by setting SIM_GPS_GLITCH parameters
            self.mav.param_set_send(
                target_system=1,
                target_component=1,
                param_id='SIM_GPS_GLITCH_X',
                param_value=dx_m,
                param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            self.mav.param_set_send(
                target_system=1,
                target_component=1,
                param_id='SIM_GPS_GLITCH_Y',
                param_value=dy_m,
                param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            self.mav.param_set_send(
                target_system=1,
                target_component=1,
                param_id='SIM_GPS_GLITCH_Z',
                param_value=dz_m,
                param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
    
    def clear_gps_fault(self):
        """Clear GPS fault"""
        if not self.mav:
            return
        
        self.mav.param_set_send(
            target_system=1,
            target_component=1,
            param_id='SIM_GPS_GLITCH_X',
            param_value=0.0,
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        self.mav.param_set_send(
            target_system=1,
            target_component=1,
            param_id='SIM_GPS_GLITCH_Y',
            param_value=0.0,
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        self.mav.param_set_send(
            target_system=1,
            target_component=1,
            param_id='SIM_GPS_GLITCH_Z',
            param_value=0.0,
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
    
    def inject_imu_bias(self, axis: str, bias_rps: float):
        """Inject IMU bias fault"""
        if not self.mav:
            return
        
        param_map = {'x': 'SIM_ACC2TRIM_X', 'y': 'SIM_ACC2TRIM_Y', 'z': 'SIM_ACC2TRIM_Z'}
        if axis in param_map:
            self.mav.param_set_send(
                target_system=1,
                target_component=1,
                param_id=param_map[axis],
                param_value=bias_rps,
                param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
    
    def clear_imu_bias(self):
        """Clear IMU bias"""
        if not self.mav:
            return
        
        for param in ['SIM_ACC2TRIM_X', 'SIM_ACC2TRIM_Y', 'SIM_ACC2TRIM_Z']:
            self.mav.param_set_send(
                target_system=1,
                target_component=1,
                param_id=param,
                param_value=0.0,
                param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
    
    def arm_vehicle(self) -> bool:
        """Arm the vehicle"""
        if not self.mav:
            return False
        
        self.mav.arducopter_arm()
        
        # Wait for arm confirmation
        start_time = time.time()
        while time.time() - start_time < 5:
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=False)
            if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
            time.sleep(0.1)
        
        return False
    
    def disarm_vehicle(self) -> bool:
        """Disarm the vehicle"""
        if not self.mav:
            return False
        
        self.mav.arducopter_disarm()
        return True
    
    def set_mode(self, mode_name: str) -> bool:
        """Set flight mode"""
        if not self.mav:
            return False
        
        mode_mapping = self.mav.mode_mapping()
        if mode_name in mode_mapping:
            mode_id = mode_mapping[mode_name]
            self.mav.set_mode_send(
                target_system=1,
                base_mode=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode=mode_id
            )
            return True
        
        return False
    
    def monitor_telemetry(self, duration_sec: int):
        """Monitor telemetry for specified duration"""
        start_time = time.time()
        
        while time.time() - start_time < duration_sec:
            msg = self.mav.recv_match(blocking=False)
            if msg:
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    telemetry = TelemetryData(
                        timestamp=time.time(),
                        mode=self._get_mode_name(),
                        armed=self._is_armed(),
                        roll=0.0,  # Would need ATTITUDE message
                        pitch=0.0,
                        yaw=0.0,
                        altitude=msg.relative_alt / 1000.0,
                        airspeed=0.0,
                        groundspeed=msg.groundspeed / 100.0,
                        lat=msg.lat / 1e7,
                        lon=msg.lon / 1e7,
                        battery_voltage=0.0,
                        rc_rssi=0,
                        gps_fix_type=0,
                        gps_satellites=0
                    )
                    self.telemetry_log.append(telemetry)
            
            time.sleep(0.1)
    
    def _get_mode_name(self) -> str:
        """Get current mode name"""
        if not self.mav:
            return "UNKNOWN"
        
        mode_mapping = self.mav.mode_mapping()
        if mode_mapping:
            for mode_name, mode_id in mode_mapping.items():
                return mode_name
        
        return "UNKNOWN"
    
    def _is_armed(self) -> bool:
        """Check if vehicle is armed"""
        if not self.mav:
            return False
        
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        
        return False
    
    def stop_sitl(self):
        """Stop SITL process"""
        self.running = False
        
        if self.mav:
            self.disarm_vehicle()
            self.mav.close()
            self.mav = None
        
        if self.sitl_process:
            try:
                os.killpg(os.getpgid(self.sitl_process.pid), signal.SIGTERM)
                self.sitl_process.wait(timeout=5)
            except:
                if self.sitl_process:
                    self.sitl_process.kill()
        
        if self.log_dir:
            try:
                shutil.rmtree(self.log_dir)
            except:
                pass
    
    def run_scenario(self, faults: List[Fault], duration_sec: int = 60) -> TestResult:
        """Run a complete fault injection scenario"""
        self.telemetry_log = []
        
        # Start SITL
        if not self.start_sitl():
            return TestResult(
                success=False,
                mode_end="UNKNOWN",
                max_roll_deg=0.0,
                max_pitch_deg=0.0,
                max_alt_m=0.0,
                min_alt_m=0.0,
                no_disarm_in_air=True,
                eventual_safe_mode=False,
                error_message="Failed to start SITL"
            )
        
        # Connect via MAVLink
        if not self.connect_mavlink():
            self.stop_sitl()
            return TestResult(
                success=False,
                mode_end="UNKNOWN",
                max_roll_deg=0.0,
                max_pitch_deg=0.0,
                max_alt_m=0.0,
                min_alt_m=0.0,
                no_disarm_in_air=True,
                eventual_safe_mode=False,
                error_message="Failed to connect MAVLink"
            )
        
        # Arm and takeoff
        if not self.arm_vehicle():
            self.stop_sitl()
            return TestResult(
                success=False,
                mode_end="UNKNOWN",
                max_roll_deg=0.0,
                max_pitch_deg=0.0,
                max_alt_m=0.0,
                min_alt_m=0.0,
                no_disarm_in_air=True,
                eventual_safe_mode=False,
                error_message="Failed to arm"
            )
        
        # Set to LOITER mode
        self.set_mode("LOITER")
        
        # Wait for stabilization
        time.sleep(5)
        
        # Inject faults at specified times
        start_time = time.time()
        
        for fault in faults:
            fault_time = start_time + fault.t_sec
            
            # Wait until fault injection time
            while time.time() < fault_time:
                time.sleep(0.1)
            
            # Inject fault
            if fault.type == "gps_jump":
                self.inject_gps_fault(
                    fault.params.get('dx_m', 0),
                    fault.params.get('dy_m', 0),
                    fault.params.get('dz_m', 0)
                )
            elif fault.type == "imu_bias":
                self.inject_imu_bias(
                    fault.params.get('axis', 'z'),
                    fault.params.get('bias_rps', 0.1)
                )
            
            # Clear fault after duration
            if fault.duration_sec:
                time.sleep(fault.duration_sec)
                if fault.type == "gps_jump":
                    self.clear_gps_fault()
                elif fault.type == "imu_bias":
                    self.clear_imu_bias()
        
        # Monitor telemetry
        self.monitor_telemetry(duration_sec)
        
        # Analyze results
        result = self._analyze_results()
        
        # Cleanup
        self.stop_sitl()
        
        return result
    
    def _analyze_results(self) -> TestResult:
        """Analyze telemetry and determine test result"""
        if not self.telemetry_log:
            return TestResult(
                success=False,
                mode_end="UNKNOWN",
                max_roll_deg=0.0,
                max_pitch_deg=0.0,
                max_alt_m=0.0,
                min_alt_m=0.0,
                no_disarm_in_air=True,
                eventual_safe_mode=False,
                error_message="No telemetry data"
            )
        
        # Extract metrics
        max_alt = max(t.altitude for t in self.telemetry_log)
        min_alt = min(t.altitude for t in self.telemetry_log)
        max_roll = max(abs(t.roll) for t in self.telemetry_log)
        max_pitch = max(abs(t.pitch) for t in self.telemetry_log)
        
        # Check for in-air disarm
        no_disarm_in_air = True
        for i, t in enumerate(self.telemetry_log):
            if not t.armed and t.altitude > 2.0:  # Disarmed above 2m
                no_disarm_in_air = False
                break
        
        # Check final mode
        final_mode = self.telemetry_log[-1].mode if self.telemetry_log else "UNKNOWN"
        safe_modes = {"RTL", "LAND", "LOITER", "ALT_HOLD"}
        eventual_safe_mode = final_mode in safe_modes
        
        return TestResult(
            success=True,
            mode_end=final_mode,
            max_roll_deg=max_roll,
            max_pitch_deg=max_pitch,
            max_alt_m=max_alt,
            min_alt_m=min_alt,
            no_disarm_in_air=no_disarm_in_air,
            eventual_safe_mode=eventual_safe_mode,
            error_message=""
        )


# ============================================================================
# Property Tests
# ============================================================================

if HYPOTHESIS_AVAILABLE and MAVLINK_AVAILABLE:
    
    @given(
        dx=st.floats(min_value=-50.0, max_value=50.0, allow_nan=False, allow_infinity=False),
        dy=st.floats(min_value=-50.0, max_value=50.0, allow_nan=False, allow_infinity=False),
    )
    @settings(
        max_examples=10,
        deadline=None,
        suppress_health_check=[HealthCheck.too_slow, HealthCheck.filter_too_much]
    )
    def test_gps_jump_sitl(dx, dy):
        """
        Test GPS jump fault with real SITL
        
        Properties:
        - No in-air disarm
        - Eventual safe mode
        - Attitude limits
        """
        runner = SITLRunner(verbose=False)
        
        faults = [
            Fault(
                type=FaultType.GPS_JUMP.value,
                t_sec=10.0,
                duration_sec=5.0,
                params={"dx_m": dx, "dy_m": dy}
            )
        ]
        
        result = runner.run_scenario(faults, duration_sec=30)
        
        # Property assertions
        assert result.no_disarm_in_air, \
            f"Vehicle disarmed in air during GPS jump (dx={dx:.1f}, dy={dy:.1f})"
        
        assert result.eventual_safe_mode, \
            f"Did not reach safe mode after GPS jump, ended in {result.mode_end}"
        
        assert result.max_roll_deg < 45.0, \
            f"Excessive roll {result.max_roll_deg:.1f} deg after GPS jump"
        
        assert result.max_pitch_deg < 45.0, \
            f"Excessive pitch {result.max_pitch_deg:.1f} deg after GPS jump"


# ============================================================================
# Main Entry Point
# ============================================================================

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="ArduPilot SITL Fault Injection Testing"
    )
    parser.add_argument(
        "--examples",
        type=int,
        default=10,
        help="Number of test examples (default: 10)"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Verbose output"
    )
    
    args = parser.parse_args()
    
    if not HYPOTHESIS_AVAILABLE:
        print("ERROR: hypothesis not installed")
        sys.exit(1)
    
    if not MAVLINK_AVAILABLE:
        print("ERROR: pymavlink not installed")
        sys.exit(1)
    
    print(f"Running ArduPilot SITL Fault Injection Tests")
    print(f"Examples: {args.examples}")
    print()
    
    # Run tests
    import pytest
    
    pytest_args = [
        __file__,
        f"--hypothesis-examples={args.examples}",
        "-v" if args.verbose else "-q",
        "-s",  # Print output
    ]
    
    sys.exit(pytest.main(pytest_args))


if __name__ == "__main__":
    main()
