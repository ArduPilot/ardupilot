#!/usr/bin/env python3
"""Run a disabled test by temporarily re-enabling it.

Usage:
    python run_disabled_test.py <test_name> [vehicle]
    
Examples:
    python run_disabled_test.py RTLStoppingDistanceSpeed copter
    python run_disabled_test.py FlyMissionTwice copter
    python run_disabled_test.py LandingDrift plane
"""
import sys
import os
import re
import subprocess
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent.parent

# Map test name -> (file, vehicle)
TEST_MAP = {
    "RTLStoppingDistanceSpeed": ("arducopter.py", "copter"),
    "FlyMissionTwice": ("arducopter.py", "copter"),
    "FlyMissionTwiceWithReset": ("arducopter.py", "copter"),
    "SMART_RTL_EnterLeave": ("arducopter.py", "copter"),
    "SMART_RTL_Repeat": ("arducopter.py", "copter"),
    "GroundEffectCompensation_takeOffExpected": ("arducopter.py", "copter"),
    "GroundEffectCompensation_touchDownExpected": ("arducopter.py", "copter"),
    "LandingDrift": ("arduplane.py", "plane"),
    "SoaringClimbRate": ("arduplane.py", "plane"),
    "SlewRate": ("rover.py", "rover"),
    "PolyFenceObjectAvoidanceBendyRuler": ("rover.py", "rover"),
    "FRSkyPassThrough": ("quadplane.py", "quadplane"),
    "ConfigErrorLoop": ("quadplane.py", "quadplane"),
}


def remove_from_disabled(file_path: str, test_name: str) -> bool:
    """Remove test from disabled_tests() dict. Returns True if found and removed."""
    with open(file_path, "r") as f:
        content = f.read()
    
    # Pattern to match the test name in the disabled dict
    # Matches: "TestName": "reason",
    pattern = rf'^\s+"{re.escape(test_name)}":\s+"[^"]+",$\n'
    
    new_content = re.sub(pattern, "", content, flags=re.MULTILINE)
    
    if new_content == content:
        print(f"  Test '{test_name}' not found in disabled dict in {file_path}")
        return False
    
    with open(file_path, "w") as f:
        f.write(new_content)
    
    print(f"  Removed '{test_name}' from disabled_tests() in {file_path}")
    return True


def restore_file(file_path: str):
    """Restore file to original state via git checkout."""
    result = subprocess.run(
        ["git", "checkout", "--", file_path],
        cwd=ROOT,
        capture_output=True,
        text=True
    )
    if result.returncode == 0:
        print(f"  Restored {file_path} via git checkout")
    else:
        print(f"  WARNING: git checkout failed: {result.stderr.strip()}")


def run_test(test_name: str, vehicle: str):
    """Run a single disabled test."""
    print(f"\n{'='*60}")
    print(f"Running DISABLED test: {test_name} (vehicle: {vehicle})")
    print(f"{'='*60}\n")
    
    # Find the source file
    filename = TEST_MAP.get(test_name)
    if not filename:
        print(f"ERROR: Unknown test '{test_name}'. Choose from: {list(TEST_MAP.keys())}")
        return 1
    
    src_file, vehicle_from_map = filename
    if vehicle != vehicle_from_map:
        print(f"WARNING: Vehicle '{vehicle}' doesn't match expected '{vehicle_from_map}'")
    
    file_path = str(ROOT / "Tools" / "autotest" / src_file)
    
    # Temporarily re-enable the test
    removed = remove_from_disabled(file_path, test_name)
    if not removed:
        # Maybe it's not in disabled_tests but already enabled? Try running anyway.
        print("  Test may not be in disabled dict - running anyway...")
    
    try:
        # Run the test (results go to buildlogs/ automatically)
        cmd = [
            sys.executable,
            str(ROOT / "Tools" / "autotest" / "autotest.py"),
            f"build.{vehicle.capitalize()}",
            f"test.{vehicle.capitalize()}.{test_name}",
        ]

        print(f"Running: {' '.join(cmd)}")
        result = subprocess.run(
            cmd,
            cwd=ROOT,
            capture_output=False,  # Let output stream directly
        )
        
        if result.returncode == 0:
            print(f"\n✅ TEST PASSED: {test_name}")
            return 0
        else:
            print(f"\n❌ TEST FAILED: {test_name} (exit code {result.returncode})")
            return 1
    finally:
        # Always restore the file
        restore_file(file_path)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    
    test_name = sys.argv[1]
    vehicle = sys.argv[2] if len(sys.argv) > 2 else None
    
    # Auto-detect vehicle if not specified
    if not vehicle:
        filename = TEST_MAP.get(test_name)
        if filename:
            vehicle = filename[1]
        else:
            print(f"ERROR: Unknown test '{test_name}'. Cannot auto-detect vehicle.")
            return 1
    
    return run_test(test_name, vehicle)


if __name__ == "__main__":
    sys.exit(main())
