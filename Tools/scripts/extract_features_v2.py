#!/usr/bin/env python3

"""
Script to validate feature enablement and locate feature definitions in ArduPilot
"""
import os
import sys
import subprocess
import argparse
import re
import fnmatch
import shutil
import traceback
from build_options import BUILD_OPTIONS 

class FeatureEditor:
    def __init__(self, options_file=None):
        """Initialize the feature editor with path to the build_options.py file"""
        self.root_dir = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../..'))
        
        if options_file is None:
            self.options_file = os.path.join(self.root_dir, "Tools/scripts/build_options.py")
        else:
            self.options_file = options_file
        
        if not os.path.exists(self.options_file):
            raise FileNotFoundError(f"Could not find build_options.py at: {self.options_file}")

    def find_define_locations(self):
        """Find actual locations of feature defines in header files."""
        # Dictionary to store results - define name to file paths
        results = {}
        

        print("Searching for feature define locations...")
        print(f"Total features to process: {len(BUILD_OPTIONS)}")
        
        for i, option in enumerate(BUILD_OPTIONS):
            feature_define = option.define
            print(f"[{i+1}/{len(BUILD_OPTIONS)}] Processing: {feature_define}")
            
            define_locations = self.find_define(feature_define)
            
            results[feature_define] = define_locations
            
            if define_locations:
                print(f"  Found in: {', '.join(define_locations[:3])}")
                if len(define_locations) > 3:
                    print(f"  And {len(define_locations) - 3} more locations")
            else:
                print(f"  Not found in header files for {feature_define}")
        
        return results

    def replace_path(self, path, replacements):
        """Replace path using wildcard-based search."""
        for pattern, replacement in replacements.items():
            if fnmatch.fnmatch(path, pattern):
                return replacement
        return path
    
    def find_define(self, pattern):
        """Find header files where a pattern is defined."""
        valid_paths = []
        while(True):
            # First try to find in config files
            config_files = self.search_for_files("*_config.h")
            
            for config_file in config_files:
                if self.check_file_for_define(config_file, pattern):
                    rel_path = os.path.relpath(config_file, f"{self.root_dir}/libraries/")
                    if rel_path not in valid_paths:
                        valid_paths.append(rel_path)
            checked_headers = [config_file]
            if valid_paths:
                break
                
            # Look in board definitions
            boards_file = os.path.join(self.root_dir, "libraries/AP_HAL/AP_HAL_Boards.h")
            if os.path.exists(boards_file) and self.check_file_for_define(boards_file, pattern):
                rel_path = os.path.relpath(boards_file, f"{self.root_dir}/libraries/")
                if rel_path not in valid_paths:
                    valid_paths.append(rel_path)
            checked_headers.append(boards_file)
            if valid_paths:
                break

            # Look in all header files, with some exclusions
            ignore_files = [
                "*libraries/AP_HAL_ChibiOS/hwdef/scripts/defaults_*",
                "*libraries/AP_HAL/board/*",
                "*ArduCopter/*",
                "*ArduPlane/*",
                "*ArduSub/*",
                "*Rover/*",
                "*Blimp/*",
                "*AntennaTracker/*",
                "*Tools/AP_Periph/*",
            ]
            header_files = self.search_for_files("*.h", ignore_files)
            # Remove already checked headers
            for file in header_files:
                if file in checked_headers:
                    header_files.remove(file)
            checked_headers += header_files
            for header_file in header_files:
                if self.check_file_for_define(header_file, pattern):
                    rel_path = os.path.relpath(header_file, f"{self.root_dir}/libraries/")
                    if rel_path not in valid_paths:
                        valid_paths.append(rel_path)
            
            if valid_paths:
                break
                    
            # Dictionary to store path replacements
            path_replacements = {
                "Rover/*.h": "AP_Vehicle/AP_Vehicle.h",
                "ArduCopter/*.h": "AP_Vehicle/AP_Vehicle.h",
                "ArduPlane/*.h": "AP_Vehicle/AP_Vehicle.h",
                "ArduSub/*.h": "AP_Vehicle/AP_Vehicle.h",
                "Blimp/*.h": "AP_Vehicle/AP_Vehicle.h",
                "AntennaTracker/*.h": "AP_Vehicle/AP_Vehicle.h",
                "Tools/AP_Periph/*.h": "AP_Vehicle/AP_Vehicle.h",
            }
            # Final pass - look in all header files
            ignore_files = [
                "*libraries/*"
            ]
            header_files = self.search_for_files("*.h", ignore_files)
            checked_headers += header_files
            
            # we didn't find the define in libraries, so search in the root directory
            for header_file in header_files:
                if self.check_file_for_define(header_file, pattern):
                    # replace path if needed
                    rel_path = os.path.relpath(header_file, self.root_dir)
                    rel_path = self.replace_path(rel_path, path_replacements)
                    if rel_path not in valid_paths:
                        valid_paths.append(rel_path)
            break
        valid_paths = list(set(valid_paths))
        if not valid_paths:
            raise RuntimeError(f"No header files found for define {pattern}")
        if len(valid_paths) > 1:
            raise RuntimeError(f"Multiple header files found for define {pattern}: {valid_paths}")
        return valid_paths

    def search_for_files(self, pattern, ignore_pattern=None):
        """Use git ls-files to find files matching the pattern."""
        try:
            cmd = ["git", "ls-files", "*" + pattern]
            output = subprocess.check_output(cmd, cwd=self.root_dir, stderr=subprocess.PIPE, universal_newlines=True)
            files = [os.path.join(self.root_dir, file) for file in output.strip().split('\n') if file]
            
            if ignore_pattern:
                if isinstance(ignore_pattern, list):
                    for pattern in ignore_pattern:
                        files = [file for file in files if not fnmatch.fnmatch(file, pattern)]
                else:
                    files = [file for file in files if not fnmatch.fnmatch(file, ignore_pattern)]
                
            return files
        except subprocess.CalledProcessError:
            return []

    def check_file_for_define(self, file_path, pattern):
        """Check if a file contains a define for the given pattern, ignoring commented defines."""
        try:
            with open(file_path, 'r', errors='replace') as f:
                content = f.read()

                # Remove single-line comments
                content = re.sub(r'//.*$', '', content, flags=re.MULTILINE)
                
                # Remove multi-line comments
                content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
                
                ifndef_pattern = rf'#ifndef\s+{pattern}\s*\n\s*#define\s+{pattern}'
                define_pattern = rf'#\s*define\s+{pattern}\b'
                
                if re.search(ifndef_pattern, content) or re.search(define_pattern, content):
                    return True
                    
                option_pattern = rf'{pattern}\s*[!=]=\s*\d'
                if re.search(option_pattern, content):
                    return True
                    
                return False
        except Exception as e:
            print(f"Error checking file {file_path}: {e}")
            return False

    def check_features_headers(self, dry_run=True):
        """Check feature definitions in build_options.py against correct header paths without updating."""
        # First get the define locations
        try:
            define_locations = self.find_define_locations()
            
            # Dictionary to track issues
            missing_headers = []
            incorrect_headers = []
            
            # Check each feature in BUILD_OPTIONS
            for option in BUILD_OPTIONS:
                define_name = option.define
                
                # Get the header paths for this define
                header_paths = define_locations.get(define_name, [])
                
                # Check if the feature has a header defined
                has_header = hasattr(option, 'header') and option.header
                
                if not header_paths:
                    print(f"WARNING: No header found for {define_name}")
                    continue
                    
                if not has_header:
                    missing_headers.append((define_name, header_paths[0]))
                    print(f"Missing header for {define_name}: should be {header_paths[0]}")
                elif header_paths and option.header != header_paths[0]:
                    incorrect_headers.append((define_name, option.header, header_paths[0]))
                    print(f"Incorrect header for {define_name}: has '{option.header}', should be '{header_paths[0]}'")
            
            print("\nSummary:")
            print(f"Features with missing headers: {len(missing_headers)}")
            print(f"Features with incorrect headers: {len(incorrect_headers)}")
            
            return len(missing_headers) + len(incorrect_headers)
            
        except Exception as e:
            print(f"Error checking feature headers: {e}")
            traceback.print_exc()
            return 0

def generate_cpp_file(output_path=None):
    """Generate a C++ file with symbols for all features"""
    
    # Use BUILD_OPTIONS instead of ExtractFeatures for feature definitions
    features = [(option.define, option.description, "") for option in BUILD_OPTIONS]
    
    # Extract all unique header files from the features list
    header_files = set()
    for option in BUILD_OPTIONS:
        try:
            if len(option.header) == 0:
                # Skip features without headers
                continue
        except Exception as e:
            raise Exception(f"Feature extraction for {option.define} failed: {e}")
        # Add the header file if it exists
        header_files.add(option.header)
    
    cpp_content = """// Auto-generated file to embed symbols for feature detection
// Do not edit by hand - generated by extract_features_v2.py

#include <AP_HAL/AP_HAL_Boards.h>
#if !defined(HAL_BOOTLOADER_BUILD) && !defined(IOMCU_FW)
"""
    # remove AP_HAL_Boards.h from header_files
    header_files.discard("AP_HAL/AP_HAL_Boards.h")
    # Add includes for all header files
    for header in sorted(header_files):
        cpp_content += f'#include "{header}"\n'
    
    cpp_content += """
// This file intentionally doesn't have any other code
// The header inclusions will cause symbols to be included in the binary
// which extract_features_compiled.py can detect

// Attributes to prevent function optimization

#include <stdint.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#include "../../ArduPlane/quadplane.h"
#include "../../ArduPlane/config.h"
#include "../../ArduPlane/mode.h"
#elif APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include "../../ArduCopter/config.h"
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
#include "../../ArduSub/config.h"
#elif APM_BUILD_TYPE(APM_BUILD_AntennaTracker)
#include "../../AntennaTracker/config.h"
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
#include "../../Rover/config.h"
#elif APM_BUILD_TYPE(APM_BUILD_Heli)
#include "../../ArduCopter/config.h"
#elif APM_BUILD_TYPE(APM_BUILD_Blimp)
#include "../../Blimp/config.h"
#elif APM_BUILD_TYPE(APM_BUILD_AP_Periph)
#include "../../Tools/AP_Periph/AP_Periph.h"
#endif
"""
    # Add symbols for each feature
    cpp_content += "// Embed symbols for each feature to ensure they can be detected\n"

    # Keep track of all features we've processed to avoid duplicates
    processed_features = set()

    for feature_name, _, _ in features:
        if feature_name not in processed_features:
            # Create unique symbol names based on the feature name
            symbol_name = re.sub(r'[^a-zA-Z0-9_]', '_', feature_name)
            cpp_content += f"""
#if defined({feature_name}) && {feature_name}
// Feature: {feature_name}
extern "C" {{
    void ap_feature_enabled_{symbol_name}();
    void ap_feature_enabled_{symbol_name}() {{}}
}}
#endif
"""
            processed_features.add(feature_name)

    cpp_content += """
#endif // !defined(HAL_BOOTLOADER_BUILD)
"""
    # Use default path if not specified
    if output_path is None:
        ardupilot_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
        output_path = os.path.join(ardupilot_root, "libraries/AP_Common/AP_FeatureValidator.cpp")
    output_dir = os.path.dirname(output_path)
    # Create directories if they don't exist
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    with open(output_path, 'w') as f:
        f.write(cpp_content)
    
    print(f"Generated {output_path} with {len(processed_features)} features")

def extract_features(object_file, nm, feature_txt_path="feature.txt"):
    """Compare enabled features in a binary against expected features and save to feature.txt"""
    print(f"Extracting features from object file: {object_file}")
    
    # Get all potential features
    all_features = set()
    for option in BUILD_OPTIONS:
        all_features.add(option.define)
    
    # Get features from the binary using nm command
    try:
        # Look for symbols with pattern ap_feature_enabled_*
        # Ensure nm is a string, not a list
        nm_cmd = nm[0] if isinstance(nm, list) else nm
        cmd = [nm_cmd, object_file]
        output = subprocess.check_output(cmd, stderr=subprocess.PIPE, universal_newlines=True)
        
        # Convert back to original feature name
        # Build a map of symbol names to feature names
        symbol_to_feature = {}
        for option in BUILD_OPTIONS:
            feature_name = option.define
            symbol_name = re.sub(r'[^a-zA-Z0-9_]', '_', feature_name)
            symbol_to_feature[symbol_name] = feature_name
        
        # Extract all feature symbols from nm output
        enabled_features = set()
        for line in output.splitlines():
            if "ap_feature_enabled_" in line:
                # Extract feature name from symbol
                symbol_match = re.search(r'ap_feature_enabled_([A-Za-z0-9_]+)', line)
                if symbol_match:
                    symbol_name = symbol_match.group(1)
                    if symbol_name in symbol_to_feature:
                        enabled_features.add(symbol_to_feature[symbol_name])
        
        # Calculate disabled features
        disabled_features = all_features - enabled_features
        
        # Generate feature list
        feature_list = []
        # Add enabled features
        for feature in sorted(enabled_features):
            feature_list.append(feature)
        # Add disabled features with ! prefix
        for feature in sorted(disabled_features):
            feature_list.append(f"!{feature}")
            
        # Write the feature list to feature.txt
        with open(feature_txt_path, 'w') as f:
            for feature in feature_list:
                f.write(f"{feature}\n")
                
        print(f"\nResults")
        print(f"Total enabled features: {len(enabled_features)}")
        print(f"Total disabled features: {len(disabled_features)}")
        print(f"Feature list written to {feature_txt_path}")
        
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"Error analyzing binary: {e}")
        return False
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='ArduPilot Feature Management Tools')
    subparsers = parser.add_subparsers(dest='command')
    
    # Generate command
    generate_parser = subparsers.add_parser('generate', help='Generate C++ file with feature symbols')
    generate_parser.add_argument('--output', type=str, 
                               help='Path to output C++ file (default: libraries/AP_Common/AP_FeatureValidator.cpp)')
    
    # Extract command
    extracter = subparsers.add_parser('extract', help='Extract features in binary')
    extracter.add_argument('--object-file', type=str, required=True,
                                 help='object file to extract features from')
    extracter.add_argument('--feature-out', type=str, required=True,
                               help='File to write feature list')
    extracter.add_argument('-nm', type=str, default="arm-none-eabi-nm", help='nm binary to use.')
    
    # Check features command
    check_features_parser = subparsers.add_parser('check-features-headers', help='Check feature header information against BUILD_OPTIONS')
    check_features_parser.add_argument('--options-file', type=str,
                                    help='Path to build_options.py file')
    
    args = parser.parse_args()
    
    if args.command == 'generate':
        generate_cpp_file(args.output)
    elif args.command == 'validate':
        result = extract_features(args.object, args.nm, args.expected)
        sys.exit(0 if result else 1)
    elif args.command == 'check-features-headers':
        editor = FeatureEditor(options_file=args.options_file)
        editor.check_features_headers()
    else:
        parser.print_help()
        sys.exit(1)

if __name__ == "__main__":
    main()
