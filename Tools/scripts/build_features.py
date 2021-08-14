#!/usr/bin/env python3

import subprocess
import re

# list of build options to offer
result = subprocess.run(['git', 'grep', '-w', '-h', 
                        '-o', '-E', 'define *[_A-Z]*_ENABLED.*'], 
                        capture_output=True, text=True)
minimized_feature = re.compile("define *([_A-Z]*_ENABLED).*!HAL_MINIMIZE_FEATURES.*")
board_1mb_feature = re.compile("define *([_A-Z]*_ENABLED).*BOARD_FLASH_SIZE.*1024.*")
regular_feature = re.compile("define *([_A-Z]*_ENABLED).*")
features = set(result.stdout.splitlines())
BUILD_OPTIONS = [
    ['EKF2', 'HAL_NAVEKF2_AVAILABLE', 'Enable EKF2', False],
    ['EKF3', 'HAL_NAVEKF3_AVAILABLE', 'Enable EKF3', True]
    ]
for f in features:
    build_option = []
    optional_feature = minimized_feature.match(f) or board_1mb_feature.match(f)
    matched = regular_feature.match(f)
    if optional_feature:
        define = matched.group(1)
        feature = define.replace('HAL_','').replace('_ENABLED','')
        build_option.append(feature)
        build_option.append(define)
        build_option.append('Enable ' + 
                            feature.replace('_', ' ').lower().capitalize())
        build_option.append(False)
        BUILD_OPTIONS.append(build_option)

print(BUILD_OPTIONS)