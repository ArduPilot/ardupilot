import os
import json

VS_LAUNCH_DEFAULT_TEMPLATE = {
    "version": "0.2.0",
    "configurations": []
}

SITL_DEFAULT_ARGS = [
    "-S",
    "--model",
    "+",
    "--speedup",
    "1",
    "--slave",
    "0",
    "--sim-address=127.0.0.1",
    "-IO"
]

H7_DUAL_BANK_LIST = [
   "STM32H7A3xx",
   "STM32H7A3xxq",
   "STM32H7B3xx",
   "STM32H7B3xxq",
   "STM32H742xx",
   "STM32H743xx",
   "STM32H745xg",
   "STM32H745xx",
   "STM32H747xg",
   "STM32H747xx",
   "STM32H753xx",
   "STM32H755xx",
   "STM32H755xx",
   "STM32H757xx",
] # List of H7 boards with dual bank

CONIFIGURATIONTS = [
    {'name': 'ArduCopter', 'target': 'arducopter'},
    {'name': 'ArduPlane', 'target': 'arduplane'},
    {'name': 'ArduCopter-Heli', 'target': 'arducopter-heli'},
    {'name': 'ArduRover', 'target': 'ardurover'},
    {'name': 'ArduSub', 'target': 'ardusub'},
    {'name': 'AP_Periph', 'target': 'AP_Periph'},
    {'name': 'AP_Blimp', 'target': 'blimp'},
    {'name': 'AntennaTracker', 'target': 'antennatracker'},
    {'name': 'AP_Bootloader', 'target': 'ap_bootloader'},
] #pending to add more targets

def merge_dicts(dict1, dict2):
    """
    Recursively merges dict2 into dict1.
    """
    for key, value in dict2.items():
        if key in dict1 and isinstance(dict1[key], dict) and isinstance(value, dict):
            merge_dicts(dict1[key], value)
        else:
            dict1[key] = value
    return dict1

def _load_vscode_launch_profile_json(file_path):
    if os.path.exists(file_path):
        try:
            with open(file_path, 'r') as f:
                content = f.read().strip()
                if content:
                    launch_profile_json = json.loads(content)
                else:
                    launch_profile_json = {}
            return launch_profile_json
        except json.JSONDecodeError:
            print(f"\033[91mError: Invalid JSON in {file_path}. \033[0m")
            return {}
    else:
        return {}

def _load_vscode_launch_json(file_path):
    if os.path.exists(file_path):
        try:
            with open(file_path, 'r') as f:
                content = f.read().strip()
                if content:
                    launch_json = json.loads(content)
                else:
                    launch_json = VS_LAUNCH_DEFAULT_TEMPLATE
            if "configurations" not in launch_json:
                launch_json["configurations"] = [] #initialize configurations
            if type(launch_json["configurations"]) is not list:
                launch_json["configurations"] = [] #overwrite invalid configurations
        except json.JSONDecodeError:
            print(f"\033[91mError: Invalid JSON in {file_path}. Creating a new launch.json file.\033[0m")
            launch_json = VS_LAUNCH_DEFAULT_TEMPLATE
    else:
        launch_json = VS_LAUNCH_DEFAULT_TEMPLATE
    return launch_json

def _create_vscode_launch_json(cfg):
    launch_json_path = os.path.join(cfg.srcnode.abspath(), '.vscode', 'launch.json')
    launch_profile_json_path = os.path.join(cfg.srcnode.abspath(), '.vscode', 'launch_profile.json')
    launch_json = _load_vscode_launch_json(launch_json_path)
    launch_profile = _load_vscode_launch_profile_json(launch_profile_json_path)
    configurations = [
        {'name': 'ArduCopter', 'target': 'arducopter'},
        {'name': 'ArduPlane', 'target': 'arduplane'},
        {'name': 'ArduCopter-Heli', 'target': 'arducopter-heli'},
        {'name': 'ArduRover', 'target': 'ardurover'},
        {'name': 'ArduSub', 'target': 'ardusub'},
        {'name': 'AP_Periph', 'target': 'AP_Periph'},
        {'name': 'AP_Blimp', 'target': 'blimp'},
        {'name': 'AntennaTracker', 'target': 'antennatracker'},
        {'name': 'AP_Bootloader', 'target': 'ap_bootloader'},
    ] #pending to add more targets

    for configuration in CONIFIGURATIONTS:
        if cfg.options.board == 'sitl':
            if configuration['name'] == 'AP_Bootloader':
                continue
            if os.uname().sysname == 'Darwin':
                MIMode = "lldb"
            else:
                MIMode = "gdb"
            
            launch_configuration = {
                "name": configuration['name'],
                "type": "cppdbg",
                "request": "launch",
                "program": "${workspaceFolder}/build/sitl/bin/" + configuration['target'],
                "args": SITL_DEFAULT_ARGS,
                "stopAtEntry": False,
                "cwd": "${workspaceFolder}",
                "environment": [],
                "externalConsole": False,
                "MIMode": MIMode,
                "setupCommands": [
                    {
                        "description": "Enable pretty-printing for gdb",
                        "text": "-enable-pretty-printing",
                        "ignoreFailures": False
                    }
                ],
            }

            if launch_profile.get('sitl'):
                merge_dicts(launch_configuration, launch_profile.get('sitl'))
        else:
            if configuration['name'] == 'AP_Bootloader':
                executable_path = "${workspaceFolder}/build/"+ cfg.options.board + "/bootloader/AP_Bootloader"
            else:
                executable_path = "${workspaceFolder}/build/"+ cfg.options.board + "/bin/" + configuration['target']
            launch_configuration = {
                "name": configuration['name'],
                "cwd": "${workspaceFolder}",
                "executable": executable_path,
                "variables": {
                    "ELF_FILE": executable_path,
                },
                "liveWatch": {
                    "enabled": True,
                    "samplesPerSecond": 4
                },
                "request": "launch",
                "type": "cortex-debug",
                "servertype": "openocd",
                "configFiles": [
                    "${workspaceFolder}/build/" + cfg.options.board + "/openocd.cfg",
                ]
            }

            if launch_profile.get('hardware'):
                merge_dicts(launch_configuration, launch_profile.get('hardware'))

        configuration_overwrited = False
        for index, current_configuration in enumerate(launch_json["configurations"]):
            if current_configuration.get('name') == launch_configuration['name']:
                launch_json["configurations"][index] = launch_configuration
                configuration_overwrited = True
                break
        if not configuration_overwrited:
            launch_json["configurations"].append(launch_configuration)

    with open(launch_json_path, 'w') as f:
        json.dump(launch_json, f, indent=4)

    #create openocd.cfg file
    if cfg.options.board != 'sitl':
        openocd_cfg_path = os.path.join(cfg.srcnode.abspath(), 'build', cfg.options.board, 'openocd.cfg')
        mcu_type = cfg.env.get_flat('APJ_BOARD_TYPE')
        openocd_target = ''
        if mcu_type.startswith("STM32H7"):
            if mcu_type in H7_DUAL_BANK_LIST:
                openocd_target = 'stm32h7_dual_bank.cfg'
            else:
                openocd_target = 'stm32h7x.cfg'
        elif mcu_type.startswith("STM32F7"):
            openocd_target = 'stm32f7x.cfg'
        elif mcu_type.startswith("STM32F4"):
            openocd_target = 'stm32f4x.cfg'
        elif mcu_type.startswith("STM32F3"):
            openocd_target = 'stm32f3x.cfg'
        elif mcu_type.startswith("STM32L4"):
            openocd_target = 'stm32l4x.cfg'
        elif mcu_type.startswith("STM32G4"):
            openocd_target = 'stm32g4x.cfg'

        with open(openocd_cfg_path, 'w+') as f:
            f.write("source [find interface/stlink.cfg]\n")
            f.write(f"source [find target/{openocd_target}]\n")
            f.write("init\n")
            f.write("$_TARGETNAME configure -rtos auto\n")
