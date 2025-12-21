# flake8: noqa

import os
import json
import shutil

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

ELF_NAME_MAPPING = {
    'copter': 'arducopter',
    'plane': 'arduplane',
    'rover': 'ardurover',
    'sub': 'ardusub',
    'blimp': 'blimp',
    'antennatracker': 'antennatracker',
    'bootloader': 'AP_Bootloader',
    'AP_Periph': 'AP_Periph',
}

def update_settings(bld):
    if not bld.cmd in ELF_NAME_MAPPING:
        return
    board_name = bld.env.BOARD
    elf_file_name = ELF_NAME_MAPPING.get(bld.cmd, bld.cmd)
    if elf_file_name == 'ap_bootloader':
        elf_file_path = os.path.join("${workspaceFolder}", "build", board_name, "bootloader", elf_file_name)
    else:
        elf_file_path = os.path.join("${workspaceFolder}", "build", board_name, "bin", elf_file_name)
    vscode_setting_json_path = os.path.join(bld.srcnode.abspath(), '.vscode', 'settings.json')

    if not os.path.exists(vscode_setting_json_path):
        with open(vscode_setting_json_path, 'w') as f:
            json.dump({}, f, indent=4)

    try:
        with open(vscode_setting_json_path, 'r') as f:
            content = f.read().strip()
            if content:
                settings_json = json.loads(content)
            else:
                settings_json = {}
    except json.JSONDecodeError:
        print(f"VS-LAUNCH: \033[91m Error: invalid JSON in .vscode/settings.json, please fix it and try again.\033[0m")
        return

    settings_json['wscript.elf_file_path'] = elf_file_path
    settings_json['wscript.board'] = board_name
    if board_name == 'sitl':
        if os.uname().sysname == 'Darwin':
            settings_json['wscript.MIMode'] =  "lldb" 
        else:
            settings_json['wscript.MIMode'] =  "gdb" 

    with open(vscode_setting_json_path, 'w') as f:
        json.dump(settings_json, f, indent=4)

def update_openocd_cfg(cfg):
    if cfg.options.board != 'sitl':
        openocd_cfg_path = os.path.join(cfg.srcnode.abspath(), 'build', cfg.options.board, 'openocd.cfg')
        mcu_type = cfg.env.get_flat('APJ_BOARD_TYPE')
        openocd_target = ''
        if mcu_type.startswith("STM32H7"):
            if mcu_type in H7_DUAL_BANK_LIST:
                openocd_target = 'stm32h7x_dual_bank.cfg'
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

        if openocd_target:
            with open(openocd_cfg_path, 'w+') as f:
                f.write("source [find interface/stlink.cfg]\n")
                f.write(f"source [find target/{openocd_target}]\n")
                f.write("init\n")
                if mcu_type.startswith("STM32H7"):
                    f.write("stm32h7x.cpu0 configure -rtos auto\n")
                else:
                    f.write("$_TARGETNAME configure -rtos auto\n")

def init_launch_json_if_not_exist(cfg):
    launch_json_path = os.path.join(cfg.srcnode.abspath(), '.vscode', 'launch.json')
    launch_default_json_path = os.path.join(cfg.srcnode.abspath(),'.vscode', 'launch.default.json')
    
    if not os.path.exists(launch_json_path):
        if os.path.exists(launch_default_json_path):
            print(f"Copying {launch_default_json_path} to {launch_json_path}")
            shutil.copy(launch_default_json_path, launch_json_path)
