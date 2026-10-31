# flake8: noqa

import os
import json
import pathlib
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
        content = pathlib.Path(vscode_setting_json_path).read_text().strip()
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

def _board_name(ctx):
    board = getattr(getattr(ctx, 'options', None), 'board', None)
    if not board and hasattr(ctx, 'env'):
        board = ctx.env.get_flat('BOARD')
    return board

def update_openocd_cfg(cfg):
    board = _board_name(cfg)
    if board and board != 'sitl':
        openocd_cfg_path = os.path.join(cfg.srcnode.abspath(), 'build', board, 'openocd.cfg')
        openocd_dir = os.path.dirname(openocd_cfg_path)
        if not os.path.isdir(openocd_dir):
            print(f"VS-LAUNCH: \033[91m{openocd_dir} does not exist yet, skipping openocd.cfg\033[0m")
            return
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

def _vscode_dir(ctx):
    return os.path.join(ctx.srcnode.abspath(), '.vscode')

def _copy_if_missing(src, dst):
    if os.path.exists(dst):
        return
    if not os.path.exists(src):
        print(f"VS-LAUNCH: \033[91mmissing {src}, cannot create {os.path.basename(dst)}\033[0m")
        return
    print(f"Copying {src} to {dst}")
    shutil.copy(src, dst)

def init_launch_json_if_not_exist(cfg):
    vscode_dir = _vscode_dir(cfg)
    _copy_if_missing(
        os.path.join(vscode_dir, 'launch.default.json'),
        os.path.join(vscode_dir, 'launch.json'),
    )
    _copy_if_missing(
        os.path.join(vscode_dir, 'settings.default.json'),
        os.path.join(vscode_dir, 'settings.json'),
    )
