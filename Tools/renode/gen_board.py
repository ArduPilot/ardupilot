#!/usr/bin/env python3
'''
Generate a Renode board overlay from ArduPilot ChibiOS hwdef files.

AP_FLAKE8_CLEAN

The normal ChibiOS hwdef compiler is deliberately used here. Its expanded
configuration and resolved DMA definitions remain the single interpretation of
hwdef.dat; this generator only translates those results into Renode wiring.
'''

import argparse
import contextlib
import io
import json
import re
import shlex
import sys
from pathlib import Path

FAMILIES = {
    'STM32F405xx': {
        'name': 'f405',
        'base': 'stm32f405_base.repl',
        'script': 'ardupilot_f405.resc',
        'spis': {'SPI1', 'SPI2', 'SPI3'},
        'i2cs': {'I2C1', 'I2C2', 'I2C3'},
        'uarts': {'USART1', 'USART2', 'USART3', 'UART4', 'UART5', 'USART6'},
        'sd_buses': set(),
        'timers': {2, 3, 4, 5, 8},
        'uart_irq': {
            'USART1': 37, 'USART2': 38, 'USART3': 39,
            'UART4': 52, 'UART5': 53, 'USART6': 71,
        },
        'spi_irq': {'SPI1': 35, 'SPI2': 36, 'SPI3': 51},
        'i2c_irq': {'I2C1': (31, 32), 'I2C2': (33, 34), 'I2C3': (72, 73)},
    },
    'STM32H743xx': {
        'name': 'h743',
        'base': 'stm32h743_base.repl',
        'script': 'ardupilot_h743.resc',
        'spis': {'SPI1', 'SPI2', 'SPI3', 'SPI4', 'SPI5', 'SPI6'},
        'i2cs': {'I2C1', 'I2C2', 'I2C3', 'I2C4'},
        'uarts': {
            'USART1', 'USART2', 'USART3', 'UART4', 'UART5',
            'USART6', 'UART7', 'UART8', 'LPUART1',
        },
        'sd_buses': {'SDMMC1'},
        'timers': {1, 2, 3, 4, 5, 8, 12, 15},
        'uart_irq': {
            'USART1': 26, 'USART2': 27, 'USART3': 28, 'USART6': 29,
            'UART4': 30, 'UART5': 31, 'UART7': 32, 'UART8': 33,
            'LPUART1': 64,
        },
        'spi_irq': {
            'SPI1': 35, 'SPI2': 36, 'SPI3': 51,
            'SPI4': 84, 'SPI5': 85, 'SPI6': 86,
        },
    },
}


IMU_MODELS = {
    'Invensense': 'Sensors.AP_ICM20689',
    'Invensensev3': 'Sensors.AP_ICM42688',
}

BARO_MODELS = {
    'BMP280': 'Sensors.AP_BMP280',
    'DPS310': 'Sensors.AP_DPS310',
    'SPL06': 'Sensors.AP_DPS310',
}


def _hwdef_root(root):
    return root / 'libraries' / 'AP_HAL_ChibiOS' / 'hwdef'


def _resolved_mcu(path, seen=None):
    '''Find the effective MCU while following the same relative includes.'''
    if seen is None:
        seen = set()
    path = path.resolve()
    if path in seen:
        return None
    seen.add(path)
    mcu = None
    try:
        lines = path.read_text().splitlines()
    except OSError:
        return None
    for raw in lines:
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        fields = shlex.split(line)
        if fields[0] == 'include' and len(fields) == 2:
            included = _resolved_mcu(path.parent / fields[1], seen)
            if included is not None:
                mcu = included
        elif fields[0] == 'MCU' and len(fields) >= 3:
            mcu = fields[2]
        elif fields[0] == 'undef' and len(fields) == 2 and fields[1] == 'MCU':
            mcu = None
    return mcu


def supported_boards(root):
    '''Board name -> MCU for boards backed by both application and BL hwdefs.'''
    found = {}
    for board_dir in sorted(_hwdef_root(root).iterdir()):
        app = board_dir / 'hwdef.dat'
        bootloader = board_dir / 'hwdef-bl.dat'
        if not app.is_file() or not bootloader.is_file():
            continue
        mcu = _resolved_mcu(app)
        if mcu in FAMILIES:
            found[board_dir.name] = mcu
    return found


def _load_compiler(root):
    scripts = _hwdef_root(root) / 'scripts'
    if str(scripts) not in sys.path:
        sys.path.insert(0, str(scripts))
    from chibios_hwdef import ChibiOSHWDef
    return ChibiOSHWDef


def _compile_hwdef(root, hwdef, outdir, bootloader=False):
    '''Run the production hwdef parser and return its resolved object.'''
    outdir.mkdir(parents=True, exist_ok=True)
    compiler = _load_compiler(root)
    config = compiler(
        outdir=str(outdir),
        bootloader=bootloader,
        quiet=True,
        hwdef=[str(hwdef)],
        # The direct script currently requires a string here. A deliberately
        # absent absolute path still allows the board's defaults.parm fallback.
        default_params_filepath=str(outdir / '__no_command_line_defaults__'),
    )
    # Some sensor conversion helpers print diagnostics independently of quiet.
    diagnostics = io.StringIO()
    try:
        with contextlib.redirect_stdout(diagnostics):
            config.run()
    except SystemExit as error:
        message = diagnostics.getvalue().strip()
        raise ValueError(message or 'hwdef compilation failed') from error
    return config


def _defines(path):
    found = {}
    for line in path.read_text().splitlines():
        fields = line.split('//', 1)[0].split(None, 2)
        if len(fields) == 3 and fields[0] == '#define':
            found[fields[1]] = fields[2].strip()
    return found


def _dma_stream(value):
    match = re.fullmatch(r'STM32_DMA_STREAM_ID\((\d+),(\d+)\)', value.replace(' ', ''))
    if match is None:
        return None
    return int(match.group(1)), int(match.group(2))


def _dmamux_requests(root):
    path = (root / 'modules' / 'ChibiOS' / 'os' / 'hal' / 'ports' /
            'STM32' / 'STM32H7xx' / 'stm32_dmamux.h')
    requests = {}
    for line in path.read_text().splitlines():
        match = re.match(r'#define\s+(STM32_DMAMUX1_\w+)\s+(\d+)', line)
        if match:
            requests[match.group(1)] = int(match.group(2))
    return requests


def _safe_name(value):
    return re.sub(r'[^a-zA-Z0-9_]', '_', value)


def _spi_device_name(expression):
    match = re.search(r'get_device\("([^"\n]+)"\)', expression)
    return match.group(1) if match else None


def _sensor_devices(config, family, defines, fram_path, warnings):
    '''Return (REPL declarations, CS pin -> peripheral names).'''
    spi_devices = {dev[0]: dev for dev in config.spidev}
    declarations = []
    chip_selects = {}

    for index, imu in enumerate(config.imu_list):
        if len(imu) < 2 or imu[0] not in IMU_MODELS:
            warnings.append('unmodelled IMU: %s' % ' '.join(imu))
            continue
        device_name = _spi_device_name(imu[1])
        device = spi_devices.get(device_name)
        if device is None:
            warnings.append('cannot resolve IMU SPI device: %s' % ' '.join(imu))
            continue
        bus, cs_label = device[1], device[3]
        if bus not in family['spis']:
            warnings.append('%s is not present in the current MCU base' % bus)
            continue
        cs = config.bylabel.get(cs_label)
        if cs is None:
            warnings.append('cannot resolve chip select %s' % cs_label)
            continue
        name = 'imu%d' % index
        declarations += [
            '%s: %s @ %s' % (name, IMU_MODELS[imu[0]], bus.lower()),
            '',
        ]
        chip_selects.setdefault((cs.port, cs.pin), []).append(name)
        # One healthy IMU is sufficient to reach the scheduler. Additional
        # instances can be added when their exact models are implemented.
        break

    i2c_order = config.get_config('I2C_ORDER', required=False, aslist=True) or []
    used_addresses = set()
    for index, baro in enumerate(config.baro_list):
        model = BARO_MODELS.get(baro.driver)
        if model is None:
            warnings.append('unmodelled barometer: %s' % baro.driver)
            continue
        devices = [d for d in baro.devlist
                   if hasattr(d, 'busnum') and hasattr(d, 'busaddr')]
        if not devices:
            warnings.append('unsupported barometer bus: %s' % baro.driver)
            continue
        device = devices[0]
        if not 0 <= device.busnum < len(i2c_order):
            warnings.append('invalid I2C index for barometer %s' % baro.driver)
            continue
        bus = i2c_order[device.busnum]
        if bus not in family['i2cs']:
            warnings.append('%s is not present in the current MCU base' % bus)
            continue
        key = (bus, device.busaddr)
        if key in used_addresses:
            continue
        used_addresses.add(key)
        declarations += [
            'baro%d: %s @ %s 0x%02X' %
            (index, model, bus.lower(), device.busaddr),
            '',
        ]
        # A single barometer is enough for ArduPilot's initialization gate.
        break

    has_fram = defines.get('HAL_WITH_RAMTRON', '0') != '0'
    if has_fram:
        device = spi_devices.get('ramtron')
        if device is None:
            warnings.append('HAL_WITH_RAMTRON is set without SPIDEV ramtron')
            has_fram = False
        else:
            bus, cs_label = device[1], device[3]
            cs = config.bylabel.get(cs_label)
            if bus not in family['spis']:
                warnings.append('%s RAMTRON bus is not present in the current MCU base' % bus)
                has_fram = False
            elif cs is None:
                warnings.append('cannot resolve RAMTRON chip select %s' % cs_label)
                has_fram = False
            else:
                declarations += [
                    'fram: Miscellaneous.AP_RAMTRON @ %s' % bus.lower(),
                    '    fileName: %s' % json.dumps(str(fram_path)),
                    '',
                ]
                chip_selects.setdefault((cs.port, cs.pin), []).append('fram')

    return declarations, chip_selects, has_fram


def _gpio_routes(family_name, chip_selects):
    lines = []
    ports = sorted({port for port, _ in chip_selects})
    for port in ports:
        lines += ['gpioPort%s:' % port]
        for pin in range(16):
            targets = ['%s@0' % name for name in chip_selects.get((port, pin), [])]
            if family_name == 'f405':
                targets.append('exti@%d' % pin)
            else:
                targets.append('syscfg#%d@%d' % (ord(port) - ord('A'), pin))
            lines.append('    %d -> %s' % (pin, ' | '.join(targets)))
        lines.append('')
    return lines


def _f405_dma_wiring(defines, family, alloc, warnings):
    lines = []
    rx_uarts = []
    for peripheral, irq in family['uart_irq'].items():
        value = defines.get('STM32_UART_%s_RX_DMA_STREAM' % peripheral)
        stream = _dma_stream(value) if value else None
        if stream is None:
            continue
        dma, channel = stream
        lines += [
            '%s:' % peripheral.lower(),
            '    -> nvic@%d' % irq,
            '    DMARequest -> dma%d@%d' % (dma, channel),
            '',
        ]
        rx_uarts.append((peripheral, dma, channel))

    for peripheral, irq in family['spi_irq'].items():
        value = defines.get('STM32_SPI_%s_RX_DMA_STREAM' % peripheral)
        stream = _dma_stream(value) if value else None
        if stream is None:
            continue
        dma, channel = stream
        lines += [
            '%s:' % peripheral.lower(),
            '    IRQ -> nvic@%d' % irq,
            '    DMARecieve -> dma%d@%d' % (dma, channel),
            '',
        ]

    for peripheral, (event_irq, error_irq) in family['i2c_irq'].items():
        value = defines.get('STM32_I2C_%s_RX_DMA_STREAM' % peripheral)
        stream = _dma_stream(value) if value else None
        if stream is None:
            continue
        dma, channel = stream
        lines += [
            '%s:' % peripheral.lower(),
            '    EventInterrupt -> nvic@%d' % event_irq,
            '    ErrorInterrupt -> nvic@%d' % error_irq,
            '    RxDmaRequest -> dma%d@%d' % (dma, channel),
            '',
        ]

    for peripheral, dma, channel in rx_uarts:
        name = _safe_name(peripheral.lower() + 'RxPump')
        lines += [
            '%s: Miscellaneous.AP_UartRxDmaPump @ sysbus 0x%08X' % (name, alloc()),
            '    dma: dma%d' % dma,
            '    stream: %d' % channel,
            '    uart: %s' % peripheral.lower(),
            '',
        ]

    for name, value in sorted(defines.items()):
        match = re.fullmatch(r'STM32_TIM_TIM(\d+)_UP_DMA_STREAM', name)
        stream = _dma_stream(value)
        if match is None or stream is None:
            continue
        timer = int(match.group(1))
        if timer not in family['timers']:
            warnings.append('timer%d update DMA is not in the current MCU base' % timer)
            continue
        dma, channel = stream
        lines += [
            'timer%dUpdateDMA: Miscellaneous.AP_STM32_Timer_UpdateDMA @ sysbus 0x%08X' %
            (timer, alloc()),
            '    timer: timer%d' % timer,
            '    UpdateDMA -> dma%d@%d' % (dma, channel),
            '',
        ]
    return lines


def _h743_dma_wiring(root, defines, family, alloc, warnings):
    lines = []
    requests = _dmamux_requests(root)

    for peripheral, irq in family['uart_irq'].items():
        channel = defines.get('STM32_UART_%s_RX_DMA_CHAN' % peripheral)
        request = requests.get(channel)
        if request is None:
            continue
        irq_target = 'exti@%d' % irq if peripheral != 'LPUART1' else 'exti@64'
        lines += [
            '%s:' % peripheral.lower(),
            '    IRQ -> %s' % irq_target,
            '    ReceiveDmaRequest -> dmamux1@%d' % request,
            '',
            '%sIdle: Miscellaneous.AP_STM32F7_USART_Idle @ sysbus 0x%08X' %
            (peripheral.lower(), alloc()),
            '    uart: %s' % peripheral.lower(),
            '',
        ]

    for peripheral, irq in family['spi_irq'].items():
        channel = defines.get('STM32_SPI_%s_RX_DMA_CHAN' % peripheral)
        request = requests.get(channel)
        if request is None:
            continue
        lines += [
            '%s:' % peripheral.lower(),
            '    IRQ -> nvic@%d' % irq,
            '    DMARecieve -> dmamux1@%d' % request,
            '',
        ]

    for name, channel in sorted(defines.items()):
        match = re.fullmatch(r'STM32_TIM_TIM(\d+)_UP_DMA_CHAN', name)
        if match is None:
            continue
        timer = int(match.group(1))
        request = requests.get(channel)
        if request is None:
            continue
        if timer not in family['timers']:
            warnings.append('timer%d update DMA is not in the current MCU base' % timer)
            continue
        lines += [
            'timer%dUpdateDMA: Miscellaneous.AP_STM32_Timer_UpdateDMA @ sysbus 0x%08X' %
            (timer, alloc()),
            '    timer: timer%d' % timer,
            '    UpdateDMA -> dmamux1@%d' % request,
            '',
        ]
    return lines


def _platform(root, board, app, outdir, fram_path, warnings):
    family = FAMILIES[app.mcu_type]
    base = root / 'Tools' / 'renode' / 'platforms' / family['base']
    lines = [
        '// GENERATED from libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat.' % board,
        '// Edit the hwdef or Tools/renode/gen_board.py, not this file.',
        '',
        'using "%s"' % base,
        '',
    ]
    defines = _defines(outdir / 'hwdef' / 'hwdef.h')
    sensor_lines, chip_selects, has_fram = _sensor_devices(
        app, family, defines, fram_path, warnings)
    lines += sensor_lines

    address = 0x60000010

    def alloc():
        nonlocal address
        value = address
        address += 4
        return value

    lines += [
        'dma1Fix: Miscellaneous.AP_STM32DMA_Fixup @ sysbus 0x%08X' % alloc(),
        '    dma: dma1',
        '',
        'dma2Fix: Miscellaneous.AP_STM32DMA_Fixup @ sysbus 0x%08X' % alloc(),
        '    dma: dma2',
        '',
    ]
    if family['name'] == 'f405':
        lines += _f405_dma_wiring(defines, family, alloc, warnings)
    else:
        lines += _h743_dma_wiring(root, defines, family, alloc, warnings)
    lines += _gpio_routes(family['name'], chip_selects)
    return '\n'.join(lines).rstrip() + '\n', has_fram


def _serial_device(app, family, serial_index):
    order = app.get_config('SERIAL_ORDER', required=False, aslist=True) or []
    requested_index = serial_index
    if serial_index is not None:
        if not 0 <= serial_index < len(order):
            raise ValueError('SERIAL%d is outside SERIAL_ORDER' % serial_index)
        device = order[serial_index]
    else:
        serial_index = next((index for index, entry in enumerate(order)
                             if entry in family['uarts']), None)
        device = order[serial_index] if serial_index is not None else None
    if device is None and requested_index is None:
        return None, None
    if device is None or device in {'EMPTY', 'OTG1', 'OTG2'}:
        raise ValueError('selected serial port is not a UART')
    if device not in family['uarts']:
        raise ValueError('%s is not present in the current MCU base' % device)
    return serial_index, device


def _script(root, board, app, bootloader, platform, serial_index, uart_port,
            warnings):
    family = FAMILIES[app.mcu_type]
    reserve_kb = app.get_config('FLASH_RESERVE_START_KB', default=0, type=int)
    app_base = 0x08000000 + reserve_kb * 1024
    boot_kb = bootloader.get_config(
        'FLASH_BOOTLOADER_LOAD_KB', required=False, default=reserve_kb, type=int)
    if boot_kb != reserve_kb:
        warnings.append('application starts at %uK, bootloader loads at %uK' %
                        (reserve_kb, boot_kb))
    serial_index, serial = _serial_device(app, family, serial_index)
    tick = app.get_config('STM32_ST_USE_TIMER', required=False, default=None)
    sd_buses = {name for name in app.bytype
                if name.startswith('SDMMC') or name == 'SDIO'}
    has_sd = bool(sd_buses & family['sd_buses'])
    for bus in sorted(sd_buses - family['sd_buses']):
        warnings.append('%s is not present in the current MCU base' % bus)
    common = root / 'Tools' / 'renode' / 'scripts' / family['script']
    lines = [
        ':name: %s' % board,
        ':description: generated ArduPilot platform from hwdef.dat',
        '',
        '# GENERATED by Tools/renode/gen_board.py.',
        '$repo?=@%s' % root,
        '$elf?=@none',
        '$platform=@%s' % platform,
        '$app_base=0x%08X' % app_base,
        'include @%s' % common,
        '',
    ]
    if serial is not None:
        lines += [
            'emulation CreateServerSocketTerminal %u "serial" false' % uart_port,
            'connector Connect sysbus.%s serial' % serial.lower(),
            '',
        ]
    if has_sd:
        lines += [
            '$sdcard?=@none',
            'machine SdCardFromFile $sdcard sysbus.sdmmc 0x10000000 True "sdcard"',
            '',
        ]
    if tick is not None and int(tick) in family['timers']:
        lines += [
            'sysbus SetHookBeforePeripheralWrite sysbus.timer%s '
            '"if offset == 0x34 and value == 0: value = 1"' % tick,
            '',
        ]
    elif tick is not None:
        warnings.append('system timer%s is not in the current MCU base' % tick)

    if family['name'] == 'h743':
        for spi in app.spi_list:
            if spi in family['spis']:
                lines.append('sysbus SetHookBeforePeripheralWrite sysbus.%s '
                             '"if offset == 0x0C: value |= 0x400000"' % spi.lower())
        lines.append('')

    noisy = sorted({name.lower() for name in app.spi_list
                    if name in family['spis']} |
                   {name.lower() for name in app.get_config(
                       'I2C_ORDER', required=False, aslist=True) or []
                    if name in family['i2cs']} |
                   {name.lower() for name in app.get_config(
                       'SERIAL_ORDER', required=False, aslist=True) or []
                    if name in family['uarts']} |
                   {'dma1', 'dma2', 'nvic'})
    lines += ['logLevel 3 sysbus.%s' % name for name in noisy]
    if has_sd:
        lines.append('logLevel 3 sysbus.sdmmc')
    lines.append('')
    return '\n'.join(lines), {
        'app_base': app_base,
        'bootloader_load_base': 0x08000000 + boot_kb * 1024,
        'has_sdcard': has_sd,
        'serial_index': serial_index,
        'serial': serial,
        'family': family['name'],
    }


def generate(root, board, outdir, serial_index=None, uart_port=5762,
             state_dir=None):
    '''Generate the board REPL/RESC and return paths plus run metadata.'''
    root = root.resolve()
    outdir = outdir.resolve()
    state_dir = (Path(state_dir) if state_dir is not None else
                 root / 'renode' / board).resolve()
    boards = supported_boards(root)
    if board not in boards:
        raise ValueError('%s is not a supported STM32F405/STM32H743 board' % board)
    board_dir = _hwdef_root(root) / board
    outdir.mkdir(parents=True, exist_ok=True)
    app = _compile_hwdef(root, board_dir / 'hwdef.dat', outdir / 'hwdef')
    bootloader = _compile_hwdef(
        root, board_dir / 'hwdef-bl.dat', outdir / 'hwdef-bl', bootloader=True)
    if app.mcu_type != boards[board] or bootloader.mcu_type != app.mcu_type:
        raise ValueError('application and bootloader MCU definitions disagree')

    warnings = []
    repl = outdir / ('%s.repl' % board)
    resc = outdir / ('%s.resc' % board)
    platform, has_fram = _platform(
        root, board, app, outdir, state_dir / 'fram.img', warnings)
    repl.write_text(platform)
    script, metadata = _script(
        root, board, app, bootloader, repl, serial_index, uart_port, warnings)
    resc.write_text(script)
    metadata.update({
        'repl': repl,
        'resc': resc,
        'warnings': warnings,
        'hwdef_h': outdir / 'hwdef' / 'hwdef.h',
        'has_fram': has_fram,
        'fram_size': 32 * 1024 if has_fram else 0,
    })
    return metadata


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('board', nargs='?')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--outdir')
    parser.add_argument('--state-dir',
                        help='persistent state directory (default: renode/<board>)')
    parser.add_argument('--serial', type=int, help='SERIAL_ORDER index to expose')
    parser.add_argument('--uart-port', type=int, default=5762)
    args = parser.parse_args()
    boards = supported_boards(root)
    if args.list:
        print('\n'.join(boards))
        return 0
    if args.board is None:
        parser.error('a board is required unless --list is used')
    outdir = (Path(args.outdir) if args.outdir else
              root / 'build' / args.board / 'renode' / 'generated')
    try:
        result = generate(root, args.board, outdir, args.serial, args.uart_port,
                          args.state_dir)
    except (OSError, ValueError) as error:
        print('error: %s' % error, file=sys.stderr)
        return 1
    print(result['repl'])
    print(result['resc'])
    for warning in result['warnings']:
        print('warning: %s' % warning, file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
