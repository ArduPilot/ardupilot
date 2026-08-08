#!/usr/bin/env python3
'''
Generate a Renode board overlay from ArduPilot ChibiOS hwdef files.

AP_FLAKE8_CLEAN

The normal ChibiOS hwdef compiler is deliberately used here. Its expanded
configuration and resolved DMA definitions remain the single interpretation of
hwdef.dat; this generator only translates those results into Renode wiring.
'''

import argparse
import ast
import contextlib
import io
import json
import re
import shlex
import sys
from pathlib import Path

FAMILIES = {
    'STM32F103xB': {
        'name': 'f103',
        'base': 'stm32f103_base.repl',
        'script': 'ardupilot_f103.resc',
        'spis': {'SPI1', 'SPI2'},
        'i2cs': {'I2C1', 'I2C2'},
        'uarts': {'USART1', 'USART2', 'USART3', 'UART4', 'UART5'},
        'sd_buses': {},
        'can_buses': {'CAN': 'can1'},
        'timers': {1, 2, 3, 4},
        'uart_irq': {
            'USART1': 37, 'USART2': 38, 'USART3': 39,
            'UART4': 52, 'UART5': 53,
        },
        'spi_irq': {'SPI1': 35, 'SPI2': 36},
        'i2c_irq': {'I2C1': (31, 32), 'I2C2': (33, 34)},
    },
    'STM32G474xx': {
        'name': 'g474',
        'base': 'stm32g474_base.repl',
        'script': 'ardupilot_g474.resc',
        'spis': {'SPI1', 'SPI2', 'SPI3', 'SPI4'},
        'i2cs': {'I2C1', 'I2C2', 'I2C3', 'I2C4'},
        'uarts': {
            'USART1', 'USART2', 'USART3', 'UART4', 'UART5', 'LPUART1',
        },
        'sd_buses': {},
        'can_buses': {'CAN1': 'fdcan1', 'CAN2': 'fdcan2'},
        'timers': {1, 2, 3, 4, 5, 6, 7, 8, 15, 16, 17, 20},
        'uart_irq': {
            'USART1': 37, 'USART2': 38, 'USART3': 39,
            'UART4': 52, 'UART5': 53, 'LPUART1': 91,
        },
        'spi_irq': {'SPI1': 35, 'SPI2': 36, 'SPI3': 51, 'SPI4': 84},
        'i2c_irq': {
            'I2C1': (31, 32), 'I2C2': (33, 34),
            'I2C3': (92, 93), 'I2C4': (82, 83),
        },
    },
    'STM32F405xx': {
        'name': 'f405',
        'base': 'stm32f405_base.repl',
        'script': 'ardupilot_f405.resc',
        'adcs': {'ADC1': 'adc1'},
        'spis': {'SPI1', 'SPI2', 'SPI3'},
        'i2cs': {'I2C1', 'I2C2', 'I2C3'},
        'uarts': {'USART1', 'USART2', 'USART3', 'UART4', 'UART5', 'USART6'},
        'sd_buses': {'SDIO': 'sdmmc'},
        'sd_fifo': {'SDIO': 0x40012C80},
        'can_buses': {'CAN1': 'can1', 'CAN2': 'can2'},
        'timers': {1, 2, 3, 4, 5, 8},
        'uart_irq': {
            'USART1': 37, 'USART2': 38, 'USART3': 39,
            'UART4': 52, 'UART5': 53, 'USART6': 71,
        },
        'spi_irq': {'SPI1': 35, 'SPI2': 36, 'SPI3': 51},
        'i2c_irq': {'I2C1': (31, 32), 'I2C2': (33, 34), 'I2C3': (72, 73)},
    },
    'STM32F407xx': {
        'name': 'f407',
        'base': 'stm32f405_base.repl',
        'script': 'ardupilot_f405.resc',
        'adcs': {'ADC1': 'adc1'},
        'spis': {'SPI1', 'SPI2', 'SPI3'},
        'i2cs': {'I2C1', 'I2C2', 'I2C3'},
        'uarts': {'USART1', 'USART2', 'USART3', 'UART4', 'UART5', 'USART6'},
        'sd_buses': {'SDIO': 'sdmmc'},
        'sd_fifo': {'SDIO': 0x40012C80},
        'can_buses': {'CAN1': 'can1', 'CAN2': 'can2'},
        'timers': {1, 2, 3, 4, 5, 8},
        'uart_irq': {
            'USART1': 37, 'USART2': 38, 'USART3': 39,
            'UART4': 52, 'UART5': 53, 'USART6': 71,
        },
        'spi_irq': {'SPI1': 35, 'SPI2': 36, 'SPI3': 51},
        'i2c_irq': {'I2C1': (31, 32), 'I2C2': (33, 34), 'I2C3': (72, 73)},
    },
    'STM32F427xx': {
        'name': 'f427',
        'base': 'stm32f427_base.repl',
        'script': 'ardupilot_f405.resc',
        'adcs': {'ADC1': 'adc1'},
        'spis': {'SPI1', 'SPI2', 'SPI3', 'SPI4', 'SPI5', 'SPI6'},
        'i2cs': {'I2C1', 'I2C2', 'I2C3'},
        'uarts': {
            'USART1', 'USART2', 'USART3', 'UART4', 'UART5',
            'USART6', 'UART7', 'UART8',
        },
        'sd_buses': {'SDIO': 'sdmmc'},
        'sd_fifo': {'SDIO': 0x40012C80},
        'can_buses': {'CAN1': 'can1', 'CAN2': 'can2'},
        'timers': {1, 2, 3, 4, 5, 8, 12},
        'uart_irq': {
            'USART1': 37, 'USART2': 38, 'USART3': 39,
            'UART4': 52, 'UART5': 53, 'USART6': 71,
            'UART7': 82, 'UART8': 83,
        },
        'spi_irq': {
            'SPI1': 35, 'SPI2': 36, 'SPI3': 51,
            'SPI4': 84, 'SPI5': 85, 'SPI6': 86,
        },
        'i2c_irq': {'I2C1': (31, 32), 'I2C2': (33, 34), 'I2C3': (72, 73)},
    },
    'STM32F767xx': {
        'name': 'f767',
        'base': 'stm32f767_base.repl',
        'script': 'ardupilot_f767.resc',
        'spis': {'SPI1', 'SPI2', 'SPI3', 'SPI4', 'SPI5', 'SPI6'},
        'i2cs': {'I2C1', 'I2C2', 'I2C3', 'I2C4'},
        'uarts': {
            'USART1', 'USART2', 'USART3', 'UART4', 'UART5',
            'USART6', 'UART7', 'UART8',
        },
        'sd_buses': {
            'SDMMC1': 'sdmmc', 'SDMMC2': 'sdmmc2', 'SDIO': 'sdmmc',
        },
        'sd_fifo': {
            'SDMMC1': 0x40012C80, 'SDMMC2': 0x40011C80,
            'SDIO': 0x40012C80,
        },
        'can_buses': {'CAN1': 'can1', 'CAN2': 'can2'},
        'timers': {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14},
        'uart_irq': {
            'USART1': 37, 'USART2': 38, 'USART3': 39,
            'UART4': 52, 'UART5': 53, 'USART6': 71,
            'UART7': 82, 'UART8': 83,
        },
        'spi_irq': {
            'SPI1': 35, 'SPI2': 36, 'SPI3': 51,
            'SPI4': 84, 'SPI5': 85, 'SPI6': 86,
        },
        'i2c_irq': {
            'I2C1': (31, 32), 'I2C2': (33, 34),
            'I2C3': (72, 73), 'I2C4': (95, 96),
        },
    },
    'STM32H743xx': {
        'name': 'h743',
        'base': 'stm32h743_base.repl',
        'script': 'ardupilot_h743.resc',
        'adcs': {'ADC1': 'adcM1S2'},
        'adc_existing': True,
        'adc_sample_shift': 4,
        'spis': {'SPI1', 'SPI2', 'SPI3', 'SPI4', 'SPI5', 'SPI6'},
        'i2cs': {'I2C1', 'I2C2', 'I2C3', 'I2C4'},
        'uarts': {
            'USART1', 'USART2', 'USART3', 'UART4', 'UART5',
            'USART6', 'UART7', 'UART8', 'LPUART1',
        },
        'sd_buses': {'SDMMC1': 'sdmmc', 'SDMMC2': 'sdmmc2'},
        'can_buses': {'CAN1': 'fdcan1', 'CAN2': 'fdcan2'},
        'timers': {1, 2, 3, 4, 5, 8, 12, 13, 14, 15},
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
    'STM32H757xx': {
        'name': 'h757',
        'base': 'stm32h743_base.repl',
        'script': 'ardupilot_h743.resc',
        'adcs': {'ADC1': 'adcM1S2'},
        'adc_existing': True,
        'adc_sample_shift': 4,
        'spis': {'SPI1', 'SPI2', 'SPI3', 'SPI4', 'SPI5', 'SPI6'},
        'i2cs': {'I2C1', 'I2C2', 'I2C3', 'I2C4'},
        'uarts': {
            'USART1', 'USART2', 'USART3', 'UART4', 'UART5',
            'USART6', 'UART7', 'UART8', 'LPUART1',
        },
        'sd_buses': {'SDMMC1': 'sdmmc', 'SDMMC2': 'sdmmc2'},
        'can_buses': {'CAN1': 'fdcan1', 'CAN2': 'fdcan2'},
        'timers': {1, 2, 3, 4, 5, 8, 12, 13, 14, 15},
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
    'STM32L431xx': {
        'name': 'l4',
        'base': 'stm32l4_base.repl',
        'script': 'ardupilot_l4.resc',
        'adcs': {'ADC1': 'adc1'},
        'adc_existing': True,
        'spis': {'SPI1', 'SPI2', 'SPI3'},
        'i2cs': {'I2C1', 'I2C2', 'I2C3', 'I2C4'},
        'uarts': {'USART1', 'USART2', 'USART3', 'UART4', 'LPUART1'},
        'sd_buses': {},
        'can_buses': {'CAN1': 'can1'},
        'timers': {1, 2, 3, 4, 5, 6, 7, 8, 15, 16, 17},
        'uart_irq': {
            'USART1': 37, 'USART2': 38, 'USART3': 39,
            'UART4': 52, 'LPUART1': 70,
        },
        'spi_irq': {'SPI1': 35, 'SPI2': 36, 'SPI3': 51},
        'i2c_irq': {
            'I2C1': (31, 32), 'I2C2': (33, 34),
            'I2C3': (72, 73), 'I2C4': (83, 84),
        },
    },
    'STM32F303xC': {
        'name': 'f303',
        'base': 'stm32f303_base.repl',
        'script': 'ardupilot_f303.resc',
        'spis': {'SPI1', 'SPI2', 'SPI3'},
        'i2cs': {'I2C1', 'I2C2', 'I2C3'},
        'uarts': {'USART1', 'USART2', 'USART3', 'UART4', 'UART5'},
        'sd_buses': {},
        'can_buses': {'CAN': 'can1'},
        'timers': {1, 2, 3, 4, 6, 7, 8, 15, 16, 17, 20},
        'uart_irq': {
            'USART1': 37, 'USART2': 38, 'USART3': 39,
            'UART4': 52, 'UART5': 53,
        },
        'spi_irq': {'SPI1': 35, 'SPI2': 36, 'SPI3': 51},
        'i2c_irq': {
            'I2C1': (31, 32), 'I2C2': (33, 34),
            'I2C3': (72, 73),
        },
    },
}

# The F732 peripherals used by AP_Periph are register-compatible with the
# existing F767 platform model.
FAMILIES['STM32F732xx'] = FAMILIES['STM32F767xx']
# F412 AP_Periph boards use the subset of F4 peripherals already modelled by
# the F405 platform.
FAMILIES['STM32F412Rx'] = dict(
    FAMILIES['STM32F405xx'], base='stm32f412_base.repl')
# G441 and G491 use the G4 peripheral instances exercised by the G474 model.
FAMILIES['STM32G441xx'] = FAMILIES['STM32G474xx']
FAMILIES['STM32G491xx'] = FAMILIES['STM32G474xx']
# F105 connectivity-line parts retain the F1 register layout while adding
# memory and peripherals.
FAMILIES['STM32F105xC'] = dict(
    FAMILIES['STM32F103xB'], name='f105', base='stm32f105_base.repl',
    spis={'SPI1', 'SPI2', 'SPI3'},
    spi_irq={'SPI1': 35, 'SPI2': 36, 'SPI3': 51})
# The CKS F407 and H723 targets use the peripheral subsets modelled by their
# corresponding STM32 family platforms.
FAMILIES['CKS32F407xx'] = FAMILIES['STM32F407xx']
FAMILIES['STM32H723xx'] = FAMILIES['STM32H743xx']
# The AP_Periph L4 targets use the same peripheral instances, with the L496
# additionally exposing UART5.
FAMILIES['STM32L476xx'] = FAMILIES['STM32L431xx']
FAMILIES['STM32L496xx'] = dict(
    FAMILIES['STM32L431xx'],
    uarts=FAMILIES['STM32L431xx']['uarts'] | {'UART5'},
    uart_irq=dict(FAMILIES['STM32L431xx']['uart_irq'], UART5=53))

IMU_MODELS = {
    'ADIS1647x': 'Sensors.AP_ADIS1647x',
    'ADIS16607': 'Sensors.AP_ADIS16607',
    'BMI055': 'Sensors.AP_RegisterIMU',
    'BMI088': 'Sensors.AP_RegisterIMU',
    'BMI160': 'Sensors.AP_RegisterIMU',
    'BMI270': 'Sensors.AP_RegisterIMU',
    'Invensense': 'Sensors.AP_ICM20689',
    'Invensensev2': 'Sensors.AP_InvensenseV2',
    'Invensensev3': 'Sensors.AP_ICM42688',
    'LSM9DS0': 'Sensors.AP_RegisterIMU',
    'LSM6DSV': 'Sensors.AP_RegisterIMU',
    'SCHA63T': 'Sensors.AP_SCHA63T',
}

WHOAMI_VALUES = {
    'MPU_WHOAMI_MPU60X0': 0x68,
    'MPU_WHOAMI_MPU9250': 0x71,
    'MPU_WHOAMI_ICM20608': 0xAF,
    'MPU_WHOAMI_ICM20602': 0x12,
    'MPU_WHOAMI_ICM20689': 0x98,
    'INV2_WHOAMI_ICM20948': 0xEA,
    'INV2_WHOAMI_ICM20649': 0xE1,
}

BARO_MODELS = {
    'AUAV': 'Sensors.AP_Baro_AUAV',
    'BMP085': 'Sensors.AP_BMP085',
    'BMP280': 'Sensors.AP_BMP280',
    'BMP388': 'Sensors.AP_BMP388',
    'BMP581': 'Sensors.AP_BMP581',
    'DPS280': 'Sensors.AP_DPS310',
    'DPS310': 'Sensors.AP_DPS310',
    'ICP201XX': 'Sensors.AP_ICP201XX',
    'LPS2XH': 'Sensors.AP_LPS2XH',
    'MS5611': 'Sensors.AP_MS5611',
    'SPL06': 'Sensors.AP_DPS310',
}

SPI_BARO_IDENTITIES = {
    'BMP280': (0x58, 0xD0),
    'BMP388': (0x50, 0x00),
    'BMP581': (0x50, 0x01),
    'DPS280': (0x10, 0x0D),
    'DPS310': (0x10, 0x0D),
    'LPS2XH': (0xB1, 0x0F),
    'SPL06': (0x10, 0x0D),
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


def _resolved_env(path, name, seen=None):
    '''Find an inherited hwdef env value while preserving include order.'''
    if seen is None:
        seen = set()
    path = path.resolve()
    if path in seen:
        return None
    seen.add(path)
    value = None
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
            included = _resolved_env(path.parent / fields[1], name, seen)
            if included is not None:
                value = included
        elif fields[0] == 'env' and len(fields) >= 3 and fields[1] == name:
            value = fields[2]
    return value


def supported_boards(root):
    '''Board name -> MCU for targets with enough hwdef data to run directly.'''
    found = {}
    for board_dir in sorted(_hwdef_root(root).iterdir()):
        app = board_dir / 'hwdef.dat'
        bootloader = board_dir / 'hwdef-bl.dat'
        if not app.is_file():
            continue
        is_periph = _resolved_env(app, 'AP_PERIPH') == '1'
        if not bootloader.is_file() and not is_periph:
            continue
        mcu = _resolved_mcu(app)
        if mcu in FAMILIES:
            found[board_dir.name] = mcu
    return found


def is_periph_board(root, board):
    return _resolved_env(_hwdef_root(root) / board / 'hwdef.dat', 'AP_PERIPH') == '1'


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
            # Board-specific definitions precede the guarded AP_Periph
            # fallbacks in generated hwdef.h.  The latter are only active
            # when the former do not exist, so retain the first definition.
            found.setdefault(fields[1], fields[2].strip())
    return found


def _define_enabled(defines, name):
    value = defines.get(name)
    if value is None:
        return False
    try:
        return _constant_integer(value) != 0
    except ValueError:
        return value not in ('FALSE', 'false')


def _constant_integer(expression):
    '''Evaluate an integer-only hwdef expression without executing code.'''
    operators = {
        ast.Add: lambda left, right: left + right,
        ast.Sub: lambda left, right: left - right,
        ast.Mult: lambda left, right: left * right,
        ast.FloorDiv: lambda left, right: left // right,
        ast.LShift: lambda left, right: left << right,
        ast.RShift: lambda left, right: left >> right,
        ast.BitOr: lambda left, right: left | right,
        ast.BitAnd: lambda left, right: left & right,
    }

    def evaluate(node):
        if isinstance(node, ast.Constant) and isinstance(node.value, int):
            return node.value
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            value = evaluate(node.operand)
            return value if isinstance(node.op, ast.UAdd) else -value
        if isinstance(node, ast.BinOp) and type(node.op) in operators:
            return operators[type(node.op)](evaluate(node.left), evaluate(node.right))
        raise ValueError('not a constant integer expression: %s' % expression)

    return evaluate(ast.parse(expression, mode='eval').body)


def _constant_number(expression):
    '''Evaluate the simple floating-point expressions emitted by hwdef.'''
    operators = {
        ast.Add: lambda left, right: left + right,
        ast.Sub: lambda left, right: left - right,
        ast.Mult: lambda left, right: left * right,
        ast.Div: lambda left, right: left / right,
    }

    def evaluate(node):
        if (isinstance(node, ast.Constant) and
                isinstance(node.value, (int, float))):
            return node.value
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            value = evaluate(node.operand)
            return value if isinstance(node.op, ast.UAdd) else -value
        if isinstance(node, ast.BinOp) and type(node.op) in operators:
            return operators[type(node.op)](evaluate(node.left), evaluate(node.right))
        raise ValueError('not a constant numeric expression: %s' % expression)

    cleaned = re.sub(r'(?<=\d)[fFuUlL]+\b', '', expression)
    return float(evaluate(ast.parse(cleaned, mode='eval').body))


def _analog_pins(path):
    '''Return logical analog pin -> (ADC channel, volts/count).'''
    pins = {}
    in_macro = False
    for line in path.read_text().splitlines():
        if line.startswith('#define HAL_ANALOG_PINS'):
            in_macro = True
            continue
        if not in_macro:
            continue
        match = re.match(
            r'\s*\{\s*(\d+)\s*,\s*(\d+)\s*,\s*([^}]+)\}', line)
        if match is None:
            break
        channel, logical_pin, scale = match.groups()
        pins[int(logical_pin)] = (int(channel), _constant_number(scale.strip()))
    return pins


def _default_parameters(path):
    parameters = {}
    if not path.is_file():
        return parameters
    for line in path.read_text().splitlines():
        fields = line.split('#', 1)[0].split()
        if len(fields) >= 2:
            parameters[fields[0]] = fields[1]
    return parameters


def _crc32_small(data, padded_size):
    '''Match AP_Math crc32_small(), including erased IO flash padding.'''
    if len(data) > padded_size:
        raise ValueError('IO firmware is larger than AP_IOMCU_FW_FLASH_SIZE')
    crc = 0
    for value in data + b'\xff' * (padded_size - len(data)):
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc


def _dma_stream(value):
    match = re.fullmatch(r'STM32_DMA_STREAM_ID\((\d+),(\d+)\)', value.replace(' ', ''))
    if match is None:
        return None
    return int(match.group(1)), int(match.group(2))


def _imu_whoami(device_name, defines):
    '''Find the board-validation WHOAMI for a named SPI sensor, if present.'''
    validation = defines.get('HAL_VALIDATE_BOARD', '')
    match = re.search(
        r'spi_check_register(?:_inv2)?\(\s*"%s"\s*,[^,]+,\s*([A-Za-z0-9_xX]+)' %
        re.escape(device_name), validation)
    if match is not None:
        value = match.group(1)
        if value in WHOAMI_VALUES:
            return WHOAMI_VALUES[value]
        try:
            return int(value, 0)
        except ValueError:
            pass
    names = {
        'icm20602': 0x12,
        'icm20608': 0xAF,
        'icm20689': 0x98,
        'icm20948': 0xEA,
        'icm20649': 0xE1,
        'icm42670': 0x67,
        'icm42688': 0x47,
        'icm45686': 0xE9,
        'mpu6000': 0x68,
        'mpu9250': 0x71,
    }
    for part, value in names.items():
        if part in device_name.lower():
            return value
    return None


def _imu_children(imu, resolved_devices, defines, name):
    '''Return model/device/property tuples for one resolved IMU declaration.'''
    driver = imu[0]
    model = IMU_MODELS[driver]
    if driver == 'SCHA63T':
        return [('%sPart%d' % (name, index), model, bus, cs, [])
                for index, (_, bus, cs) in enumerate(resolved_devices)]
    if driver in ('BMI055', 'BMI088', 'LSM9DS0'):
        children = []
        for index, (device_name, bus, cs) in enumerate(resolved_devices):
            is_accel = index == 0 or device_name.lower().endswith(('_a', 'accel'))
            if driver == 'LSM9DS0':
                whoami = 0x49 if index else 0xD4
            else:
                whoami = (0xFA if driver == 'BMI055' else 0x1E) if is_accel else 0x0F
            children.append((
                '%s%s' % (name, 'Accel' if is_accel else 'Gyro'),
                model, bus, cs, ['    whoAmI: 0x%02X' % whoami]))
        return children

    device_name, bus, cs = resolved_devices[0]
    properties = []
    fixed_whoami = {
        'BMI160': 0xD1,
        'BMI270': 0x24,
        'LSM6DSV': 0x70,
    }.get(driver)
    whoami = fixed_whoami if fixed_whoami is not None else _imu_whoami(
        device_name, defines)
    if whoami is not None:
        properties.append('    whoAmI: 0x%02X' % whoami)
    return [(name, model, bus, cs, properties)]


def _dmamux_requests(root):
    path = (root / 'modules' / 'ChibiOS' / 'os' / 'hal' / 'ports' /
            'STM32' / 'STM32H7xx' / 'stm32_dmamux.h')
    requests = {}
    for line in path.read_text().splitlines():
        match = re.match(r'#define\s+(STM32_DMAMUX1_\w+)\s+(\d+)', line)
        if match:
            requests[match.group(1)] = int(match.group(2))
    return requests


def _g474_dmamux_requests(root):
    path = (root / 'modules' / 'ChibiOS' / 'os' / 'hal' / 'ports' /
            'STM32' / 'STM32G4xx' / 'stm32_dmamux.h')
    requests = {}
    for line in path.read_text().splitlines():
        match = re.match(r'#define\s+(STM32_DMAMUX1_\w+)\s+(\d+)', line)
        if match:
            requests[match.group(1)] = int(match.group(2))
    return requests


def _safe_name(value):
    return re.sub(r'[^a-zA-Z0-9_]', '_', value)


def _spi_device_names(expression):
    matches = re.findall(r'get_device\("([^"\n]+)"\)', expression)
    if matches:
        return matches
    match = re.fullmatch(r'SPI:([^\s]+)', expression)
    return [match.group(1)] if match else []


def _i2c_device(expression):
    match = re.fullmatch(r'GET_I2C_DEVICE\((\d+),\s*(0x[0-9A-Fa-f]+|\d+)\)', expression)
    if match is None:
        return None
    return int(match.group(1)), int(match.group(2), 0)


def _sensor_devices(config, family, defines, fram_path, warnings):
    '''Return (REPL declarations, CS pin -> peripheral names).'''
    spi_devices = {dev[0]: dev for dev in config.spidev}
    declarations = []
    chip_selects = {}
    spi_children = []

    imu_locations = set()
    for index, imu in enumerate(config.imu_list):
        if len(imu) < 2:
            warnings.append('unmodelled IMU: %s' % ' '.join(imu))
            continue
        device_names = []
        for argument in imu[1:]:
            for device_name in _spi_device_names(argument):
                if device_name not in spi_devices:
                    aliases = [device_name.removesuffix('_cs')]
                    device_name = next(
                        (alias for alias in aliases if alias in spi_devices),
                        device_name)
                if device_name in spi_devices and device_name not in device_names:
                    device_names.append(device_name)
        if not device_names:
            i2c_devices = [device for device in map(_i2c_device, imu[1:])
                           if device is not None]
            if i2c_devices and imu[0] in ('BMI055', 'BMI088', 'Invensense'):
                i2c_order = config.get_config(
                    'I2C_ORDER', required=False, aslist=True) or []
                resolved_i2c = []
                for device_index, (bus_index, address) in enumerate(i2c_devices):
                    if not 0 <= bus_index < len(i2c_order):
                        continue
                    bus = i2c_order[bus_index]
                    if bus not in family['i2cs']:
                        continue
                    location = (bus, address)
                    if location in imu_locations:
                        continue
                    imu_locations.add(location)
                    whoami = 0x0F
                    if device_index == 0:
                        whoami = 0xFA if imu[0] == 'BMI055' else 0x1E
                    if imu[0] == 'Invensense':
                        whoami = 0x68
                    resolved_i2c.append((bus, address, whoami))
                for device_index, (bus, address, whoami) in enumerate(resolved_i2c):
                    declarations += [
                        'imu%dI2C%d: Sensors.AP_I2CRegisterIMU @ %s 0x%02X' %
                        (index, device_index, bus.lower(), address),
                        '    whoAmI: 0x%02X' % whoami,
                        '',
                    ]
                continue
            warnings.append('cannot resolve IMU SPI device: %s' % ' '.join(imu))
            continue
        locations = []
        resolved_devices = []
        for device_name in device_names:
            device = spi_devices[device_name]
            bus, cs_label = device[1], device[3]
            if bus not in family['spis']:
                warnings.append('%s is not present in the current MCU base' % bus)
                break
            cs = config.bylabel.get(cs_label)
            if cs is None:
                warnings.append('cannot resolve chip select %s' % cs_label)
                break
            locations.append((bus, cs.port, cs.pin))
            resolved_devices.append((device_name, bus, cs))
        if len(resolved_devices) != len(device_names):
            continue
        if imu[0] not in IMU_MODELS:
            warnings.append('unmodelled IMU: %s' % ' '.join(imu))
            continue
        if any(location in imu_locations for location in locations):
            continue
        imu_locations.update(locations)
        name = 'imu%d' % index
        spi_children += _imu_children(imu, resolved_devices, defines, name)

    i2c_order = config.get_config('I2C_ORDER', required=False, aslist=True) or []
    baro_locations = set()
    for index, baro in enumerate(config.baro_list):
        if not baro.devlist:
            warnings.append('unsupported barometer bus: %s' % baro.driver)
            continue
        device = baro.devlist[0]
        i2c_declaration = None
        cs = None
        if hasattr(device, 'busnum') and hasattr(device, 'busaddr'):
            if not 0 <= device.busnum < len(i2c_order):
                warnings.append('invalid I2C index for barometer %s' % baro.driver)
                continue
            bus = i2c_order[device.busnum]
            if bus not in family['i2cs']:
                warnings.append('%s is not present in the current MCU base' % bus)
                continue
            location = (bus, device.busaddr)
            i2c_declaration = 'baro%d: %%s @ %s 0x%02X' % (
                index, bus.lower(), device.busaddr)
        elif hasattr(device, 'name'):
            spi_device = spi_devices.get(device.name)
            if spi_device is None:
                warnings.append('cannot resolve barometer SPI device: %s' % device.name)
                continue
            bus, cs_label = spi_device[1], spi_device[3]
            if bus not in family['spis']:
                warnings.append('%s is not present in the current MCU base' % bus)
                continue
            cs = config.bylabel.get(cs_label)
            if cs is None:
                warnings.append('cannot resolve chip select %s' % cs_label)
                continue
            location = (bus, cs.port, cs.pin)
        else:
            warnings.append('unsupported barometer bus: %s' % baro.driver)
            continue
        if location in baro_locations:
            continue
        baro_locations.add(location)
        model = BARO_MODELS.get(baro.driver)
        if model is None:
            warnings.append('unmodelled barometer: %s' % baro.driver)
            continue
        name = 'baro%d' % index
        if cs is not None:
            properties = []
            identity = SPI_BARO_IDENTITIES.get(baro.driver)
            if identity is not None:
                model = 'Sensors.AP_SPIBarometer'
                properties = [
                    '    chipId: 0x%02X' % identity[0],
                    '    chipIdRegister: 0x%02X' % identity[1],
                ]
            spi_children.append((name, model, bus, cs, properties))
        else:
            declarations += [i2c_declaration % model, '']

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
                spi_children.append((
                    'fram', 'Miscellaneous.AP_RAMTRON', bus, cs,
                    ['    fileName: %s' % json.dumps(str(fram_path))]))

    children_by_bus = {}
    for child in spi_children:
        children_by_bus.setdefault(child[2], []).append(child)
    for bus in sorted(children_by_bus):
        mux = '%sMux' % bus.lower()
        children = children_by_bus[bus]
        declarations += [
            '%s: Miscellaneous.AP_SPIMultiplexer @ %s' % (mux, bus.lower()),
        ]
        if all(child[1] != 'Miscellaneous.AP_RAMTRON' for child in children):
            declarations.append('    frameOnTransfer: true')
        declarations.append('')
        for address, (name, model, _, cs, properties) in enumerate(children):
            declarations += [
                '%s: %s @ %s %d' % (name, model, mux, address),
            ] + properties + ['']
            chip_selects.setdefault((cs.port, cs.pin), []).append(
                '%s@%d' % (mux, address))

    return declarations, chip_selects, has_fram


def _iomcu_device(root, app, family, defines, address, warnings):
    uart = app.get_config('IOMCU_UART', required=False, default=None)
    if uart is None:
        return [], None
    if uart not in family['uarts']:
        warnings.append('%s IOMCU UART is not present in the current MCU base' % uart)
        return [], None
    firmware = app.romfs.get('io_firmware.bin')
    if firmware is None:
        warnings.append('IOMCU_UART is set without ROMFS io_firmware.bin')
        return [], None
    firmware_path = Path(firmware)
    if not firmware_path.is_absolute():
        firmware_path = root / firmware_path
    try:
        data = firmware_path.read_bytes()
        flash_size = _constant_integer(
            defines.get('AP_IOMCU_FW_FLASH_SIZE', '0xF000'))
        firmware_crc = _crc32_small(data, flash_size)
    except (OSError, SyntaxError, ValueError, ZeroDivisionError) as error:
        warnings.append('cannot configure IOMCU firmware CRC: %s' % error)
        return [], None
    return [
        'iomcu: Miscellaneous.AP_IOMCU @ sysbus 0x%08X' % address,
        '    firmwareCrc: 0x%08X' % firmware_crc,
        '',
    ], uart


def _gpio_routes(family_name, chip_selects):
    lines = []
    ports = sorted({port for port, _ in chip_selects})
    for port in ports:
        lines += ['gpioPort%s:' % port]
        for pin in range(16):
            targets = list(chip_selects.get((port, pin), []))
            if family_name in ('f103', 'f105', 'f405', 'f407', 'f427'):
                targets.append('exti@%d' % pin)
            else:
                targets.append('syscfg#%d@%d' % (ord(port) - ord('A'), pin))
            lines.append('    %d -> %s' % (pin, ' | '.join(targets)))
        lines.append('')
    return lines


def _f405_dma_wiring(defines, family, alloc, warnings):
    lines = []
    rx_uarts = []

    if _define_enabled(defines, 'AP_PERIPH_BATTERY_ENABLED'):
        for peripheral, model in family.get('adcs', {}).items():
            value = defines.get('STM32_ADC_%s_DMA_STREAM' % peripheral)
            stream = _dma_stream(value) if value else None
            if stream is None:
                continue
            dma, channel = stream
            lines += [
                '%s:' % model,
                '    DMARequest -> dma%d@%d' % (dma, channel),
                '',
                '%sCircularDma: Miscellaneous.AP_STM32DMA_Circular @ '
                'sysbus 0x%08X' % (model, alloc()),
                '    dma: dma%d' % dma,
                '    stream: %d' % channel,
                '',
            ]

    for bus, peripheral in family['sd_buses'].items():
        value = defines.get('STM32_SDC_%s_DMA_STREAM' % bus)
        stream = _dma_stream(value) if value else None
        if stream is None:
            continue
        dma, channel = stream
        lines += [
            '%s:' % peripheral,
            '    DMAReceive -> dma%d@%d' % (dma, channel),
            '',
            '%sDmaPump: Miscellaneous.AP_STM32F_SDMMC_DmaPump @ sysbus 0x%08X' %
            (peripheral, alloc()),
            '    sdmmc: %s' % peripheral,
            '    dma: dma%d' % dma,
            '    stream: %d' % channel,
            '    fifoAddress: 0x%08X' % family['sd_fifo'][bus],
            '',
        ]

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


def _f767_dma_wiring(defines, family, alloc, warnings):
    lines = []

    for bus, peripheral in family['sd_buses'].items():
        value = defines.get('STM32_SDC_%s_DMA_STREAM' % bus)
        stream = _dma_stream(value) if value else None
        if stream is None:
            continue
        dma, channel = stream
        lines += [
            '%s:' % peripheral,
            '    DMAReceive -> dma%d@%d' % (dma, channel),
            '',
            '%sDmaPump: Miscellaneous.AP_STM32F_SDMMC_DmaPump @ sysbus 0x%08X' %
            (peripheral, alloc()),
            '    sdmmc: %s' % peripheral,
            '    dma: dma%d' % dma,
            '    stream: %d' % channel,
            '    fifoAddress: 0x%08X' % family['sd_fifo'][bus],
            '',
        ]

    for peripheral in family['uart_irq']:
        value = defines.get('STM32_UART_%s_RX_DMA_STREAM' % peripheral)
        stream = _dma_stream(value) if value else None
        if stream is None:
            continue
        dma, channel = stream
        lines += [
            '%s:' % peripheral.lower(),
            '    ReceiveDmaRequest -> dma%d@%d' % (dma, channel),
            '',
            '%sIdle: Miscellaneous.AP_STM32F7_USART_Idle @ sysbus 0x%08X' %
            (peripheral.lower(), alloc()),
            '    uart: %s' % peripheral.lower(),
            '',
        ]

    for peripheral in family['spi_irq']:
        value = defines.get('STM32_SPI_%s_RX_DMA_STREAM' % peripheral)
        stream = _dma_stream(value) if value else None
        if stream is None:
            continue
        dma, channel = stream
        lines += [
            '%s:' % peripheral.lower(),
            '    DMARecieve -> dma%d@%d' % (dma, channel),
            '',
        ]

    for peripheral in family['i2c_irq']:
        value = defines.get('STM32_I2C_%s_RX_DMA_STREAM' % peripheral)
        stream = _dma_stream(value) if value else None
        if stream is None:
            continue
        dma, channel = stream
        lines += [
            '%s:' % peripheral.lower(),
            '    RxDmaRequest -> dma%d@%d' % (dma, channel),
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

    for peripheral, model in family.get('adcs', {}).items():
        channel = defines.get('STM32_ADC_%s_DMA_CHAN' % peripheral)
        request = requests.get(channel)
        if request is None:
            continue
        lines += [
            '%s:' % model,
            '    DMARequest -> dmamux1@%d' % request,
            '',
        ]
        stream = _dma_stream(defines.get(
            'STM32_ADC_%s_DMA_STREAM' % peripheral, ''))
        if stream is not None:
            dma, dma_stream = stream
            lines += [
                '%sCircularDma: Miscellaneous.AP_STM32DMA_Circular @ '
                'sysbus 0x%08X' % (model, alloc()),
                '    dma: dma%d' % dma,
                '    stream: %d' % dma_stream,
                '',
            ]

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


def _g474_dma_wiring(root, defines, family, alloc, warnings):
    lines = []
    requests = _g474_dmamux_requests(root)

    for peripheral in sorted(family['i2cs']):
        if 'STM32_I2C_%s_DMA_CHANNEL' % peripheral not in defines:
            continue
        rx_request = requests.get('STM32_DMAMUX1_%s_RX' % peripheral)
        tx_request = requests.get('STM32_DMAMUX1_%s_TX' % peripheral)
        if rx_request is None or tx_request is None:
            warnings.append('cannot resolve %s DMAMUX requests' % peripheral)
            continue
        lines += [
            '%s:' % peripheral.lower(),
            '    RxDmaRequest -> dmamux1@%d' % rx_request,
            '    TxDmaRequest -> dmamux1@%d' % tx_request,
            '',
        ]

    for peripheral, irq in family['uart_irq'].items():
        channel = defines.get('STM32_UART_%s_RX_DMA_CHAN' % peripheral)
        request = requests.get(channel)
        if request is None:
            continue
        lines += [
            '%s:' % peripheral.lower(),
            '    IRQ -> nvic@%d' % irq,
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

    return lines


def _l4_dma_wiring(defines, family, alloc, warnings):
    lines = []

    for peripheral, model in family.get('adcs', {}).items():
        value = defines.get('STM32_ADC_%s_DMA_STREAM' % peripheral)
        stream = _dma_stream(value) if value else None
        if stream is None:
            continue
        dma, channel = stream
        lines += [
            '%s:' % model,
            '    DMARequest -> dma%d@%d' % (dma, channel),
            '',
        ]

    for peripheral in sorted(family['i2cs']):
        rx_value = defines.get('STM32_I2C_%s_RX_DMA_STREAM' % peripheral)
        tx_value = defines.get('STM32_I2C_%s_TX_DMA_STREAM' % peripheral)
        rx_stream = _dma_stream(rx_value) if rx_value else None
        tx_stream = _dma_stream(tx_value) if tx_value else None
        if rx_stream is None or tx_stream is None:
            continue
        rx_dma, rx_channel = rx_stream
        tx_dma, tx_channel = tx_stream
        lines += [
            '%s:' % peripheral.lower(),
            '    RxDmaRequest -> dma%d@%d' % (rx_dma, rx_channel),
            '    TxDmaRequest -> dma%d@%d' % (tx_dma, tx_channel),
            '',
        ]

    for peripheral in family['uart_irq']:
        value = defines.get('STM32_UART_%s_RX_DMA_STREAM' % peripheral)
        stream = _dma_stream(value) if value else None
        if stream is None:
            continue
        dma, channel = stream
        lines += [
            '%s:' % peripheral.lower(),
            '    ReceiveDmaRequest -> dma%d@%d' % (dma, channel),
            '',
            '%sIdle: Miscellaneous.AP_STM32F7_USART_Idle @ sysbus 0x%08X' %
            (peripheral.lower(), alloc()),
            '    uart: %s' % peripheral.lower(),
            '',
        ]

    for peripheral in family['spi_irq']:
        value = defines.get('STM32_SPI_%s_RX_DMA_STREAM' % peripheral)
        stream = _dma_stream(value) if value else None
        if stream is None:
            continue
        dma, channel = stream
        lines += [
            '%s:' % peripheral.lower(),
            '    DMARecieve -> dma%d@%d' % (dma, channel),
            '',
        ]

    return lines


def _platform(root, board, app, outdir, fram_path, is_periph, warnings):
    family = FAMILIES[app.mcu_type]
    base = root / 'Tools' / 'renode' / 'platforms' / family['base']
    lines = [
        '// GENERATED from libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat.' % board,
        '// Edit the hwdef or Tools/renode/gen_board.py, not this file.',
        '// Primary application RAM is 0x%08X (%u KiB).' %
        (app.get_ram_map()[0][0], app.get_ram_map()[0][1]),
        '',
        'using "%s"' % base,
        '',
    ]
    hwdef_h = outdir / 'hwdef' / 'hwdef.h'
    defines = _defines(hwdef_h)
    defaults = _default_parameters(
        outdir / 'hwdef' / 'processed_defaults.parm')
    sensor_lines, chip_selects, has_fram = _sensor_devices(
        app, family, defines, fram_path, warnings)
    lines += sensor_lines
    address = 0x60000010
    iomcu_lines, iomcu_uart = _iomcu_device(
        root, app, family, defines, address, warnings)
    lines += iomcu_lines
    if iomcu_uart is not None:
        address += 4

    def alloc(size=4):
        nonlocal address
        value = address
        address += size
        return value

    def periph_uart(port_name, port_value):
        index = _constant_integer(port_value)
        if index < 0:
            return None
        serial_order = app.get_config(
            'SERIAL_ORDER', required=False, aslist=True) or []
        if not 0 <= index < len(serial_order):
            warnings.append('%s selects missing SERIAL%d' % (port_name, index))
            return None
        uart = serial_order[index]
        if uart not in family['uarts']:
            warnings.append('%s selects unsupported %s' % (port_name, uart))
            return None
        return uart

    gps_uart = None
    if is_periph and _define_enabled(defines, 'AP_PERIPH_GPS_ENABLED'):
        if 'GPS_PORT' in defaults:
            gps_uart = periph_uart('GPS_PORT', defaults['GPS_PORT'])
        elif 'HAL_PERIPH_GPS_PORT_DEFAULT' in defines:
            gps_uart = periph_uart(
                'GPS_PORT', defines['HAL_PERIPH_GPS_PORT_DEFAULT'])
        else:
            serial_order = app.get_config(
                'SERIAL_ORDER', required=False, aslist=True) or []
            gps_uart = next((uart for uart in reversed(serial_order)
                             if uart in family['uarts']), None)
        if gps_uart is not None:
            lines += [
                'gps: Sensors.AP_UBlox @ sysbus 0x%08X' % alloc(),
                '',
            ]

    airspeed_bus = None
    if is_periph and _define_enabled(defines, 'AP_PERIPH_AIRSPEED_ENABLED'):
        i2c_order = app.get_config(
            'I2C_ORDER', required=False, aslist=True) or []
        airspeed_index = _constant_integer(defaults.get(
            'ARSPD_BUS', defines.get('HAL_AIRSPEED_BUS_DEFAULT', '0')))
        airspeed_type = _constant_integer(defaults.get(
            'ARSPD_TYPE', defines.get('HAL_AIRSPEED_TYPE_DEFAULT', '1')))
        if not 0 <= airspeed_index < len(i2c_order):
            warnings.append('airspeed bus %d is not present in I2C_ORDER' %
                            airspeed_index)
        elif i2c_order[airspeed_index] not in family['i2cs']:
            warnings.append('airspeed bus selects unsupported %s' %
                            i2c_order[airspeed_index])
        elif airspeed_type in (1, 7, 9, 10, 11, 12):
            airspeed_bus = i2c_order[airspeed_index]
            lines += [
                'airspeed: Sensors.AP_Airspeed @ %s 0x28' %
                airspeed_bus.lower(),
                '',
            ]
        elif airspeed_type == 15:
            airspeed_bus = i2c_order[airspeed_index]
            lines += [
                'airspeed: Sensors.AP_Airspeed_ASP5033 @ %s 0x6C' %
                airspeed_bus.lower(),
                '',
            ]
        elif airspeed_type in (17, 18, 19):
            airspeed_bus = i2c_order[airspeed_index]
            lines += [
                'airspeed: Sensors.AP_Airspeed_AUAV @ %s 0x26' %
                airspeed_bus.lower(),
                '',
            ]
        elif airspeed_type == 0:
            pass
        else:
            warnings.append('unmodelled airspeed type %d' % airspeed_type)

    rangefinder_uart = None
    rangefinder_type = None
    if is_periph and _define_enabled(defines, 'AP_PERIPH_RANGEFINDER_ENABLED'):
        rangefinder_uart = periph_uart(
            'RNGFND_PORT', defaults.get(
                'RNGFND_PORT', defines.get(
                    'AP_PERIPH_RANGEFINDER_PORT_DEFAULT', '3')))
        rangefinder_type = _constant_integer(
            defaults.get('RNGFND1_TYPE', '0'))
        if rangefinder_uart is not None:
            model = ('Sensors.AP_LightWare' if rangefinder_type == 8 else
                     'Sensors.AP_Benewake')
            lines += [
                'rangefinder: %s @ sysbus 0x%08X' % (model, alloc()),
                '',
            ]

    battery_samples = []
    if is_periph and _define_enabled(defines, 'AP_PERIPH_BATTERY_ENABLED'):
        analog_pins = _analog_pins(hwdef_h)
        battery_inputs = []
        adc_disabled = any(
            name in defines and not _define_enabled(defines, name)
            for name in ('HAL_USE_ADC', 'STM32_ADC_USE_ADC1'))
        if not adc_disabled:
            max_instances = _constant_integer(defines.get(
                'AP_BATT_MONITOR_MAX_INSTANCES', '1'))
            if any(name.startswith('HAL_BATT2_') for name in defines):
                max_instances = max(2, max_instances)
            for index in range(max_instances):
                instance = '' if index == 0 else str(index + 1)
                parameter = 'BATT%s_' % instance
                compile_default = 'HAL_BATT%s_' % instance
                battery_type = _constant_integer(defaults.get(
                    parameter + 'MONITOR', defines.get(
                        compile_default + 'MONITOR_DEFAULT', '0')))
                for kind, types, pin_suffix, scale_suffix, fallback_pin, \
                        fallback_scale, desired in (
                            ('voltage', (3, 4, 25), 'VOLT_PIN', 'VOLT_SCALE',
                             '4', '10.1', 24.0),
                            ('current', (4, 31), 'CURR_PIN', 'CURR_SCALE',
                             '3', '17.0', 10.0)):
                    runtime_pin = parameter + pin_suffix
                    compile_pin = compile_default + pin_suffix
                    enabled = (battery_type in types or runtime_pin in defaults or
                               compile_pin in defines)
                    if not enabled:
                        continue
                    pin_value = defaults.get(
                        runtime_pin, defines.get(compile_pin, fallback_pin))
                    runtime_scale = parameter + (
                        'VOLT_MULT' if kind == 'voltage' else 'AMP_PERVLT')
                    compile_scale = compile_default + scale_suffix
                    scale_value = defaults.get(
                        runtime_scale, defines.get(compile_scale, fallback_scale))
                    offset = 0.0
                    if kind == 'current':
                        offset = _constant_number(defaults.get(
                            parameter + 'AMP_OFFSET', defines.get(
                                'AP_BATT%s_CURR_AMP_OFFSET_DEFAULT' % instance,
                                '0')))
                    battery_inputs.append((
                        kind + instance, pin_value, scale_value, desired, offset))

        adcs = family.get('adcs', {})
        if battery_inputs and 'ADC1' not in adcs:
            warnings.append('battery ADC is not modelled for %s' % app.mcu_type)
        elif battery_inputs:
            if not family.get('adc_existing'):
                lines += [
                    '%s: Analog.AP_STM32_ADC @ sysbus 0x%08X' %
                    (adcs['ADC1'], family.get('adc_base', 0x40012000)),
                    '    IRQ -> nvic@%d' % family.get('adc_irq', 18),
                    '',
                ]
            for name, pin_value, scale_value, desired, offset in battery_inputs:
                logical_pin = _constant_integer(pin_value)
                if logical_pin < 0:
                    continue
                analog = analog_pins.get(logical_pin)
                if analog is None:
                    warnings.append('battery %s selects unmapped analog pin %d' %
                                    (name, logical_pin))
                    continue
                channel, volts_per_count = analog
                multiplier = _constant_number(scale_value)
                sensor_voltage = desired / multiplier + offset
                if name.startswith('voltage'):
                    sensor_voltage = min(sensor_voltage, 2.5)
                raw = round(sensor_voltage / volts_per_count)
                if not 0 <= raw <= 4095:
                    warnings.append('simulated battery %s is outside ADC range' %
                                    name)
                    raw = min(4095, max(0, raw))
                raw <<= family.get('adc_sample_shift', 0)
                battery_samples.append((name, adcs['ADC1'], channel, raw))

    can_buses = []
    for bus, peripheral in family.get('can_buses', {}).items():
        if bus not in app.bytype:
            continue
        can_buses.append((bus, peripheral))
        lines += [
            '%sMcast: CAN.AP_CANMcast @ sysbus 0x%08X' %
            (bus.lower(), alloc(0x100)),
            '',
        ]

    has_ethernet = family['name'] in ('h743', 'h757') and 'ETH1' in app.bytype
    if has_ethernet:
        lines += [
            'ethernet: Network.SynopsysDWCEthernetQualityOfService @ {',
            '    sysbus 0x40028000;',
            '    sysbus new Bus.BusMultiRegistration { address: 0x40028C00; '
            'size: 0x200; region: "mtl" };',
            '    sysbus new Bus.BusMultiRegistration { address: 0x40029000; '
            'size: 0x200; region: "dma" }',
            '    }',
            '    systemClockFrequency: 50000000',
            '    dmaBusWidth: BusWidth.Bits32',
            '    -> nvic@61',
            '',
            'ethernetFixup: Miscellaneous.AP_STM32H7_Ethernet @ sysbus 0x%08X' %
            alloc(4),
            '    ethernet: ethernet',
            '',
            'ethernetPhy: Network.EthernetPhysicalLayer @ ethernet 0',
            '    BasicControl: 0x3100',
            '    BasicStatus: 0x782D',
            '    Id1: 0x0007',
            '    Id2: 0xC130',
            '    AutoNegotiationAdvertisement: 0x01E1',
            '    AutoNegotiationLinkPartnerBasePageAbility: 0x0001',
            '    AutoNegotiationExpansion: 0x0064',
            '    AutoNegotiationNextPageTransmit: 0x2001',
            '',
        ]

    if family['name'] not in ('f103', 'f105', 'f303', 'g474', 'l4'):
        lines += [
            'dma1Fix: Miscellaneous.AP_STM32DMA_Fixup @ sysbus 0x%08X' % alloc(),
            '    dma: dma1',
            '',
            'dma2Fix: Miscellaneous.AP_STM32DMA_Fixup @ sysbus 0x%08X' % alloc(),
            '    dma: dma2',
            '',
        ]
    if family['name'] in ('f405', 'f407', 'f427'):
        lines += _f405_dma_wiring(defines, family, alloc, warnings)
    elif family['name'] == 'f767':
        lines += _f767_dma_wiring(defines, family, alloc, warnings)
    elif family['name'] == 'g474':
        lines += _g474_dma_wiring(root, defines, family, alloc, warnings)
    elif family['name'] in ('f103', 'f105', 'f303', 'l4'):
        lines += _l4_dma_wiring(defines, family, alloc, warnings)
    elif family['name'] in ('h743', 'h757'):
        lines += _h743_dma_wiring(root, defines, family, alloc, warnings)
    if family['name'] in ('h743', 'h757', 'f303', 'f767', 'g474', 'l4'):
        serial_uarts = app.get_config('SERIAL_ORDER', required=False, aslist=True) or []
        for peripheral in dict.fromkeys(serial_uarts):
            if peripheral not in family['uarts']:
                continue
            lines += [
                '%sHost: Miscellaneous.AP_UARTPacer @ sysbus 0x%08X' %
                (peripheral.lower(), alloc()),
                '    uart: %s' % peripheral.lower(),
                '',
            ]
    lines += _gpio_routes(family['name'], chip_selects)
    return ('\n'.join(lines).rstrip() + '\n', has_fram, iomcu_uart,
            can_buses, has_ethernet, gps_uart, airspeed_bus, battery_samples,
            rangefinder_uart, rangefinder_type)


def _serial_device(app, family, serial_index, excluded=None):
    order = app.get_config('SERIAL_ORDER', required=False, aslist=True) or []
    excluded = set(excluded or ())
    requested_index = serial_index
    if serial_index is not None:
        if not 0 <= serial_index < len(order):
            raise ValueError('SERIAL%d is outside SERIAL_ORDER' % serial_index)
        device = order[serial_index]
    else:
        serial_index = next((index for index, entry in enumerate(order)
                             if (entry in family['uarts'] and
                                 entry not in excluded)), None)
        device = order[serial_index] if serial_index is not None else None
    if device is None and requested_index is None:
        return None, None
    if device is None or device in {'EMPTY', 'OTG1', 'OTG2'}:
        raise ValueError('selected serial port is not a UART')
    if device not in family['uarts']:
        raise ValueError('%s is not present in the current MCU base' % device)
    return serial_index, device


def _script(root, board, app, bootloader, platform, serial_index, uart_port,
            iomcu_uart, can_buses, has_ethernet, gps_uart, airspeed_bus,
            battery_samples, rangefinder_uart, rangefinder_type, warnings,
            quiet_peripherals=True):
    family = FAMILIES[app.mcu_type]
    reserve_kb = app.get_config('FLASH_RESERVE_START_KB', default=0, type=int)
    boot_kb = bootloader.get_config(
        'FLASH_BOOTLOADER_LOAD_KB', required=False, default=reserve_kb, type=int)
    # Older F4 hwdefs omit FLASH_RESERVE_START_KB from the application file.
    # In that case the bootloader load offset is the authoritative application
    # base, as it is in the generated ChibiOS build configuration.
    effective_reserve_kb = boot_kb if reserve_kb == 0 else reserve_kb
    app_base = 0x08000000 + effective_reserve_kb * 1024
    if boot_kb != reserve_kb and reserve_kb != 0:
        warnings.append('application starts at %uK, bootloader loads at %uK' %
                        (reserve_kb, boot_kb))
    serial_index, serial = _serial_device(
        app, family, serial_index,
        excluded=(gps_uart, rangefinder_uart, iomcu_uart))
    tick = app.get_config('STM32_ST_USE_TIMER', required=False, default=None)
    sd_buses = {name for name in app.bytype
                if name.startswith('SDMMC') or name == 'SDIO'}
    supported_sd_buses = set(family['sd_buses'])
    present_sd_buses = sorted(sd_buses & supported_sd_buses)
    has_sd = bool(present_sd_buses)
    sd_bus = present_sd_buses[0] if has_sd else None
    if len(present_sd_buses) > 1:
        warnings.append('multiple SDMMC buses are not supported concurrently')
    for bus in sorted(sd_buses - supported_sd_buses):
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
    for name, adc, channel, raw in battery_samples:
        lines += [
            '# simulated battery %s input' % name,
            'sysbus.%s FeedSample %u %u -1' % (adc, raw, channel),
            '',
        ]
    if serial is not None:
        serial_target = serial.lower()
        if family['name'] in ('h743', 'h757', 'f303', 'f767', 'g474', 'l4'):
            serial_target += 'Host'
        lines += [
            'emulation CreateServerSocketTerminal %u "serial" false' % uart_port,
            'connector Connect sysbus.%s serial' % serial_target,
            '',
        ]
    if iomcu_uart is not None:
        lines += [
            'emulation CreateUARTHub "iomcuHub"',
            'connector Connect sysbus.%s iomcuHub' % iomcu_uart.lower(),
            'connector Connect sysbus.iomcu iomcuHub',
            '',
        ]
    if gps_uart is not None:
        gps_target = gps_uart.lower()
        if family['name'] in ('h743', 'h757', 'f303', 'f767', 'g474', 'l4'):
            gps_target += 'Host'
        lines += [
            'emulation CreateUARTHub "gpsHub"',
            'connector Connect sysbus.%s gpsHub' % gps_target,
            'connector Connect sysbus.gps gpsHub',
            '',
        ]
    if rangefinder_uart is not None:
        rangefinder_target = rangefinder_uart.lower()
        if family['name'] in ('h743', 'h757', 'f303', 'f767', 'g474', 'l4'):
            rangefinder_target += 'Host'
        lines += [
            'emulation CreateUARTHub "rangefinderHub"',
            'connector Connect sysbus.%s rangefinderHub' % rangefinder_target,
            'connector Connect sysbus.rangefinder rangefinderHub',
            '',
        ]
    for bus, peripheral in can_buses:
        hub = '%sHub' % bus.lower()
        lines += [
            'emulation CreateCANHub "%s"' % hub,
            'connector Connect sysbus.%s %s' % (peripheral, hub),
            'connector Connect sysbus.%sMcast %s' % (bus.lower(), hub),
            '',
        ]
    if has_ethernet:
        lines += [
            'sysbus.ethernet ActivePhy RMII',
            '',
        ]
    for index in range(1, 5):
        label = 'GPIO_CAN_I2C%d_SEL' % index
        pin = app.bylabel.get(label)
        if pin is not None:
            lines += [
                '# select I2C rather than CAN on the shared connector',
                ('sysbus SetHookAfterPeripheralRead sysbus.gpioPort%s '
                 '"if offset == 0x10: value &= ~(1 << %u)"') %
                (pin.port, pin.pin),
                '',
            ]
    if has_sd:
        lines += [
            '$sdcard?=@none',
            'machine SdCardFromFile $sdcard sysbus.%s 0x10000000 True "sdcard"' %
            family['sd_buses'][sd_bus],
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

    if family['name'] in ('h743', 'h757'):
        for spi in app.spi_list:
            if spi in family['spis']:
                lines.append('sysbus SetHookBeforePeripheralWrite sysbus.%s '
                             '"if offset == 0x0C: value |= 0x400000"' % spi.lower())
        lines.append('')

    if quiet_peripherals:
        noisy = sorted({name.lower() for name in app.spi_list
                        if name in family['spis']} |
                       {name.lower() for name in app.get_config(
                           'I2C_ORDER', required=False, aslist=True) or []
                        if name in family['i2cs']} |
                       {name.lower() for name in app.get_config(
                           'SERIAL_ORDER', required=False, aslist=True) or []
                        if name in family['uarts']} |
                       (set(family.get('adcs', {}).values())
                        if battery_samples else set()) |
                       {'dma1', 'dma2', 'nvic'})
        lines += ['logLevel 3 sysbus.%s' % name for name in noisy]
        if has_sd:
            lines.append('logLevel 3 sysbus.%s' % family['sd_buses'][sd_bus])
        if has_ethernet:
            lines.append('logLevel 3 sysbus.ethernet')
    lines.append('')
    return '\n'.join(lines), {
        'app_base': app_base,
        'bootloader_load_base': 0x08000000 + boot_kb * 1024,
        'has_sdcard': has_sd,
        'serial_index': serial_index,
        'serial': serial,
        'iomcu_uart': iomcu_uart,
        'family': family['name'],
        'can_buses': [bus for bus, _ in can_buses],
        'has_ethernet': has_ethernet,
        'primary_ram_base': app.get_ram_map()[0][0],
        'airspeed_bus': airspeed_bus,
        'battery_channels': {
            name: channel for name, _, channel, _ in battery_samples
        },
        'rangefinder_uart': rangefinder_uart,
        'rangefinder_type': rangefinder_type,
    }


def generate(root, board, outdir, serial_index=None, uart_port=5762,
             state_dir=None, quiet_peripherals=True):
    '''Generate the board REPL/RESC and return paths plus run metadata.'''
    root = root.resolve()
    outdir = outdir.resolve()
    state_dir = (Path(state_dir) if state_dir is not None else
                 root / 'renode' / board).resolve()
    boards = supported_boards(root)
    if board not in boards:
        raise ValueError('%s is not a supported Renode target' % board)
    board_dir = _hwdef_root(root) / board
    is_periph = _resolved_env(board_dir / 'hwdef.dat', 'AP_PERIPH') == '1'
    outdir.mkdir(parents=True, exist_ok=True)
    app = _compile_hwdef(root, board_dir / 'hwdef.dat', outdir / 'hwdef')
    bootloader_hwdef = board_dir / 'hwdef-bl.dat'
    bootloader = (_compile_hwdef(
        root, bootloader_hwdef, outdir / 'hwdef-bl', bootloader=True)
        if bootloader_hwdef.is_file() else app)
    if app.mcu_type != boards[board] or bootloader.mcu_type != app.mcu_type:
        raise ValueError('application and bootloader MCU definitions disagree')

    warnings = []
    repl = outdir / ('%s.repl' % board)
    resc = outdir / ('%s.resc' % board)
    (platform, has_fram, iomcu_uart, can_buses, has_ethernet, gps_uart,
     airspeed_bus, battery_samples, rangefinder_uart,
     rangefinder_type) = _platform(
        root, board, app, outdir, state_dir / 'fram.img', is_periph, warnings)
    repl.write_text(platform)
    script, metadata = _script(
        root, board, app, bootloader, repl, serial_index, uart_port,
        iomcu_uart, can_buses, has_ethernet, gps_uart, airspeed_bus,
        battery_samples, rangefinder_uart, rangefinder_type, warnings,
        quiet_peripherals)
    resc.write_text(script)
    metadata.update({
        'repl': repl,
        'resc': resc,
        'warnings': warnings,
        'hwdef_h': outdir / 'hwdef' / 'hwdef.h',
        'has_fram': has_fram,
        'fram_size': 32 * 1024 if has_fram else 0,
        'gps_uart': gps_uart,
        'is_periph': is_periph,
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
