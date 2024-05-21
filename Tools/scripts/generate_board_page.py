#!/usr/bin/env python3

'''
AP_FLAKE8_CLEAN
'''

import os
import pathlib
import shutil

# modify our search path:
import board_list


class GenerateBoardPage():
    '''an object that will create many html files into a directory, one
    for each hwdef board'''

    def __init__(self):
        self.outdir = "/tmp/some_html"

    def generate_UART_mapping(self, hwdef):
        uart_order = hwdef.get_config('SERIAL_ORDER', required=False, aslist=True)
        if uart_order is None:
            return None
        serial_purpose = hwdef.get_serial_purpose()
        count = 0
        ret = ""
        for uart in uart_order:
            purpose = serial_purpose.get(count, "")
            if purpose != "":
                purpose = f" ({purpose})"
            rtsctsnote = ""
            if hwdef.serial_has_cts_rts(count):
                rtsctsnote = " (RTS/CTS pins)"
            ret += f" - SERIAL{count} -> {uart}{purpose}{rtsctsnote}\n"
            count += 1

        return ret

    def generate_SensorFromList(self, hwdef, sensor_list):
        ret = ""

        dev_counts = {}
        for baro in sensor_list:
            dev = baro[0]
            if dev not in dev_counts.keys():
                dev_counts[dev] = 0
            dev_counts[dev] += 1
        for dev in dev_counts.keys():
            ret += f" - {dev}"
            if dev_counts[dev] > 1:
                ret += f" * {dev_counts[dev]}"
            ret += "\n"

        return ret

    def generate_Baro(self, hwdef):
        return self.generate_SensorFromList(hwdef, hwdef.baro_list)

    def generate_Compass(self, hwdef):
        return self.generate_SensorFromList(hwdef, hwdef.compass_list)

    def generate_IMU(self, hwdef):
        return self.generate_SensorFromList(hwdef, hwdef.imu_list)

    def generate_SDCard(self, hwdef):
        if hwdef.have_type_prefix('SDMMC2'):
            t = 'SDMMC2'
        elif hwdef.have_type_prefix('SDMMC'):
            t = 'SDMMC'
        elif hwdef.has_sdcard_spi():
            t = 'SPI'
        else:
            return None
        return f' - microSD card slot ({t})'

    def content_section(self, section_name, content):
        if content is None:
            return ""
        return f"""
{"=" * len(section_name)}
{section_name}
{"=" * len(section_name)}

{content}
"""

    def generate_content_for_board(self, board):
        hwdef = board.HWDef()

        content = ""

        content += self.content_section("UART Mapping", self.generate_UART_mapping(hwdef))
        content += self.content_section("Baro", self.generate_Baro(hwdef))
        content += self.content_section("Compass", self.generate_Compass(hwdef))
        content += self.content_section("IMU", self.generate_IMU(hwdef))
        content += self.content_section("SDCard", self.generate_SDCard(hwdef))

        return content

    def run(self):
        try:
            shutil.rmtree(self.outdir)
        except FileNotFoundError:
            pass
        pathlib.Path(self.outdir).mkdir(parents=True, exist_ok=True)

        for board in board_list.BoardList().boards:
            if board.name not in ["Pixhawk6X", "bbbmini"]:
                continue

            output = self.generate_content_for_board(board)

            outfile = os.path.join(self.outdir, f"{board.name}.rst")
            pathlib.Path(outfile).write_text(output)


gbp = GenerateBoardPage()
gbp.run()
