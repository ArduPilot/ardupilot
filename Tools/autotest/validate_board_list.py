#!/usr/bin/env python3

'''
Check Tools/bootloader/board_types.txt for problems

AP_FLAKE8_CLEAN

'''

import re


class ValidateBoardList(object):

    class BoardType(object):
        def __init__(self, name, board_id):
            self.name = name
            self.board_id = board_id

    def __init__(self):
        self.filepath = "Tools/AP_Bootloader/board_types.txt"

    def read_filepath(self, filepath):
        '''return contents of filepath'''
        content = ""
        with open(filepath) as fh:
            content += fh.read()
            fh.close()
        return content

    def parse_filepath_content(self, filepath):
        '''read contents of filepath, returns a list of (id, name) tuples'''
        content = self.read_filepath(filepath)
        ret = []
        for line in content.split("\n"):
            # strip comments:
            line = re.sub("#.*", "", line)
            # remove empty lines:
            if not len(line) or line.isspace():
                continue
            # remove trailing whitespace
            line = line.rstrip()
            m = re.match(r"^(.*?)\s+(\d+)$", line)
            if m is None:
                raise ValueError("Failed to match (%s)" % line)
            print("line: (%s)" % str(line))
            ret.append((int(m.group(2)), m.group(1)))
        return ret

    def validate_filepath_content(self, tuples):
        '''validate a list of (id, name) tuples'''

        # a list of board IDs which can map to multiple names for
        # historical reasons:
        board_id_whitelist = frozenset([
            9,  # fmuv2 and fmuv3
            10, # TARGET_HW_PX4_FMU_V4_PRO and TARGET_HW_PX4_PIO_V3
            13, # TARGET_HW_PX4_FMU_V4_PRO and TARGET_HW_PX4_PIO_V3
            29, # TARGET_HW_AV_V1 and TARGET_HW_AV_X_V1
            51, # TARGET_HW_PX4_FMU_V5X and Reserved  PX4 [BL] FMU v5X.x
            52, # TARGET_HW_PX4_FMU_V6 and Reserved "PX4 [BL] FMU v6.x"
            53, # TARGET_HW_PX4_FMU_V6X and Reserved "PX4 [BL] FMU v6X.x"
            57, # TARGET_HW_ARK_FMU_V6X and Reserved "ARK [BL] FMU v6X.x"
            59, # TARGET_HW_ARK_FPV and Reserved "ARK [BL] FPV"
            80, # TARGET_HW_ARK_CAN_FLOW and Reserved "ARK CAN FLOW"
            20, # TARGET_HW_UVIFY_CORE and AP_HW_F4BY
        ])

        dict_by_id = {}
        dict_by_name = {}
        for (board_id, name) in tuples:
            print("Checking (%u, %s)" % (board_id, name))
            if board_id in dict_by_id and board_id not in board_id_whitelist:
                raise ValueError("Duplicate ID %s in file for (%s) and (%s)" %
                                 (board_id, dict_by_id[board_id], name))
            if name in dict_by_name:
                raise ValueError("Duplicate name %s in file for (%s) and (%s)" %
                                 (name, dict_by_name[name], board_id))
            dict_by_name[name] = board_id
            dict_by_id[board_id] = name

    def run(self):
        parsed = self.parse_filepath_content(self.filepath)
        self.validate_filepath_content(parsed)
        return 0


if __name__ == '__main__':
    validator = ValidateBoardList()
    validator.run()
