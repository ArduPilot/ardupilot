#!/usr/bin/env python3

"""
Parses a dataflash (.bin) or MAVLink telemetry (.tlog) log and shows how
bitmask fields changed over time.  For dataflash logs, bit names come from
ArduPilot XML log-message metadata.  For telemetry logs, bit names come from
MAVLink enum definitions embedded in the dialect.

Usage: bitmask_change.py <logfile> MSG.Field [MSG.Field ...]

AP_FLAKE8_CLEAN

"""

import argparse
import os
import sys
import time

from pymavlink import mavutil


class BitmaskChange(object):
    def __init__(self, logfile, fields):
        self.logfile = logfile
        self.fields = []
        self.optional_fields = set()
        for field in fields:
            if len(field) == 3:
                msg_type, field_name, optional = field
            else:
                msg_type, field_name = field
                optional = False
            self.fields.append((msg_type, field_name, optional))
            if optional:
                self.optional_fields.add((msg_type, field_name))

    def error(self, msg):
        print("ERROR: %s" % msg, file=sys.stderr)
        sys.exit(1)

    def _build_meta_dataflash(self, mlog):
        bitmask_meta = {}
        tree = mlog.metadata.metadata_tree()
        if tree is None:
            self.error("metadata_tree() returned None - XML log-message metadata not found")
        for msg_type, field_name, optional in self.fields:
            if msg_type not in tree:
                self.error("message type '%s' not found in metadata" % msg_type)
            msg_node = tree[msg_type]
            field_node = None
            for f in msg_node.fields.field:
                if f.get('name') == field_name:
                    field_node = f
                    break
            if field_node is None:
                if optional:
                    continue
                self.error("field '%s' not found in message '%s'" % (field_name, msg_type))
            bitmask_node = field_node.find('bitmask')
            if bitmask_node is None:
                if optional:
                    continue
                self.error("field '%s.%s' has no bitmask metadata" % (msg_type, field_name))
            bits = []
            for bit in bitmask_node.iterchildren('bit'):
                bits.append((int(bit.value), bit.get('name')))
            bits.sort(key=lambda x: x[0])
            if msg_type not in bitmask_meta:
                bitmask_meta[msg_type] = {}
            bitmask_meta[msg_type][field_name] = bits
        return bitmask_meta

    def _build_meta_tlog(self):
        bitmask_meta = {}
        for msg_type, field_name, optional in self.fields:
            cls_name = "MAVLink_%s_message" % msg_type.lower()
            msg_class = getattr(mavutil.mavlink, cls_name, None)
            if msg_class is None:
                if optional:
                    continue
                self.error("unknown MAVLink message type '%s'" % msg_type)
            enum_name = msg_class.fieldenums_by_name.get(field_name)
            if enum_name is None or enum_name not in mavutil.mavlink.enums:
                if optional:
                    continue
                self.error("field '%s.%s' has no bitmask enum in MAVLink dialect" % (msg_type, field_name))
            raw_entries = []
            for v, e in mavutil.mavlink.enums[enum_name].items():
                if v > 0 and (v & (v - 1)) == 0:
                    raw_entries.append((v, e.name))
            if not raw_entries:
                if optional:
                    continue
                self.error("enum '%s' has no power-of-two entries" % enum_name)
            prefix = os.path.commonprefix([name for _, name in raw_entries])
            bits = [(v, name[len(prefix):] or name) for v, name in raw_entries]
            bits.sort()
            bitmask_meta.setdefault(msg_type, {})[field_name] = bits
        return bitmask_meta

    @staticmethod
    def parse_msg_fields(msg_fields):
        fields = []
        for spec in msg_fields:
            parts = spec.split(".", 1)
            if len(parts) != 2:
                print("ERROR: invalid MSG.Field specifier '%s' - expected format: MSG.Field" % spec,
                      file=sys.stderr)
                sys.exit(1)
            fields.append((parts[0], parts[1]))
        return fields

    def run(self):
        mlog = mavutil.mavlink_connection(self.logfile)

        is_dataflash = hasattr(mlog, 'metadata')

        if is_dataflash:
            bitmask_meta = self._build_meta_dataflash(mlog)
        else:
            bitmask_meta = self._build_meta_tlog()

        prev = {}
        for msg_type, fields_dict in bitmask_meta.items():
            prev[msg_type] = {}
            for fn in fields_dict:
                prev[msg_type][fn] = 0

        msg_types = list(bitmask_meta.keys())
        while True:
            m = mlog.recv_match(type=msg_types)
            if m is None:
                break
            msg_type = m.get_type()
            for field_name, bits in list(bitmask_meta[msg_type].items()):
                try:
                    new_val = getattr(m, field_name)
                except AttributeError:
                    if (msg_type, field_name) in self.optional_fields:
                        del bitmask_meta[msg_type][field_name]
                        continue
                    raise
                old_val = prev[msg_type][field_name]
                if new_val == old_val:
                    continue

                # Build set of known bit values
                known_bit_values = set(bv for bv, _ in bits)

                changes = []
                for bit_val, bit_name in bits:
                    old_set = old_val & bit_val
                    new_set = new_val & bit_val
                    if new_set and not old_set:
                        changes.append("+%s" % bit_name)
                    elif not new_set and old_set:
                        changes.append("-%s" % bit_name)

                # Handle unknown bits (iterate individual bit positions)
                for i in range(64):
                    mask = 1 << i
                    if mask > (old_val | new_val):
                        break
                    if mask in known_bit_values:
                        continue
                    old_set = old_val & mask
                    new_set = new_val & mask
                    if new_set and not old_set:
                        changes.append("+UNKNOWN_BIT%u" % i)
                    elif not new_set and old_set:
                        changes.append("-UNKNOWN_BIT%u" % i)

                if not changes:
                    prev[msg_type][field_name] = new_val
                    continue

                timestamp = getattr(m, '_timestamp', 0.0)
                formatted_timestamp = "%s.%02u" % (
                    time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp)),
                    int(timestamp * 100.0) % 100)

                print("%s: %s.%s %s" % (
                    formatted_timestamp,
                    msg_type,
                    field_name,
                    " ".join(changes)))

                prev[msg_type][field_name] = new_val


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Show bitmask field changes in a dataflash (.bin) or MAVLink telemetry (.tlog) log",
        usage="%(prog)s logfile MSG.Field [MSG.Field ...]")
    parser.add_argument("logfile", help="path to .bin dataflash or .tlog MAVLink telemetry log")
    parser.add_argument("msg_fields", metavar="MSG.Field", nargs="+",
                        help="message.field specifier, e.g. LDET.Flags")

    args = parser.parse_args()

    fields = BitmaskChange.parse_msg_fields(args.msg_fields)
    tool = BitmaskChange(args.logfile, fields)
    tool.run()
