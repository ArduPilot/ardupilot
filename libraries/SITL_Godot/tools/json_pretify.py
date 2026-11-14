#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Lay JSON out with less whitespace: indented, but anything that fits in
WIDTH columns stays on one line. A filter, for writing models/*.gltf:

  ./json_pretify.py < exported.gltf > ../models/mine.gltf

AP_FLAKE8_CLEAN
'''
import json
import sys

WIDTH = 200


def pretty(v, indent=0, key=""):
    flat = json.dumps(v, separators=(", ", ": "))
    if indent + len(key) + len(flat) < WIDTH or not isinstance(v, (dict, list)) or not v:
        return flat
    pad, sep = " " * indent, ",\n" + " " * (indent + 1)
    if isinstance(v, list):
        return "[\n " + pad + sep.join(pretty(x, indent + 1) for x in v) + "\n" + pad + "]"
    items = (json.dumps(k) + ": " + pretty(x, indent + 1, json.dumps(k) + ": ") for k, x in v.items())
    return "{\n " + pad + sep.join(items) + "\n" + pad + "}"


if __name__ == "__main__":
    sys.stdout.write(pretty(json.load(sys.stdin)) + "\n")
