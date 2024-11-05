#!/usr/bin/env python3
from utils import get_firmware_revision
import sys, os


if __name__ == '__main__':
    if len(sys.argv) > 1:
        print(get_firmware_revision(apj_path=sys.argv[1]))
    else:
        print(get_firmware_revision())
