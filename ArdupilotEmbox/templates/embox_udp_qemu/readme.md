## Run embox
AUTOQEMU_DISABLE_SUDO="y" ./scripts/qemu/auto_qemu -serial mon:stdio -serial pty

## Run mavproxy
mavproxy.py --master /dev/pts/2 --console
