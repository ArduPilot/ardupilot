#!/bin/sh
# Upload a firmware image to a flight controller using the second-stage bootloader

if [ $# -lt 2 ]; then
    echo "Usage: ssbl_uploader.sh BOARD TARGET"
    exit 1
fi

BOARD="$1"
TARGET="$2"
TARGET2MB=build/${BOARD}/bin/${TARGET}_2MB.bin

rm -f "${TARGET2MB}-VERIFY.bin"
dd if=/dev/zero ibs=1k count=2048 of=${TARGET2MB}
dd conv=notrunc if=build/${BOARD}/bin/${TARGET}_extf.bin of=${TARGET2MB}
dfu-util -D "${TARGET2MB}" -s 0x90100000:0x200000
dfu-util -U "${TARGET2MB}-VERIFY.bin" -s 0x90100000:0x200000
diff -sb ${TARGET2MB} "${TARGET2MB}-VERIFY.bin"
