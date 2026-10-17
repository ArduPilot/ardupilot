# DCD FC IO firmware

This target builds the standard STM32F103 IO application for [DCD FC](../DCD-FC/README.md).

![DCD FC](../DCD-FC/images/product_photo.webp)

It uses the upstream `iomcu-f103` configuration and disables PPM input on IO PA8, which is not connected to an external receiver port. SBUS input and all eight IO outputs are retained. The IO application starts at `0x08001000` and retains the upstream IO board ID and existing compatible IO bootloader.

Build with `./waf configure --board iomcu-DCD-FC` followed by `./waf iofirmware`. The FMU embeds `iofirmware_lowpolh.bin` as `Tools/IO_Firmware/iofirmware_DCD_FC_lowpolh.bin`. This file is an IO application, not an IO bootloader.
