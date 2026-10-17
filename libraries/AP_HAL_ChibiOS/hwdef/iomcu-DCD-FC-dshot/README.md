# DCD FC DShot IO firmware

This target builds the STM32F103 DShot IO application for [DCD FC](../DCD-FC/README.md).

![DCD FC](../DCD-FC/images/product_photo.webp)

It uses the upstream `iomcu-f103-dshot` output and DMA configuration with the [DCD FC no-PPM override](../iomcu-DCD-FC/no-ppm.inc). SBUS input and all eight IO outputs are retained. The existing compatible IO bootloader is unchanged.

Build with `./waf configure --board iomcu-DCD-FC-dshot` followed by `./waf iofirmware`. The FMU embeds `iofirmware_lowpolh.bin` as `Tools/IO_Firmware/iofirmware_DCD_FC_dshot_lowpolh.bin`. This file is an IO application, not an IO bootloader.
