# Third-party notices

## Renode and Emul8-derived files

The following files are adapted from Renode or its predecessor, Emul8:

- `peripherals/stm32/AP_STM32F4_I2C.cs`
- `platforms/stm32f405_base.repl`
- `platforms/stm32f746_ap_base.repl`
- `platforms/stm32h743_base.repl`
- `patches/usbip-device-state.patch`

Copyright (c) Antmicro <www.antmicro.com>. These files are distributed under
the [MIT licence](https://github.com/antmicro/renode/blob/v1.16.1/LICENSE).
The platform sources came from Renode 1.16.1 and
[ArduPilot's Renode fork](https://github.com/ArduPilot/renode/tree/2a060779f4e2b87d1ae7238a041d858369818805).

## pyfatfs

The Renode runtime uses
[pyfatfs 1.1.0](https://github.com/nathanhi/pyfatfs/tree/v1.1.0) to create and
read FAT images without platform-specific host utilities. pyfatfs is copyright
(c) 2018-2024 pyfatfs contributors and distributed under the
[MIT licence](https://github.com/nathanhi/pyfatfs/blob/v1.1.0/LICENSE).

## STMicroelectronics CMSIS-SVD files

The following STMicroelectronics CMSIS-SVD deliverables are hosted in the
[ArduPilot Renode SVD data directory][svd-data] outside the source tree:

- `STM32F103.svd.gz`, `STM32F105.svd.gz`, `STM32F303.svd.gz`
- `STM32F405.svd.gz`, `STM32F407.svd.gz`, `STM32F412.svd.gz`,
  `STM32F427.svd.gz`
- `STM32F732.svd.gz`, `STM32F767.svd.gz`
- `STM32G441.svd.gz`, `STM32G474.svd.gz`, `STM32G491.svd.gz`
- `STM32H723.svd.gz`, `STM32H743.svd.gz`, `STM32H757.svd.gz`
- `STM32L431.svd.gz`, `STM32L4R5.svd.gz`, `STM32L4x6.svd.gz`

All except the H723 and H757 files are redistributed under the
[STMicroelectronics End User License Agreement, version 1.0][st-eula]. The
F405 and G474 files preserve the previously vendored deliverables; the other
files in this group are sourced from the pinned
[cmsis-svd-data STMicro collection][cmsis-svd-source]. The licence permits
copying these deliverables for development tools and for representations used
to develop and debug software for the identified STMicroelectronics devices.

The H723 and H757 files are sourced from the pinned
[Open-CMSIS-Pack STM32H7 device family pack][h7-dfp-source] and distributed
under the [Apache License 2.0][h7-dfp-license]. For H757, the CM7 description
is used because ArduPilot executes on that core.

[svd-data]: https://firmware.ardupilot.org/Tools/Renode/data/SVD/
[cmsis-svd-source]: https://github.com/cmsis-svd/cmsis-svd-data/tree/40327a4d2dff0992682be2872aaa6e096f35d2f4/data/STMicro
[st-eula]: https://github.com/cmsis-svd/cmsis-svd-data/blob/40327a4d2dff0992682be2872aaa6e096f35d2f4/data/STMicro/License.html
[h7-dfp-source]: https://github.com/Open-CMSIS-Pack/STM32H7xx_DFP/tree/058af2e871a1ee2f203459f8780c02741079ba5a/CMSIS/SVD
[h7-dfp-license]: https://github.com/Open-CMSIS-Pack/STM32H7xx_DFP/blob/058af2e871a1ee2f203459f8780c02741079ba5a/LICENSE-Apache-2.0
