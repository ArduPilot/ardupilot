*****************************************************************************
*** ChibiOS products directory organization                               ***
*****************************************************************************

--{root}                - Distribution directory.
  +--os/                - ChibiOS products, this directory.
  |  +--rt/             - ChibiOS/RT product.
  |  |  +--include/     - RT kernel headers.
  |  |  +--src/         - RT kernel sources.
  |  |  +--templates/   - RT kernel port template files.
  |  |  +--ports/       - RT kernel port files.
  |  |  +--osal/        - RT kernel OSAL module for HAL interface.
  |  +--nil/            - ChibiOS/NIL product.
  |  |  +--include/     - Nil kernel headers.
  |  |  +--src/         - Nil kernel sources.
  |  |  +--templates/   - Nil kernel port template files.
  |  |  +--ports/       - Nil kernel port files.
  |  |  +--osal/        - Nil kernel OSAL module for HAL interface.
  |  +--oslib/          - ChibiOS/LIB product.
  |  |  +--include/     - OSLIB headers.
  |  |  +--src/         - OSLIB sources.
  |  +--sb/             - ChibiOS/SB product.
  |  +--ex/             - ChibiOS/EX product.
  |  +--hal/            - ChibiOS/HAL product.
  |  |  +--include/     - HAL high level headers.
  |  |  +--src/         - HAL high level sources.
  |  |  +--templates/   - HAL port template files.
  |  |  +--ports/       - HAL port files (low level drivers implementations).
  |  |  +--boards/      - HAL board files.
  |  +--common/         - Files used by multiple ChibiOS products.
  |  |  +--abstractions/- Emulation of other API.
  |  |  +--ext/         - Vendor files used by ChibiOS products.
  |  |  +--portability/ - Compilers abstraction support files.
  |  |  +--ports        - RT/NIL port files for various architectures and
  |  |                    compilers.
  |  |  +--startup/     - Startup, linker and make support for various
  |  |                    architectures and compilers.
  |  +--various/        - Various portable support files.
 