# FPGA FIFO Sensor Data Storage Project

## Project Overview
This project implements a FIFO (First-In-First-Out) buffer system for storing sensor data using the EF2M45VG81C FPGA from Anlogic. The system utilizes SPI (Serial Peripheral Interface) communication to interface with sensors, CRC-8 validation for data integrity, and dual-port RAM for efficient data buffering.

## Development Environment
- **FPGA Device**: EF2M45VG81C
- **Development Tool**: Anlogic TD
  - Download Link: [https://www.anlogic.com/support/tools-downloads](https://www.anlogic.com/support/tools-downloads)
  - **License Requirement**: TD License (registration required)

## Hardware Requirements
- EF2M45VG81C FPGA development board
- Sensor module with SPI interface
- Downloader: AL-LINK or AL-LINK-PRO
  - Usage Instructions: [https://www.anlogic.com/support/tools-downloads](https://www.anlogic.com/support/tools-downloads)

## Usage Instructions
1. **Compile the Project**:
   - Open the project in Anlogic software
   - Ensure all IP cores are properly configured
   - Compile the project to generate the bitstream

2. **Download to FPGA**:
   - Connect either AL-LINK or AL-LINK-PRO downloader to your computer
   - Connect the downloader to the FPGA development board
   - Use the Anlogic software to download the compiled bitstream to the FPGA
   - Follow the downloader usage instructions for detailed steps

## System Architecture
The project consists of the following main components:

- **SPI Interface Modules**:
  - `sch16t_direct.v`: Direct SPI communication with configurable parameters
  - `sch16t_fifo.vhd`: FPGA mode SPI communication with automatic command handling
  - `sch16t_top.v`: Top-level module managing both communication modes

- **Data Processing**:
  - `crc8.vhd`: 8-bit CRC calculation for data validation
  - `mcu_spi_interface.v`: SPI interface between MCU and FPGA

- **Memory Components**:
  - Dual-port RAM for data buffering
  - FIFO for data storage and retrieval

- **Control Modules**:
  - `top.v`: System top-level module integrating all components
  - `top_reg.v`: Register module for configuration and status
  - `spi_addr.v`: Address parameter definitions

## Porting to Other FPGA Platforms

### For Xilinx FPGAs
1. **IP Core Replacement**:
   - Identify all IP cores used in the project (marked with "IP Invoke" comments)
   - Replace Anlogic-specific IP cores with equivalent Xilinx IP cores
   - Common IP cores to replace:
     - PLL (Phase-Locked Loop) for clock generation
     - Dual-port RAM for data buffering
     - Any other platform-specific IPs

2. **IP Core Instantiation**:
   - Follow Xilinx Vivado guidelines for IP core instantiation
   - Update the instantiation templates in the code
   - Ensure proper connection of all ports and parameters

3. **Clock and Reset Logic**:
   - Adjust clock generation and reset logic to match Xilinx architecture
   - Update any platform-specific timing constraints

4. **Synthesis and Implementation**:
   - Use Xilinx Vivado for synthesis and implementation
   - Generate the bitstream for the target Xilinx device

### General Porting Guidelines
- **Identify Platform-Specific Code**:
  - Look for comments marked "IP Invoke" indicating where IP cores are used
  - Check for platform-specific pragmas or attributes (e.g., `// synthesis keep`)

- **Adapt to Target Architecture**:
  - Adjust data widths and bus interfaces to match target FPGA capabilities
  - Update timing constraints for the new platform
  - Verify clock domain crossings and synchronization

- **Testing**:
  - Test the ported design thoroughly
  - Verify SPI communication functionality
  - Validate data integrity with CRC checks
  - Confirm FIFO operation and data buffering

## Directory Structure
```
FPGA_FIFO/
└── fpga_code/
    └── ef2/
        ├── README.md           # This file
        ├── source/
        │   ├── crc8.vhd        # CRC-8 calculation module
        │   ├── mcu_spi_interface.v  # MCU SPI interface
        │   ├── spi_addr.v      # Address parameter definitions
        │   ├── top.v           # System top-level module
        │   ├── top_reg.v       # Register module
        │   └── sch16t/         # SCH16T sensor interface
        │       ├── sch16t_direct.v  # Direct SPI communication
        │       ├── sch16t_fifo.vhd  # FPGA mode SPI communication
        │       └── sch16t_top.v     # SCH16T top-level module
        └── [other project files]
```

## Key Features
- Configurable SPI communication parameters
- CRC-8 data validation for integrity
- Dual-port RAM for efficient data buffering
- FIFO for reliable data storage and retrieval
- Support for both direct and FPGA-controlled communication modes

## License
This project requires a TD License from Anlogic for development and usage.

## Support
For further assistance, refer to the Anlogic support resources at [https://www.anlogic.com/support](https://www.anlogic.com/support).