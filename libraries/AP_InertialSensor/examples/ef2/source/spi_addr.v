// spi_addr.v
// Address parameter definitions for SPI register map
// Defines various addresses used for register, RAM, and FIFO access

// FPGA version identifier
localparam FpgaVersion = 16'h1001;  // FPGA version 1.0.0.1

// Base configuration registers
localparam Addr_R32_Verison = 16'h0000;   // Read-only: FPGA version
localparam Addr_RW16_TestReg = 16'h0001;  // Read-write: Test register

// SCH16T sensor interface registers
localparam Addr_W32_Sch16tCtrl = 16'h0010;               // Write-only: SCH16T control register
localparam Addr_RW8_DirectRam8 = 16'h0011;               // Read-write: Direct RAM access (8-bit)
localparam Addr_RW16_DirectSpiCtrl = 16'h0012;           // Read-write: Direct SPI control register (16-bit)
localparam Addr_W8_SensorReadCmdRam8 = 16'h0013;         // Write-only: Sensor read command RAM (8-bit)
localparam Addr_W8_SensorValueFifo8 = 16'h0014;          // Read-only: Sensor value FIFO (8-bit)
localparam Addr_RW8_SensorFifoCtrl = 16'h0015;           // Read-write: Sensor FIFO control register (8-bit)