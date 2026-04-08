// top_reg.v
// Register module for FPGA system
// Implements register read/write operations for configuration and status
module top_reg (
    input           sysclk,         // System clock
    input           sysrst,         // System reset

    input [15:0]    spi_reg_addr,   // SPI register address
    input [15:0]    spi_ram_addr,   // SPI RAM address
    input           spi_wren,       // SPI write enable
    input [31:0]    spi_wrdt,       // SPI write data
    input           spi_rdreq,      // SPI read request

    output          spi_rdv,        // SPI read data valid
    output [31:0]   spi_rddt        // SPI read data
);

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
    localparam Addr_W8_SensorValueFifo8 = 16'h0014;          // Write-only: Sensor value FIFO (8-bit)
    localparam Addr_RW8_SensorFifoCtrl = 16'h0015;           // Read-write: Sensor FIFO control register (8-bit)


    // Internal registers
    reg [15:0] test_reg;  // Test register
    reg rdv;              // Read data valid flag
    reg [31:0] rddt;      // Read data register

    // Register write and read logic
    always @(posedge sysclk) begin
        // Test register write
        if (sysrst) begin
            test_reg <= 16'd0;  // Reset test register
        end else if (spi_wren && spi_reg_addr == Addr_RW16_TestReg) begin
            test_reg <= spi_wrdt[15:0];  // Write to test register
        end else begin
            test_reg <= test_reg;  // Hold test register value
        end

        // Register read logic
        if (spi_rdreq) begin
            case (spi_reg_addr) // Check register address
                Addr_RW16_TestReg: begin  // Test register read
                    rdv <= 1'b1;           // Set read data valid
                    rddt <= {16'd0, test_reg};  // Return test register value
                end
                Addr_R32_Verison: begin   // Version register read
                    rdv <= 1'b1;           // Set read data valid
                    rddt <= {16'd0, FpgaVersion};  // Return FPGA version
                end
                default: begin            // Invalid address
                    rdv <= 1'b0;           // Clear read data valid
                    rddt <= 32'h00000000;  // Return zero
                end
            endcase
        end else begin
            rdv <= 1'b0;           // Clear read data valid
            rddt <= 32'h00000000;  // Return zero
        end

    end

    // Output assignments
    assign spi_rdv = rdv;       // Read data valid output
    assign spi_rddt = rddt;     // Read data output



endmodule