// sch16t_top.v
// Top-level module for sensor SPI communication
// Manages two communication modes: direct mode and FPGA read/write mode
module sch16t_top (
    input           sysclk,         // System clock
    input           rst,            // Reset signal

    // SPI interface to MCU
    input [15:0]    spi_reg_addr,   // SPI register address
    input [15:0]    spi_ram_addr,   // SPI RAM address
    input           spi_wren,       // SPI write enable
    input [31:0]    spi_wrdt,       // SPI write data
    input           spi_rdreq,      // SPI read request
    output          spi_rdv,        // SPI read valid
    output [31:0]   spi_rddt,       // SPI read data

    // Sensor interface
    output          sensor_spics,   // Sensor SPI chip select
    output          sensor_spisck,  // Sensor SPI clock
    output          sensor_spisi,   // Sensor SPI serial input
    input           sensor_spiso,   // Sensor SPI serial output
    input           sensor_rdy,     // Sensor ready signal

    output reg [15 : 0] debug       // Debug output
);

    // FPGA version constant
    localparam FpgaVersion = 16'h1001;

    // Register addresses
    localparam Addr_R32_Verison = 16'h0000;                  // Version register
    localparam Addr_RW16_TestReg = 16'h0001;                 // Test register
    localparam Addr_W32_Sch16tCtrl = 16'h0010;               // Sensor control register
    localparam Addr_RW8_DirectRam8 = 16'h0011;               // Direct mode RAM access
    localparam Addr_RW16_DirectSpiCtrl = 16'h0012;           // Direct mode SPI control
    localparam Addr_W8_SensorReadCmdRam8 = 16'h0013;         // Sensor command RAM
    localparam Addr_W8_SensorValueFifo8 = 16'h0014;          // Sensor value FIFO
    localparam Addr_RW8_SensorFifoCtrl = 16'h0015;           // Sensor FIFO control

    // Sensor control registers
    reg         sensor_mode;         // Sensor mode: 0 - direct mode, 1 - FPGA read/write mode
    reg         sensor_fifo_rst;     // Sensor FIFO reset
    reg         sensor_fifo_enable;  // Sensor FIFO enable
    reg [5:0]   sensor_cmd_num_sub1; // Number of commands - 1
    reg [7:0]   sensor_baud_sub1;    // Baud rate divisor - 1

    // Direct mode RAM signals
    reg         direct_ram_wren;     // Direct RAM write enable (synthesis keep)
    reg [5:0]   direct_ram_wraddr;   // Direct RAM write address (synthesis keep)
    reg [7:0]   direct_ram_wrdt;     // Direct RAM write data (synthesis keep)
    reg         direct_ram_rdreq;    // Direct RAM read request (synthesis keep)
    reg         direct_ram_rdvalid;  // Direct RAM read valid (synthesis keep)
    reg [5:0]   direct_ram_rdaddr;   // Direct RAM read address (synthesis keep)

    // Direct mode SPI control signals
    reg         direct_func_clr;     // Direct mode clear
    reg         direct_func_start;   // Direct mode start
    reg [4:0]   direct_bytes_sub1;   // Direct mode bytes to transfer - 1

    // FIFO mode command RAM signals
    reg         cmd_ram_wren;        // Command RAM write enable (synthesis keep)
    reg [8:0]   cmd_ram_wraddr;      // Command RAM write address (synthesis keep)
    reg [7:0]   cmd_ram_wrdt;        // Command RAM write data (synthesis keep)
    reg         fifo_rdreq;          // FIFO read request
    wire [7:0]  fifo_rdvalue;        // FIFO read value
    reg         fifo_rdvalid;        // FIFO read valid
    reg         fifo_rdpkt_finsh;    // FIFO packet finish
    wire [7:0]  fifo_pkt_num;        // FIFO packet number

    // MCU read data signals
    reg         mcu_rdv;             // MCU read valid
    reg [31:0]  mcu_rddt;            // MCU read data

    // Direct mode interface signals
    wire        direct_enable;       // Direct mode enable
    wire        direct_ram_valid;    // Direct RAM valid
    wire [7:0]  direct_ram_rddt;     // Direct RAM read data
    wire        direct_busy;         // Direct mode busy
    wire        direct_spics;        // Direct mode SPI chip select
    wire        direct_spisck;       // Direct mode SPI clock
    wire        direct_spisi;        // Direct mode SPI serial input

    // FIFO mode error signal
    wire        rdy_error;           // Sensor ready error

    // Main control logic
    always @(posedge sysclk) begin

        // Sensor control register update
        if (rst) begin
            sensor_mode <= 1'b0;
            sensor_fifo_rst <= 1'b0;
            sensor_fifo_enable <= 1'b0;
            sensor_cmd_num_sub1 <= 6'd0;
            sensor_baud_sub1 <= 8'd0;
        end else if (spi_wren && spi_reg_addr == Addr_W32_Sch16tCtrl) begin
            sensor_mode <= spi_wrdt[0];          // Set sensor mode
            sensor_fifo_rst <= spi_wrdt[1];       // Set FIFO reset
            sensor_fifo_enable <= spi_wrdt[2];    // Set FIFO enable
            sensor_cmd_num_sub1 <= spi_wrdt[13:8]; // Set command count
            sensor_baud_sub1 <= spi_wrdt[23:16];  // Set baud rate
        end else begin
            sensor_mode <= sensor_mode;
            sensor_fifo_rst <= 1'b0;              // Clear reset after one cycle
            sensor_fifo_enable <= sensor_fifo_enable;
            sensor_cmd_num_sub1 <= sensor_cmd_num_sub1;
            sensor_baud_sub1 <= sensor_baud_sub1;
        end

        // Direct mode RAM write control
        direct_ram_wren <= (spi_wren && spi_reg_addr == Addr_RW8_DirectRam8) ? 1'b1 : 1'b0;
        direct_ram_wraddr <= spi_ram_addr[5:0];
        direct_ram_wrdt <= spi_wrdt[7:0];

        // Direct mode RAM read control
        direct_ram_rdreq <= (spi_rdreq && spi_reg_addr == Addr_RW8_DirectRam8) ? 1'b1 : 1'b0;
        direct_ram_rdaddr <= spi_ram_addr[5:0];
        direct_ram_rdvalid <= direct_ram_rdreq;

        // Direct mode SPI control
        if (spi_wren && spi_reg_addr == Addr_RW16_DirectSpiCtrl) begin
            direct_func_clr <= spi_wrdt[3:0] == 1 ? 1'b1 : 1'b0;
            direct_func_start <= spi_wrdt[3:0] == 2 ? 1'b1 : 1'b0;
            direct_bytes_sub1 <= spi_wrdt[12:8];
        end else begin
            direct_func_clr <= 1'b0;
            direct_func_start <= 1'b0;
            direct_bytes_sub1 <= direct_bytes_sub1;
        end

        // FPGA read command RAM write control
        cmd_ram_wren <= (spi_wren && spi_reg_addr == Addr_W8_SensorReadCmdRam8) ? 1'b1 : 1'b0;
        cmd_ram_wraddr <= spi_ram_addr[8:0];
        cmd_ram_wrdt <= spi_wrdt[7:0];

        // FIFO read control
        fifo_rdreq <= (spi_rdreq && spi_reg_addr == Addr_W8_SensorValueFifo8) ? 1'b1 : 1'b0;
        fifo_rdvalid <= fifo_rdreq;

        // FIFO packet finish control
        fifo_rdpkt_finsh <= (spi_wren && spi_reg_addr == Addr_RW8_SensorFifoCtrl) ? spi_wrdt[0] : 1'b0;

        // MCU read data multiplexing
        if (direct_ram_rdvalid) begin
            mcu_rdv <= 1'b1;
            mcu_rddt <= {24'd0, direct_ram_rddt};
        end else if (fifo_rdvalid) begin
            mcu_rdv <= 1'b1;
            mcu_rddt <= {24'd0, fifo_rdvalue};
        end else if (spi_rdreq) begin
            case (spi_reg_addr)
                Addr_RW16_DirectSpiCtrl : begin
                    mcu_rdv <= 1'b1;
                    mcu_rddt <= {31'd0, direct_busy};
                end
                Addr_RW8_SensorFifoCtrl : begin
                    mcu_rdv <= 1'b1;
                    mcu_rddt <= {23'd0, rdy_error, fifo_pkt_num};
                end
                default : begin
                    mcu_rdv <= 1'b0;
                    mcu_rddt <= mcu_rddt;
                end
            endcase
        end else begin
            mcu_rdv <= 1'b0;
            mcu_rddt <= mcu_rddt;
        end

    end

    // SPI output assignments
    assign spi_rdv = mcu_rdv;
    assign spi_rddt = mcu_rddt;

    // Direct mode enable (active when sensor_mode is 0)
    assign direct_enable = ~sensor_mode;

    // Direct mode SPI module instantiation
    sch16t_direct udirect (
        .sysclk         (sysclk),
        .rst            (rst),
        .enable         (direct_enable),

        .baud_sub1      (sensor_baud_sub1),
        .clr            (direct_func_clr),
        .start          (direct_func_start),
        .bytes_sub1     (direct_bytes_sub1),
        .busy           (direct_busy),
        .ram_wren       (direct_ram_wren),
        .ram_wraddr     (direct_ram_wraddr),
        .ram_wrdt       (direct_ram_wrdt),
        .ram_rdaddr     (direct_ram_rdaddr),
        .ram_rddt       (direct_ram_rddt),
        .spics          (direct_spics),
        .spisck         (direct_spisck),
        .spisi          (direct_spisi),
        .spiso          (sensor_spiso)
    );

    // FIFO mode signals
    wire fifo_rst;           // FIFO reset
    wire fpga_spics;         // FPGA mode SPI chip select
    wire fpga_spisck;        // FPGA mode SPI clock
    wire fpga_spisi;         // FPGA mode SPI serial input

    // FIFO reset generation
    assign fifo_rst = rst | (~sensor_mode) | sensor_fifo_rst;

    // FIFO mode SPI module instantiation
    sch16t_fifo ufifo (
        .sysclk             (sysclk),             // System clock
        .rst                (fifo_rst),           // Reset signal
        .enable             (sensor_fifo_enable),  // Enable signal

        .baud_sub1          (sensor_baud_sub1),    // Baud rate divisor - 1
        .cmd_num_sub1       (sensor_cmd_num_sub1), // Number of commands - 1

        .sensor_rdy         (sensor_rdy),          // Sensor ready signal
        .spics              (fpga_spics),          // SPI chip select
        .spisck             (fpga_spisck),         // SPI clock
        .spisi              (fpga_spisi),          // SPI serial input
        .spiso              (sensor_spiso),        // SPI serial output

        .rdy_error          (rdy_error),           // Sensor ready error

        .cmd_ram_wren       (cmd_ram_wren),        // Command RAM write enable
        .cmd_ram_wrdt       (cmd_ram_wrdt),        // Command RAM write data
        .cmd_ram_wraddr     (cmd_ram_wraddr),      // Command RAM write address

        .fifo_rdreq         (fifo_rdreq),          // FIFO read request
        .fifo_rdvalue       (fifo_rdvalue),        // FIFO read value
        .fifo_pkt_clr       (fifo_rdpkt_finsh),    // FIFO packet clear
        .fifo_usedw         (fifo_pkt_num)         // FIFO used words
    );

    // Sensor interface multiplexer
    // Select between direct mode and FPGA mode based on sensor_mode
    assign sensor_spics = sensor_mode ? fpga_spics : direct_spics;
    assign sensor_spisck = sensor_mode ? fpga_spisck : direct_spisck;
    assign sensor_spisi = sensor_mode ? fpga_spisi : direct_spisi;


endmodule
