// top.v
// Top-level module for FPGA system
// Integrates PLL, MCU SPI interface, register module, and SCH16T sensor interface
module top (
    input inclk,              // Input clock

    // MCU SPI interface
    input  spics,             // SPI chip select
    input  spisck,            // SPI clock
    input  spisi,             // SPI serial input
    output spiso,             // SPI serial output

    // Sensor SPI interface
    output sensor_spics,      // Sensor SPI chip select
    output sensor_spisck,     // Sensor SPI clock
    output sensor_spisi,      // Sensor SPI serial input
    input  sensor_spiso,      // Sensor SPI serial output
    input  sensor_rdy,        // Sensor ready signal

    output [3:0] debug,       // Debug output

    output  led               // LED output
);

    wire        pll_lock;      // PLL lock signal
    wire        sysclk;        // System clock (synthesis keep)

    reg         rst;           // Reset signal
    reg  [ 7:0] rst_wait;      // Reset wait counter
    reg  [26:0] led_cnt;       // LED counter

    // SPI interface signals (synthesis keep)
    wire [15:0] spi_reg_addr;  // SPI register address
    wire [15:0] spi_ram_addr;  // SPI RAM address
    wire        spi_wren;      // SPI write enable
    wire [31:0] spi_wrdt;      // SPI write data
    wire        spi_rdreq;     // SPI read request
    reg         spi_rdv;       // SPI read data valid
    reg  [31:0] spi_rddt;      // SPI read data

    // SPI response signals (synthesis keep)
    wire        spi_reg_rdv;   // SPI register read data valid
    wire [31:0] spi_reg_rddt;  // SPI register read data
    wire        spi_sch16_rdv; // SPI SCH16T read data valid
    wire [31:0] spi_sch16_rddt;// SPI SCH16T read data


    // PLL instantiation for clock generation (IP Invoke)
    mpll upll (
        .refclk  (inclk),      // Reference clock input
        .reset   (1'b0),       // PLL reset (always off)
        .extlock (pll_lock),    // PLL lock output
        .clk0_out(sysclk)       // System clock output
    );

    // Reset generation logic
    always @(negedge pll_lock or posedge sysclk) begin
        if (!pll_lock) begin
            rst      <= 1'b1;     // Assert reset when PLL is not locked
            rst_wait <= 8'd0;     // Reset wait counter
        end else if (rst_wait != 8'hff) begin
            rst      <= 1'b1;     // Keep reset asserted during wait period
            rst_wait <= rst_wait + 1;  // Increment wait counter
        end else begin
            rst      <= 1'b0;     // Deassert reset after wait period
            rst_wait <= rst_wait; // Hold wait counter
        end
    end

    // LED blinking logic
    always @(posedge sysclk) begin
        led_cnt <= led_cnt + 1;  // Increment LED counter
    end

    assign led = led_cnt[26];    // LED blinks based on counter

    // SPI signal synchronization
    reg spics_p;                // SPI chip select pipeline
    reg spisck_p;               // SPI clock pipeline
    reg spisi_p;                // SPI serial input pipeline
    reg spics_l;                // Latched SPI chip select
    reg spisck_l;               // Latched SPI clock
    reg spisi_l;                // Latched SPI serial input

    always @(posedge sysclk) begin
        spics_p  <= spics;      // Sample SPI chip select
        spisck_p <= spisck;     // Sample SPI clock
        spisi_p  <= spisi;      // Sample SPI serial input

        // Latch SPI signals when stable
        spics_l  <= (spics_p && spics) ? 1'b1 : (((~spics_p) && (~spics)) ? 1'b0 : spics_l);
        spisck_l <= (spisck_p && spisck) ? 1'b1 : (((~spisck_p) && (~spisck)) ? 1'b0 : spics_l);
        spisi_l  <= (spisi_p && spisi) ? 1'b1 : (((~spisi_p) && (~spisi)) ? 1'b0 : spics_l);
    end

    // MCU SPI interface instantiation
    mcu_spi_interface uspi (
        .sysclk(sysclk),        // System clock
        .rst   (rst),           // Reset signal

        // spi interface
        .spics (spics_l),       // Latched SPI chip select
        .spisck(spisck_l),      // Latched SPI clock
        .spisi (spisi_l),       // Latched SPI serial input
        .spiso (spiso),         // SPI serial output

        // write/read reg & write/read fifo & write/read ram
        .spi_reg_addr(spi_reg_addr),  // SPI register address
        .spi_ram_addr(spi_ram_addr),  // SPI RAM address
        .spi_wren    (spi_wren),      // SPI write enable
        .spi_wrdt    (spi_wrdt),      // SPI write data
        .spi_rdreq   (spi_rdreq),     // SPI read request
        .spi_rdv     (spi_rdv),       // SPI read data valid
        .spi_rddt    (spi_rddt)       // SPI read data
    );


    // Top register module instantiation
    top_reg ureg (
        .sysclk(sysclk),        // System clock
        .sysrst(rst),           // Reset signal

        .spi_reg_addr(spi_reg_addr),  // SPI register address
        .spi_ram_addr(spi_ram_addr),  // SPI RAM address
        .spi_wren    (spi_wren),      // SPI write enable
        .spi_wrdt    (spi_wrdt),      // SPI write data
        .spi_rdreq   (spi_rdreq),     // SPI read request

        .spi_rdv (spi_reg_rdv),       // SPI register read data valid
        .spi_rddt(spi_reg_rddt)       // SPI register read data
    );

    // SCH16T sensor interface instantiation
    sch16t_top usch16 (
        .sysclk(sysclk),        // System clock
        .rst   (rst),           // Reset signal

        // spi interface
        .spi_reg_addr(spi_reg_addr),  // SPI register address
        .spi_ram_addr(spi_ram_addr),  // SPI RAM address
        .spi_wren    (spi_wren),      // SPI write enable
        .spi_wrdt    (spi_wrdt),      // SPI write data
        .spi_rdreq   (spi_rdreq),     // SPI read request
        .spi_rdv     (spi_sch16_rdv), // SPI SCH16T read data valid
        .spi_rddt    (spi_sch16_rddt),// SPI SCH16T read data

        // sensor interface
        .sensor_spics (sensor_spics), // Sensor SPI chip select
        .sensor_spisck(sensor_spisck),// Sensor SPI clock
        .sensor_spisi (sensor_spisi), // Sensor SPI serial input
        .sensor_spiso (sensor_spiso), // Sensor SPI serial output
        .sensor_rdy   (sensor_rdy),   // Sensor ready signal

        .debug()                // Debug output (unused)
    );

    reg [3 : 0] debug_p;        // Debug register

    // SPI read data multiplexing
    always @(posedge sysclk) begin
        if (spi_reg_rdv) spi_rddt <= spi_reg_rddt;  // Use register read data if valid
        else spi_rddt <= spi_sch16_rddt;            // Otherwise use SCH16T read data

        spi_rdv <= spi_reg_rdv | spi_sch16_rdv;     // Set read data valid if either source is valid

        debug_p <= {spics_l, spisck_l, spisi_l, spiso};  // Debug signals
    end

    assign debug = debug_p;      // Debug output assignment



endmodule