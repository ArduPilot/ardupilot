// mcu_spi_interface.v
// SPI interface module for MCU communication
// Implements SPI communication protocol for register, RAM, and FIFO access
module mcu_spi_interface (
    input sysclk,         // System clock
    input rst,            // Reset signal

    // spi interface
    input  spics,         // SPI chip select
    input  spisck,        // SPI clock
    input  spisi,         // SPI serial input
    output spiso,         // SPI serial output

    // write/read reg & write/read fifo & write/read ram
    output [15:0] spi_reg_addr,  // Register address for SPI operations
    output [15:0] spi_ram_addr,  // RAM address for SPI operations
    output        spi_wren,      // Write enable signal
    output [31:0] spi_wrdt,      // Write data
    output        spi_rdreq,     // Read request signal
    input         spi_rdv,       // Read data valid signal
    input  [31:0] spi_rddt       // Read data
);

    // Internal registers
    reg  [ 7:0] spi_din;     // SPI input data shift register
    reg  [ 2:0] bit_cnt;     // Bit counter for SPI transactions
    reg         bit_ed;      // Bit end flag
    reg  [ 2:0] head_cnt;    // Header byte counter

    reg  [ 7:0] func;        // Function code register
    wire        is_read;     // Read operation flag
    wire        is_reg;      // Register access flag
    wire        is_ram;      // RAM access flag
    wire        is_fifo;     // FIFO access flag
    wire        is_8;        // 8-bit data flag
    wire        is_16;       // 16-bit data flag
    wire        is_32;       // 32-bit data flag

    reg         data_valid;  // Data valid flag
    reg  [ 1:0] data_cnt;    // Data byte counter

    wire [ 2:0] head_bytes_max;  // Maximum header bytes

    reg  [15:0] regaddr;     // Register address
    reg  [15:0] ramaddr;     // RAM address
    reg  [ 7:0] fifo_rdlen;  // FIFO read length

    reg  [31:0] wrdt;        // Write data register
    reg         wren;        // Write enable
    reg  [15:0] reg_wraddr;  // Register write address
    reg  [15:0] ram_wraddr;  // RAM write address

    reg         rden;        // Read enable
    reg  [15:0] reg_rdaddr;  // Register read address
    reg  [15:0] ram_rdaddr;  // RAM read address
    reg  [31:0] dout;        // Data output register
    reg         spiso_p;     // SPI output register

    reg  [ 3:0] wren_p;      // Write enable pipeline
    reg  [ 3:0] rden_p;      // Read enable pipeline
    reg         spi_wren_p;  // SPI write enable
    reg  [31:0] spi_wrdt_p;  // SPI write data
    reg         spi_rden_p;  // SPI read enable
    reg  [15:0] spi_reg_addr_p;  // SPI register address
    reg  [15:0] spi_ram_addr_p;  // SPI RAM address
    reg  [31:0] dout_p;      // Data output pipeline


    // SPI input data sampling
    always @(*) begin
        spi_din[0] <= spisi;     // Sample input bit on current clock cycle
    end

    // SPI input data shifting
    always @(posedge spisck) begin
        spi_din[7:1] <= spi_din[6:0];  // Shift data left on each SPI clock
    end

    // SPI bit and header counter logic
    always @(posedge spisck or posedge spics or posedge rst) begin

        if (rst || spics) begin
            bit_cnt  <= 3'd0;     // Reset bit counter
            head_cnt <= 3'd0;     // Reset header counter
            bit_ed   <= 1'b0;     // Reset bit end flag
        end else begin
            bit_cnt  <= bit_cnt + 1;     // Increment bit counter
            head_cnt <= (bit_ed && head_cnt != 7) ? head_cnt + 1 : head_cnt;  // Increment header counter
            bit_ed   <= (bit_cnt == 6) ? 1'b1 : 1'b0;  // Set bit end flag
        end

    end

    // Function code register
    always @(posedge spisck or posedge rst or posedge spics) begin

        if (rst || spics) begin
            func <= 0;     // Reset function code
        end else begin
            func <= (head_cnt == 0 && bit_ed) ? spi_din : func;  // Capture function code
        end

    end

    // Function code decoding
    assign is_read = func[7];            // Read operation if bit 7 is set
    assign is_reg = (~func[6]) & (~func[5]);  // Register access
    assign is_ram = (~func[6]) & (func[5]);   // RAM access
    assign is_fifo = (func[6]) & (~func[5]);  // FIFO access
    assign is_8 = (~func[4]) & (~func[3]);    // 8-bit data
    assign is_16 = (~func[4]) & func[3];      // 16-bit data
    assign is_32 = func[4] & (~func[3]);      // 32-bit data

    // Determine maximum header bytes based on operation type
    assign head_bytes_max = is_ram ? 4 : ((is_fifo && is_read) ? 3 : 2);

    // Data valid and counter logic
    always @(posedge spisck or posedge spics or posedge rst) begin
        if (rst || spics) begin
            data_valid <= 1'b0;     // Reset data valid flag
            data_cnt   <= 2'd0;     // Reset data counter
        end else begin

            data_valid <= (bit_ed && head_cnt == head_bytes_max) ? 1'b1 : data_valid;  // Set data valid flag

            if (!data_valid) data_cnt <= 2'd0;     // Reset data counter if not valid
            else if (bit_ed) data_cnt <= data_cnt + 1;  // Increment data counter
            else data_cnt <= data_cnt;  // Hold data counter
        end
    end



    // Address handling logic
    always @(posedge spisck) begin

        // Register address handling
        if (head_cnt == 1 && bit_ed) regaddr <= {8'd0, spi_din};  // LSB first
        else if (head_cnt == 2 && bit_ed) regaddr <= {spi_din, regaddr[7:0]};  // MSB second
        else if (is_reg && data_valid && bit_ed && (is_8 || (is_16 && data_cnt[0]) || (is_32 && data_cnt == 2'b11)))
            regaddr <= regaddr + 1;  // Increment address for burst operations
        else regaddr <= regaddr;  // Hold address

        // RAM address handling
        if (is_ram && head_cnt == 3 && bit_ed) ramaddr <= {8'd0, spi_din};  // LSB first
        else if (is_ram && head_cnt == 4 && bit_ed) ramaddr <= {spi_din, ramaddr[7:0]};  // MSB second
        else if (is_ram && data_valid && bit_ed && (is_8 || (is_16 && data_cnt[0]) || (is_32 && data_cnt == 2'b11)))
            ramaddr <= ramaddr + 1;  // Increment address for burst operations
        else ramaddr <= ramaddr;  // Hold address

        // FIFO read length handling
        if (is_fifo && is_read && head_cnt == 3 && bit_ed) fifo_rdlen <= spi_din;  // Set FIFO read length
        else if (is_fifo && is_read && data_valid && fifo_rdlen && rden)
            fifo_rdlen <= fifo_rdlen - 1;  // Decrement read length
        else fifo_rdlen <= fifo_rdlen;  // Hold read length

    end


    // Write data handling
    always @(posedge spisck) begin

        // Write data byte 0 (LSB)
        if (!is_read && data_valid && bit_ed && (is_8 || (is_16 && data_cnt[0] == 1'b0) || (is_32 && data_cnt[1:0] == 2'b00)))
            wrdt[7:0] <= spi_din;
        else wrdt[7:0] <= wrdt[7:0];

        // Write data byte 1
        if (!is_read && data_valid && bit_ed && ((is_16 && data_cnt[0] == 1'b1) || (is_32 && data_cnt[1:0] == 2'b01)))
            wrdt[15:8] <= spi_din;
        else wrdt[15:8] <= wrdt[15:8];

        // Write data byte 2
        if (!is_read && data_valid && bit_ed && (is_32 && data_cnt[1:0] == 2'b10))
            wrdt[23:16] <= spi_din;
        else wrdt[23:16] <= wrdt[23:16];

        // Write data byte 3 (MSB)
        if (!is_read && data_valid && bit_ed && (is_32 && data_cnt[1:0] == 2'b11))
            wrdt[31:24] <= spi_din;
        else wrdt[31:24] <= wrdt[31:24];

        // Write enable and address latching
        if (!is_read && data_valid && bit_ed && (is_8 || (is_16 && data_cnt[0]) || (is_32 && data_cnt == 2'b11))) begin
            wren <= 1'b1;           // Set write enable
            reg_wraddr <= regaddr;  // Latch register write address
            ram_wraddr <= ramaddr;  // Latch RAM write address
        end else begin
            wren <= 1'b0;           // Clear write enable
            reg_wraddr <= reg_wraddr;  // Hold register write address
            ram_wraddr <= ram_wraddr;  // Hold RAM write address
        end

    end

    // Read data handling
    always @(posedge spisck) begin
        // Read enable and address latching
        if (is_read && data_valid && (bit_cnt == 0) && (!is_fifo || fifo_rdlen) && (is_8 || (is_16 && data_cnt[0] == 0) || (is_32 && data_cnt[1:0] == 0))) begin
            rden <= 1'b1;           // Set read enable
            reg_rdaddr <= regaddr;  // Latch register read address
            ram_rdaddr <= ramaddr;  // Latch RAM read address
        end else begin
            rden <= 1'b0;           // Clear read enable
            reg_rdaddr <= reg_rdaddr;  // Hold register read address
            ram_rdaddr <= ram_rdaddr;  // Hold RAM read address
        end

        // Read data shifting
        if (is_read && data_valid && bit_ed && (is_8 || (is_16 && data_cnt[0] == 0) || (is_32 && data_cnt[1:0] == 0)))
            dout <= dout_p;  // Load new read data
        else if (is_read && data_valid && bit_ed) dout <= {8'hff, dout[31:8]};  // Shift data for next bit
        else dout <= {dout[31:8], dout[6:0], 1'b1};  // Default shift

    end

    // SPI output handling
    always @(negedge spisck) begin
        spiso_p <= dout[7];  // Output MSB first
    end

    // System clock domain processing
    always @(posedge sysclk) begin

        wren_p <= {wren_p[2:0], wren};  // Pipeline write enable
        rden_p <= {rden_p[2:0], rden};  // Pipeline read enable

        // SPI write enable and data latching
        if (wren_p[3:2] == 2'b01) begin
            spi_wren_p <= 1'b1;  // Set SPI write enable
            spi_wrdt_p <= wrdt;  // Latch write data
        end else begin
            spi_wren_p <= 1'b0;  // Clear SPI write enable
            spi_wrdt_p <= spi_wrdt_p;  // Hold write data
        end

        spi_rden_p <= (~rden_p[3]) & rden_p[2];  // Generate SPI read enable

        // Address latching
        if (wren_p[3:2] == 2'b01) begin
            spi_reg_addr_p <= reg_wraddr;  // Latch register write address
            spi_ram_addr_p <= ram_wraddr;  // Latch RAM write address
        end else if (rden_p[3:2] == 2'b01) begin
            spi_reg_addr_p <= reg_rdaddr;  // Latch register read address
            spi_ram_addr_p <= ram_rdaddr;  // Latch RAM read address
        end else begin
            spi_reg_addr_p <= spi_reg_addr_p;  // Hold register address
            spi_ram_addr_p <= spi_ram_addr_p;  // Hold RAM address
        end

        dout_p <= spi_rdv ? spi_rddt : dout_p;  // Latch read data when valid

    end

    // Output assignments
    assign spi_reg_addr = spi_reg_addr_p;
    assign spi_ram_addr = spi_ram_addr_p;
    assign spi_wren     = spi_wren_p;
    assign spi_wrdt     = spi_wrdt_p;
    assign spi_rdreq    = spi_rden_p;

    assign spiso        = spics ? 1'bz : spiso_p;  // Tri-state output when chip select is high

    // Simulation dump initialization
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(1, mcu_spi_interface);
    end

endmodule