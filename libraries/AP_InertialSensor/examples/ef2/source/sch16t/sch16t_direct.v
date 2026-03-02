// sch16t_direct.v
// Direct SPI communication module for sensor interface
// Implements SPI communication with configurable parameters
module sch16t_direct (
    input           sysclk,         // System clock
    input           rst,            // Reset signal
    input           enable,         // Enable signal

    input [7:0]     baud_sub1,      // Baud rate divisor - 1
    input           clr,            // Clear signal
    input           start,          // Start SPI transaction
    input [5:0]     bytes_sub1,     // Number of bytes to transfer - 1
    output          busy,           // Busy indicator

    input           ram_wren,       // RAM write enable
    input [5:0]     ram_wraddr,     // RAM write address
    input [7:0]     ram_wrdt,       // RAM write data
    input [5:0]     ram_rdaddr,     // RAM read address
    output [7:0]    ram_rddt,       // RAM read data

    output          spics,          // SPI chip select
    output          spisck,         // SPI clock
    output          spisi,          // SPI serial input
    input           spiso           // SPI serial output (synthesis keep)
);

    // SPI state registers
    reg         spi_busy;       // SPI busy flag (synthesis keep)
    reg [7:0]   baud_cnt;       // Baud rate counter (synthesis keep)
    reg         baud_ed;        // Baud rate end flag (synthesis keep)
    reg [2:0]   bit_cnt;        // Bit counter (synthesis keep)
    reg         bit_ed;         // Bit end flag (synthesis keep)
    reg [5:0]   byte_cnt;       // Byte counter (synthesis keep)
    reg         byte_ed;        // Byte end flag (synthesis keep)
    reg [7:0]   so_cnt;         // SPI output sample counter (synthesis keep)
    wire        so_value;       // SPI output sample value (synthesis keep)

    // TX RAM signals
    wire        txram_rden;     // TX RAM read enable
    wire [5:0]  txram_rdaddr;   // TX RAM read address
    wire [7:0]  txram_rddt;     // TX RAM read data

    // SPI output registers
    reg         spi_cs_i;       // SPI chip select internal (synthesis keep)
    reg         spi_sck_i;       // SPI clock internal (synthesis keep)
    reg         spi_si_i;       // SPI serial input internal (synthesis keep)

    // RX data registers
    reg [7:0]   spi_rdata_wrdt; // SPI receive data write data (synthesis keep)
    reg [5:0]   spi_rdata_wraddr; // SPI receive data write address (synthesis keep)
    reg         spi_rdata_wren; // SPI receive data write enable (synthesis keep)

    // TX RAM instantiation (IP Invoke)
    // Used for storing data to be transmitted via SPI
    ram_sensor_dircet utx (
        .dia    (ram_wrdt),     // Data in A port (from outside)
        .addra  (ram_wraddr),   // Address A port (from outside)
        .cea    (ram_wren),     // Chip enable A port (from outside)
        .clka   (sysclk),       // Clock A port
        .dob    (txram_rddt),   // Data out B port (to SPI transmitter)
        .addrb  (txram_rdaddr), // Address B port (from SPI state machine)
        .clkb   (sysclk)        // Clock B port
    );

    // SPI state machine and timing control
    always @(posedge sysclk) begin

        // Update busy flag
        if (rst || !enable || clr || (spi_busy && byte_ed && bit_ed && baud_ed))
            spi_busy <= 1'b0;      // Clear busy when reset, disabled, cleared, or transaction complete
        else if (start)
            spi_busy <= 1'b1;      // Set busy when start is asserted
        else
            spi_busy <= spi_busy;  // Maintain current state

        // Baud rate generation
        baud_cnt <= (!spi_busy || baud_ed)? 8'd0 : (baud_cnt + 1);  // Reset counter when not busy or end of baud period
        baud_ed <= (baud_cnt + 1 == baud_sub1) ? 1'b1 : 1'b0;        // Set end flag when counter reaches baud_sub1

        // Bit counter
        bit_cnt <= (!spi_busy) ? 0 : (baud_ed ? bit_cnt + 1 : bit_cnt);  // Increment on baud end
        bit_ed <= (bit_cnt == 7) ? 1'b1 : 1'b0;                          // Set end flag at 8th bit

        // Byte counter
        byte_cnt <= (!spi_busy) ? 0 : ((bit_ed && baud_ed) ? (byte_cnt + 1) : byte_cnt);  // Increment on byte end
        byte_ed <= (byte_cnt == bytes_sub1) ? 1'b1 : 1'b0;                                // Set end flag at last byte

        // SPI output sampling
        so_cnt <= (baud_cnt <= baud_sub1[7:1])  ? 0 : so_cnt + spiso;  // Sample spiso during high clock phase

    end

    // Determine SPI output value based on sampling
    assign so_value = (so_cnt > baud_sub1[7:2]) ? 1'b1 : 1'b0;

    // TX RAM control signals
    assign txram_rden = bit_ed & baud_ed;         // Read next byte at end of current byte
    assign txram_rdaddr = byte_cnt + txram_rden;  // Address for next byte

    // SPI output and receive data handling
    always @(posedge sysclk) begin

        // SPI chip select - active low
        spi_cs_i <= !spi_busy;
        
        // SPI clock generation
        spi_sck_i <= (baud_cnt > baud_sub1[7:1]) ? 1'b1 : 1'b0;

        // SPI serial input data shifting
        case (bit_cnt)
            0       : spi_si_i <= txram_rddt[7];  // MSB first
            1       : spi_si_i <= txram_rddt[6];
            2       : spi_si_i <= txram_rddt[5];
            3       : spi_si_i <= txram_rddt[4];
            4       : spi_si_i <= txram_rddt[3];
            5       : spi_si_i <= txram_rddt[2];
            6       : spi_si_i <= txram_rddt[1];
            default : spi_si_i <= txram_rddt[0];  // LSB
        endcase

        // SPI receive data shifting
        spi_rdata_wrdt <= (baud_ed) ? {spi_rdata_wrdt[6:0], so_value} : spi_rdata_wrdt;
        
        // RX RAM write control
        spi_rdata_wren <= baud_ed && bit_ed;                  // Write at end of byte
        spi_rdata_wraddr <= (baud_ed && bit_ed) ? byte_cnt : spi_rdata_wraddr;  // Write address

    end

    // RX RAM instantiation (IP Invoke)
    // Used for storing data received via SPI
    ram_sensor_dircet urx (
        .dia    (spi_rdata_wrdt),   // Data in A port (from SPI receiver)
        .addra  (spi_rdata_wraddr), // Address A port (from SPI state machine)
        .cea    (spi_rdata_wren),   // Chip enable A port (from SPI state machine)
        .clka   (sysclk),           // Clock A port
        .dob    (ram_rddt),         // Data out B port (to outside)
        .addrb  (ram_rdaddr),       // Address B port (from outside)
        .clkb   (sysclk)            // Clock B port
    );

    // Output assignments
    assign spics = spi_cs_i;       // SPI chip select
    assign spisck = spi_sck_i;     // SPI clock
    assign spisi = spi_si_i;       // SPI serial input
    assign busy = spi_busy;        // Busy indicator


endmodule
