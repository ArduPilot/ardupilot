-- sch16t_fifo.vhd
-- FPGA mode SPI communication module for sensor interface
-- Automatically sends commands and receives responses from sensor
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity sch16t_fifo is
port (
    sysclk              :  in std_logic;          -- System clock
    rst                 :  in std_logic;          -- Reset signal
    enable              :  in std_logic;          -- Enable signal

    baud_sub1           :  in std_logic_vector(7 downto 0);  -- Baud rate divisor - 1
    cmd_num_sub1        :  in std_logic_vector(5 downto 0);  -- Number of commands - 1

    sensor_rdy          :  in std_logic;          -- Sensor ready signal
    spics               : out std_logic;          -- SPI chip select
    spisck              : out std_logic;          -- SPI clock
    spisi               : out std_logic;          -- SPI serial input
    spiso               :  in std_logic;          -- SPI serial output

    rdy_error           : out std_logic;          -- Sensor ready error

    cmd_ram_wren        :  in std_logic;          -- Command RAM write enable
    cmd_ram_wrdt        :  in std_logic_vector(7 downto 0);  -- Command RAM write data
    cmd_ram_wraddr      :  in std_logic_vector(8 downto 0);  -- Command RAM write address

    fifo_rdreq          :  in std_logic;          -- FIFO read request
    fifo_rdvalue        : out std_logic_vector(7 downto 0);  -- FIFO read value
    fifo_pkt_clr        :  in std_logic;          -- FIFO packet clear
    fifo_usedw          : out std_logic_vector(7 downto 0)   -- FIFO used words
);
end sch16t_fifo;

architecture rtl of sch16t_fifo is

    -- 24-bit array type for sensor values
    type std24_array  is array ( natural range <> ) of std_logic_vector(24 - 1 downto 0);

    -- Command RAM component
    component ram_cmd is
    port (
        dob     : out std_logic_vector(7 downto 0);  -- Data out B port
        dia     :  in std_logic_vector(7 downto 0);  -- Data in A port
        addra   :  in std_logic_vector(8 downto 0);  -- Address A port
        cea     :  in std_logic;                     -- Chip enable A port
        clka    :  in std_logic;                     -- Clock A port
        addrb   :  in std_logic_vector(8 downto 0);  -- Address B port
        clkb    :  in std_logic                      -- Clock B port
    );
    end component;

    -- FIFO RAM component
    component ram_fifo is
    port (
        dob     : out std_logic_vector(7 downto 0);  -- Data out B port
        dia     :  in std_logic_vector(31 downto 0); -- Data in A port
        addra   :  in std_logic_vector(9 downto 0);  -- Address A port
        cea     :  in std_logic;                     -- Chip enable A port
        clka    :  in std_logic;                     -- Clock A port
        addrb   :  in std_logic_vector(11 downto 0); -- Address B port
        clkb    :  in std_logic                      -- Clock B port
    );
    end component;

    -- CRC8 component for data validation
    component crc8 is
    port(
        rst     :  in std_logic;                     -- Reset signal
        clk     :  in std_logic;                     -- Clock signal
        bit_v   :  in std_logic;                     -- Bit valid signal
        bit_i   :  in std_logic;                     -- Bit input
        crc_o   : out std_logic_vector(7 downto 0)   -- CRC output
    );
    end component;

    -- State machine constants
    constant S_Idle                 : std_logic_vector(3 downto 0) := "0000";  -- Idle state
    constant S_ReadCmd              : std_logic_vector(3 downto 0) := "0001";  -- Read command state
    constant S_Trans                : std_logic_vector(3 downto 0) := "0010";  -- Transmission state
    constant S_Chk                  : std_logic_vector(3 downto 0) := "0100";  -- Check state
    constant S_Write                : std_logic_vector(3 downto 0) := "1000";  -- Write state

    -- State machine signals
    signal cur_ms                   : std_logic_vector(3 downto 0);  -- Current state
    signal nxt_ms                   : std_logic_vector(3 downto 0);  -- Next state
    signal enable_p                 : std_logic;                    -- Enable pulse
    signal rdy                      : std_logic;                    -- Sensor ready signal
    signal rdy_p                    : std_logic;                    -- Previous ready signal
    signal rdy_chk_cnt              : std_logic_vector(18 downto 0); -- Ready check counter
    signal cmd_cnt                  : std_logic_vector(5 downto 0);  -- Command counter
    signal last_cmd                 : std_logic;                    -- Last command flag

    -- Command RAM signals
    signal cmd_ram_rdaddr           : std_logic_vector(8 downto 0);  -- Command RAM read address
    signal cmd_ram_rddt             : std_logic_vector(7 downto 0);  -- Command RAM read data
    signal read_cnt                 : std_logic_vector(5 downto 0);  -- Read counter
    signal read_end                 : std_logic;                    -- Read end flag
    signal rd_valid                 : std_logic;                    -- Read valid flag
    signal rd_isdata                : std_logic;                    -- Read is data flag
    signal data_offset              : std_logic_vector(3 downto 0);  -- Data offset
    signal status_offset            : std_logic_vector(3 downto 0);  -- Status offset
    signal cmd                      : std_logic_vector(47 downto 0);  -- Command data

    -- SPI transmission signals
    signal baud_cnt                 : std_logic_vector(7 downto 0);  -- Baud rate counter
    signal baud_ed                  : std_logic;                    -- Baud rate end flag
    signal bit_cnt                  : std_logic_vector(2 downto 0);  -- Bit counter
    signal bit_ed                   : std_logic;                    -- Bit end flag
    signal byte_cnt                 : std_logic_vector(2 downto 0);  -- Byte counter
    signal byte_ed                  : std_logic;                    -- Byte end flag
    signal trans_end                : std_logic;                    -- Transmission end flag
    signal so_cnt                   : std_logic_vector(7 downto 0);  -- SPI output sample counter
    signal so_value                 : std_logic;                    -- SPI output sample value
    signal spi_cs_i                 : std_logic;                    -- SPI chip select internal
    signal spi_sck_i                : std_logic;                    -- SPI clock internal
    signal spi_si_i                 : std_logic;                    -- SPI serial input internal
    signal mo_shift                 : std_logic_vector(47 downto 0);  -- Master out shift register
    signal so_shift                 : std_logic_vector(47 downto 0);  -- Slave out shift register
    signal crc_rst                  : std_logic;                    -- CRC reset
    signal crc_v                    : std_logic;                    -- CRC valid
    signal crc_i                    : std_logic;                    -- CRC input
    signal crc_o                    : std_logic_vector(7 downto 0);  -- CRC output
    signal crc_err                  : std_logic;                    -- CRC error

    -- Sensor data signals
    signal crc_error_flag           : std_logic;                    -- CRC error flag
    signal sensor_valid             : std_logic_vector(6 downto 0);  -- Sensor valid flags
    signal sensor_value             : std24_array(6 downto 0);       -- Sensor values
    signal sensor_sta               : std_logic_vector(11 downto 0); -- Sensor status
    signal time_scale               : std_logic_vector(3 downto 0);  -- Time scale

    -- Write process signals
    signal write_cnt                : std_logic_vector(2 downto 0);  -- Write counter
    signal write_end                : std_logic;                    -- Write end flag

    -- FIFO signals
    signal pkt_clr                  : std_logic;                    -- Packet clear
    signal fifo_wrdt                : std_logic_vector(31 downto 0);  -- FIFO write data
    signal fifo_wrdt_p              : std_logic_vector(31 downto 0);  -- FIFO write data pipeline
    signal fifo_wraddr              : std_logic_vector(9 downto 0);  -- FIFO write address
    signal fifo_wren                : std_logic;                    -- FIFO write enable
    signal fifo_wren_p              : std_logic;                    -- FIFO write enable pipeline
    signal wr_enable                : std_logic;                    -- Write enable
    signal fifo_pkt_rdaddr          : std_logic_vector(11 downto 0); -- FIFO packet read address
    signal fifo_rdaddr              : std_logic_vector(11 downto 0); -- FIFO read address
    signal fifo_rddt                : std_logic_vector(7 downto 0);  -- FIFO read data
    signal ram_usedw                : std_logic_vector(7 downto 0);  -- RAM used words
    signal fifo_q                   : std_logic_vector(7 downto 0);  -- FIFO output
    signal fifo_full                : std_logic;                    -- FIFO full flag

    -- Keep attributes for critical signals
    attribute keep : boolean;
    attribute keep of cur_ms, rdy, read_end, cmd_cnt, trans_end, last_cmd, write_end, byte_ed, bit_ed, baud_ed, fifo_wrdt, so_shift, crc_err, crc_o : signal is true;

begin

    -- Command RAM instantiation (IP Invoke)
    ucmd: ram_cmd
    port map (
        dob     => cmd_ram_rddt,  -- Data out to command processor
        dia     => cmd_ram_wrdt,  -- Data in from MCU
        addra   => cmd_ram_wraddr,-- Write address from MCU
        cea     => cmd_ram_wren,  -- Write enable from MCU
        clka    => sysclk,        -- System clock
        addrb   => cmd_ram_rdaddr,-- Read address from state machine
        clkb    => sysclk         -- System clock
    );

    -- Main state machine and control process
    process (sysclk) begin
        if rising_edge(sysclk) then

            -- Update enable_p
            if cur_ms = S_Idle then
                enable_p <= enable;
            else
                enable_p <= enable_p;
            end if;

            -- Update current state
            if rst = '1' or enable_p = '0' then
                cur_ms <= S_Idle;
            else
                cur_ms <= nxt_ms;
            end if;

            -- Update ready signals
            rdy <= sensor_rdy;
            rdy_p <= rdy;

            -- Update time scale
            if rst = '1' then
                time_scale <= (others => '0');
            elsif write_end = '1' then
                time_scale <= time_scale + 1;
            else
                time_scale <= time_scale;
            end if;

            -- Update ready check counter
            if rst = '1' or enable_p = '0' or (rdy_p /= rdy) then
                rdy_chk_cnt <= (others => '0');
            elsif rdy_chk_cnt(18) = '0' then
                rdy_chk_cnt <= rdy_chk_cnt + 1;
            else
                rdy_chk_cnt <= rdy_chk_cnt;
            end if;

            -- Set ready error flag
            rdy_error <= rdy_chk_cnt(18);

        end if;
    end process;

    -- Next state logic
    process (all) begin
        case cur_ms is
            when S_Idle     => if rdy = '1' then nxt_ms <= S_ReadCmd; else nxt_ms <= S_Idle; end if;
            when S_ReadCmd  => if read_end = '1' then nxt_ms <= S_Trans; else nxt_ms <= S_ReadCmd; end if;
            when S_Trans    => if trans_end = '1' then nxt_ms <= S_Chk; else nxt_ms <= S_Trans; end if;
            when S_Chk      => if last_cmd = '1' then nxt_ms <= S_Write; else nxt_ms <= S_ReadCmd; end if;
            when S_Write    => if write_end = '1' then nxt_ms <= S_Idle; else nxt_ms <= S_Write; end if;
            when others     => nxt_ms <= S_Idle;
        end case;
    end process;

    -- Command counter process
    process (sysclk) begin
        if rising_edge(sysclk) then

            -- Reset command counter in idle state
            if cur_ms = S_Idle then
                cmd_cnt <= (others => '0');
            elsif cur_ms = S_Chk then
                cmd_cnt <= cmd_cnt + 1;
            else
                cmd_cnt <= cmd_cnt;
            end if;

            -- Set last command flag
            if cmd_cnt = cmd_num_sub1 then
                last_cmd <= '1';
            else
                last_cmd <= '0';
            end if;

        end if;
    end process;

    -- Command RAM read address generation
    cmd_ram_rdaddr <= cmd_cnt & read_cnt(2 downto 0);

    -- Command read process
    process (sysclk) begin
        if rising_edge(sysclk) then

            -- Reset read counter when not in read state
            if cur_ms /= S_ReadCmd then
                read_cnt <= (others => '0');
            else
                read_cnt <= read_cnt + 1;
            end if;

            -- Set read end flag
            if read_cnt = 28 then
                read_end <= '1';
            else
                read_end <= '0';
            end if;

            -- Read command header
            if read_cnt = 1 then
                rd_valid <= cmd_ram_rddt(7);
                rd_isdata <= cmd_ram_rddt(6);
                data_offset <= cmd_ram_rddt(5 downto 2);
                status_offset(3 downto 2) <= cmd_ram_rddt(1 downto 0);
            else
                rd_valid <= rd_valid;
                rd_isdata <= rd_isdata;
                data_offset <= data_offset;
                status_offset(3 downto 2) <= status_offset(3 downto 2);
            end if;

            -- Read status offset extension
            if read_cnt = 2 then
                status_offset(1 downto 0) <= cmd_ram_rddt(7 downto 6);
            else
                status_offset(1 downto 0) <= status_offset(1 downto 0);
            end if;

            -- Read command data (6 bytes)
            if read_cnt = 3 then
                cmd(47 downto 40) <= cmd_ram_rddt;
            else
                cmd(47 downto 40) <= cmd(47 downto 40);
            end if;
            if read_cnt = 4 then
                cmd(39 downto 32) <= cmd_ram_rddt;
            else
                cmd(39 downto 32) <= cmd(39 downto 32);
            end if;
            if read_cnt = 5 then
                cmd(31 downto 24) <= cmd_ram_rddt;
            else
                cmd(31 downto 24) <= cmd(31 downto 24);
            end if;
            if read_cnt = 6 then
                cmd(23 downto 16) <= cmd_ram_rddt;
            else
                cmd(23 downto 16) <= cmd(23 downto 16);
            end if;
            if read_cnt = 7 then
                cmd(15 downto  8) <= cmd_ram_rddt;
            else
                cmd(15 downto  8) <= cmd(15 downto  8);
            end if;
            if read_cnt = 8 then
                cmd( 7 downto  0) <= cmd_ram_rddt;
            else
                cmd( 7 downto  0) <= cmd( 7 downto  0);
            end if;

        end if;
    end process;

    -- SPI timing and transmission process
    process (sysclk) begin
        if rising_edge(sysclk) then

            -- Baud rate generation
            if cur_ms /= S_Trans or baud_ed = '1' then
                baud_cnt <= (others => '0');
            else
                baud_cnt <= baud_cnt + 1;
            end if;

            if baud_cnt + 1 = baud_sub1 then
                baud_ed <= '1';
            else
                baud_ed <= '0';
            end if;

            -- Bit counter
            if cur_ms /= S_Trans then
                bit_cnt <= (others => '0');
            elsif baud_ed = '1' then
                bit_cnt <= bit_cnt + 1;
            else
                bit_cnt <= bit_cnt;
            end if;

            if bit_cnt = 7 then
                bit_ed <= '1';
            else
                bit_ed <= '0';
            end if;

            -- Byte counter
            if cur_ms /= S_Trans then
                byte_cnt <= (others => '0');
            elsif bit_ed = '1' and baud_ed = '1' then
                byte_cnt <= byte_cnt + 1;
            else
                byte_cnt <= byte_cnt;
            end if;

            if byte_cnt = 5 then
                byte_ed <= '1';
            else
                byte_ed <= '0';
            end if;

            -- SPI output sampling
            if baud_cnt <= baud_sub1(7 downto 1) then
                so_cnt <= (others => '0');
            else
                so_cnt <= so_cnt + spiso;
            end if;

        end if;
    end process;

    -- Transmission end and sample value
    trans_end <= byte_ed and bit_ed and baud_ed;
    so_value <= '1' when so_cnt > baud_sub1(7 downto 2) else '0';

    -- SPI signal generation process
    process (sysclk) begin
        if rising_edge(sysclk) then

            -- SPI chip select
            if cur_ms = S_Trans then
                spi_cs_i <= '0';  -- Active low
            else
                spi_cs_i <= '1';
            end if;

            -- SPI clock
            if baud_cnt > baud_sub1(7 downto 1) then
                spi_sck_i <= '1';
            else
                spi_sck_i <= '0';
            end if;

            -- Master out shift register
            if cur_ms /= S_Trans then
                mo_shift <= cmd;
            elsif baud_ed = '1' then
                mo_shift <= mo_shift(46 downto 0) & '0';
            else
                mo_shift <= mo_shift;
            end if;

            -- SPI serial input (master out)
            spi_si_i <= mo_shift(47);

            -- Slave out shift register
            if baud_ed = '1' then
                so_shift <= so_shift(46 downto 0) & so_value;
            else
                so_shift <= so_shift;
            end if;

        end if;
    end process;

    -- CRC control signals
    crc_rst <= '1' when cur_ms = S_ReadCmd else '0';
    crc_v <= baud_ed;
    crc_i <= so_value;

    -- CRC8 instantiation
    ucrc: crc8
    port map(
        rst     => crc_rst,   -- Reset signal
        clk     => sysclk,    -- System clock
        bit_v   => crc_v,     -- Bit valid
        bit_i   => crc_i,     -- Bit input
        crc_o   => crc_o      -- CRC output
    );

    -- CRC error detection
    crc_err <= '0' when crc_o = 0 else '1';

    -- CRC error flag process
    process (sysclk) begin
        if rising_edge(sysclk) then

            if cur_ms = S_Idle then
                crc_error_flag <= '0';
            elsif cur_ms = S_Chk and rd_valid = '1' and crc_err = '1' then
                crc_error_flag <= '1';
            else
                crc_error_flag <= crc_error_flag;
            end if;

        end if;
    end process;

    -- Sensor value processing
    g_sensor_value: for i in 0 to 6 generate begin

        process (sysclk) begin
            if rising_edge(sysclk) then
                if cur_ms = S_Idle then
                    sensor_valid(i) <= '0';
                    sensor_value(i) <= (others => '0');
                elsif cur_ms = S_Chk and rd_valid = '1' and crc_err = '0' and rd_isdata = '1' and data_offset = i then
                    sensor_valid(i) <= '1';
                    sensor_value(i) <= so_shift(36 downto 33) & so_shift(27 downto 8);
                else
                    sensor_valid(i) <= sensor_valid(i);
                    sensor_value(i) <= sensor_value(i);
                end if;

            end if;
        end process;

    end generate;

    -- Sensor status processing
    g_sensor_sta: for i in 0 to 11 generate begin

        process (sysclk) begin
            if rising_edge(sysclk) then
                if cur_ms = S_Idle then
                    sensor_sta(11 - i) <= '0';
                elsif cur_ms = S_Chk and rd_valid = '1' and crc_err = '0' and rd_isdata = '0' and status_offset = i and so_shift(27 downto 8) = x"ffff" then
                    sensor_sta(11 - i) <= '1';
                else
                    sensor_sta(11 - i) <= sensor_sta(11 - i);
                end if;

            end if;
        end process;

    end generate;

    -- FIFO write process
    process (sysclk) begin
        if rising_edge(sysclk) then

            -- Write enable control
            if cur_ms = S_Chk and last_cmd = '1' then
                wr_enable <= not fifo_full;
            else
                wr_enable <= wr_enable;
            end if;

            -- Write counter
            if cur_ms /= S_Write then
                write_cnt <= (others => '0');
            else
                write_cnt <= write_cnt + 1;
            end if;

            -- Write end flag
            if write_cnt = 4 then
                write_end <= '1';
            else
                write_end <= '0';
            end if;

            -- FIFO write enable
            if cur_ms = S_Write then
                fifo_wren_p <= wr_enable;
            else
                fifo_wren_p <= '0';
            end if;

            -- FIFO write data preparation
            case write_cnt is
                when o"0"   => fifo_wrdt_p <= sensor_valid & crc_error_flag & sensor_value(0);
                when o"1"   => fifo_wrdt_p <= sensor_value(1) & sensor_value(2)(23 downto 16);
                when o"2"   => fifo_wrdt_p <= sensor_value(2)(15 downto 0) & sensor_value(3)(23 downto 8);
                when o"3"   => fifo_wrdt_p <= sensor_value(3)(7 downto 0) & sensor_value(4);
                when o"4"   => fifo_wrdt_p <= sensor_value(5) & sensor_value(6)(23 downto 16);
                when o"5"   => fifo_wrdt_p <= sensor_value(6)(15 downto 0) & sensor_sta & time_scale;
                when others => fifo_wrdt_p <= (others => '0');
            end case;

            -- Byte endian swap for FIFO write data
            fifo_wrdt <= fifo_wrdt_p(7 downto 0) & fifo_wrdt_p(15 downto 8) & fifo_wrdt_p(23 downto 16) & fifo_wrdt_p(31 downto 24);
            fifo_wren <= fifo_wren_p;

            -- FIFO write address
            if rst = '1' then
                fifo_wraddr <= (others => '0');
            elsif fifo_wren = '1' then
                fifo_wraddr <= fifo_wraddr + 1;
            else
                fifo_wraddr <= fifo_wraddr;
            end if;

        end if;
    end process;

    -- FIFO RAM instantiation (IP Invoke)
    ufifo: ram_fifo
    port map (
        dob     => fifo_rddt,   -- Data out to FIFO reader
        dia     => fifo_wrdt,   -- Data in from sensor processor
        addra   => fifo_wraddr, -- Write address from write process
        cea     => fifo_wren,   -- Write enable from write process
        clka    => sysclk,      -- System clock
        addrb   => fifo_rdaddr, -- Read address from read process
        clkb    => sysclk       -- System clock
    );

    -- FIFO read and control process
    pkt_clr <= fifo_pkt_clr when ram_usedw /= 0 else '0';
    process (sysclk) begin
        if rising_edge(sysclk) then

            -- Update FIFO used words counter
            if rst = '1' then
                ram_usedw <= (others => '0');
            elsif write_end = '1' and wr_enable = '1' then
                if pkt_clr = '1' then
                    ram_usedw <= ram_usedw;
                else
                    ram_usedw <= ram_usedw + 1;
                end if;
            elsif pkt_clr = '1' then
                ram_usedw <= ram_usedw - 1;
            else
                ram_usedw <= ram_usedw;
            end if;

            -- Set FIFO full flag
            fifo_full <= ram_usedw(7);

            -- Update packet read address
            if rst = '1' then
                fifo_pkt_rdaddr <= (others => '0');
            elsif pkt_clr = '1' then
                fifo_pkt_rdaddr <= fifo_pkt_rdaddr + 24;
            else
                fifo_pkt_rdaddr <= fifo_pkt_rdaddr;
            end if;

            -- Update FIFO read address
            if rst = '1' then
                fifo_rdaddr <= (others => '0');
            elsif pkt_clr = '1' then
                fifo_rdaddr <= fifo_pkt_rdaddr + 24;
            elsif fifo_rdreq = '1' then
                fifo_rdaddr <= fifo_rdaddr + 1;
            else
                fifo_rdaddr <= fifo_rdaddr;
            end if;

            -- Update FIFO output register
            if fifo_rdreq = '1' then
                fifo_q <= fifo_rddt;
            else
                fifo_q <= fifo_q;
            end if;

        end if;
    end process;

    -- Output assignments
    spics <= spi_cs_i;
    spisck <= spi_sck_i;
    spisi <= spi_si_i;

    fifo_rdvalue <= fifo_q;
    fifo_usedw <= ram_usedw;

end architecture;