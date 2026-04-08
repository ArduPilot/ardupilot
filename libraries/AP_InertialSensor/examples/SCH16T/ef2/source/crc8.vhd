-- crc8.vhd
-- 8-bit CRC (Cyclic Redundancy Check) module
-- Uses polynomial: x^8 + x^2 + x + 1 (0x07)
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity crc8 is
port(
    rst     :  in std_logic;                     -- Reset signal
    clk     :  in std_logic;                     -- Clock signal
    bit_v   :  in std_logic;                     -- Bit valid signal
    bit_i   :  in std_logic;                     -- Input bit
    crc_o   : out std_logic_vector(7 downto 0)   -- CRC output
);
end entity crc8;

architecture rtl of crc8 is

    signal crc  : std_logic_vector(7 downto 0);  -- Internal CRC register

begin

    -- CRC calculation process
    process (clk) begin
        if rising_edge(clk) then

            -- Reset CRC register to initial value
            if rst = '1' then
                crc <= x"ff";  -- Initial value: 0xff
            elsif bit_v = '1' then
                -- CRC update logic
                if crc(7) = '1' then
                    -- If MSB is 1, shift and XOR with polynomial
                    crc(7 downto 1) <= crc(6 downto 0) xor "0010111";
                    crc(0) <= not bit_i;
                else
                    -- If MSB is 0, just shift in the new bit
                    crc <= crc(6 downto 0) & bit_i;
                end if;
            else
                -- No valid bit, hold current CRC value
                crc <= crc;
            end if;

        end if;
    end process;

    -- Output the current CRC value
    crc_o <= crc;

end rtl;