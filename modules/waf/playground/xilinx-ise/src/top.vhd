--+-------------------------------------------------------------------------------------------------+
--|
--|	VHDL example for waf build automation tool
--|
--+-------------------------------------------------------------------------------------------------+

library ieee;
use ieee.std_logic_1164.all;

entity waf_demo is
port (
	-- buttons
    BUTTON_1 : in std_logic;
    BUTTON_2 : in std_logic;

	-- leds
	DLED_2 : out std_logic;
	DLED_3 : out std_logic;
	DLED_4 : out std_logic;
	DLED_5 : out std_logic
);
end waf_demo;

architecture inside of waf_demo is 

begin

	DLED_5 <= '1';
	DLED_4 <= BUTTON_2;
	DLED_3 <= '0';
	DLED_2 <= not BUTTON_1;

end inside;

