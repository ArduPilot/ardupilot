LIBRARY ieee;
USE work.ALL;
	USE ieee.std_logic_1164.all;
LIBRARY ef2_macro;
	USE ef2_macro.EF2_COMPONENTS.all;

ENTITY ram_sensor_dircet IS
PORT (
	dob	: OUT STD_LOGIC_VECTOR(7 DOWNTO 0);

	dia	: IN STD_LOGIC_VECTOR(7 DOWNTO 0);
	addra	: IN STD_LOGIC_VECTOR(5 DOWNTO 0);
	cea	: IN STD_LOGIC;
	clka	: IN STD_LOGIC;
	addrb	: IN STD_LOGIC_VECTOR(5 DOWNTO 0);
	clkb	: IN STD_LOGIC
	);
END ram_sensor_dircet;

ARCHITECTURE struct OF ram_sensor_dircet IS

	BEGIN
	inst : EF2_LOGIC_BRAM
		GENERIC MAP (
			DATA_WIDTH_A	=> 8,
			DATA_WIDTH_B	=> 8,
			ADDR_WIDTH_A	=> 6,
			ADDR_WIDTH_B	=> 6,
			DATA_DEPTH_A	=> 64,
			DATA_DEPTH_B	=> 64,
			MODE		=> "PDPW",
			REGMODE_A	=> "NOREG",
			REGMODE_B	=> "NOREG",
			WRITEMODE_A	=> "NORMAL",
			WRITEMODE_B	=> "NORMAL",
			RESETMODE	=> "SYNC",
			IMPLEMENT		=> "9K",
			INIT_FILE		=> "NONE",
			FILL_ALL		=> "NONE"
		)
		PORT MAP (
			dia	=> dia,
			dib	=> (others=>'0'),
			addra	=> addra,
			addrb	=> addrb,
			cea	=> cea,
			ceb	=> '1',
			clka	=> clka,
			clkb	=> clkb,
			wea	=> '1',
			bea	=> (others=>'0'),
			web	=> '0',
			beb	=> (others=>'0'),
			ocea	=> '0',
			oceb	=> '0',
			rsta	=> '0',
			rstb	=> '0',
			doa	=> OPEN,
			dob	=> dob
		);

END struct;
