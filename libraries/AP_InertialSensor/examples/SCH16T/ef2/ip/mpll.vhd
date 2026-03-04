-------------------------------------------------------------------------------
--	Input frequency:               24.000000MHz
--	Clock multiplication factor: 5
--	Clock division factor:       1
--	Clock information:
--		Clock name	| Frequency 	| Phase shift
--		C0        	| 120.000000MHZ	| 0.0000  DEG  
-------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE ieee.std_logic_unsigned.ALL;
USE ieee.std_logic_arith.ALL;
LIBRARY ef2_macro;
USE ef2_macro.EF2_COMPONENTS.ALL;

ENTITY mpll IS
  PORT (
    refclk : IN STD_LOGIC;
    reset : IN STD_LOGIC;
    extlock : OUT STD_LOGIC;
    clk0_out : OUT STD_LOGIC 
  );
END mpll;

ARCHITECTURE rtl OF mpll IS
  SIGNAL clk0_buf :  STD_LOGIC;
  SIGNAL fbk_wire :  STD_LOGIC;
  SIGNAL clkc_wire :  STD_LOGIC_VECTOR (6 DOWNTO 0);
BEGIN
  bufg_feedback : EF2_LOGIC_BUFG
  PORT MAP (
    i => clk0_buf,
    o => fbk_wire 
  );

  pll_inst : EF2_PHY_PLL
  GENERIC MAP (
    DPHASE_SOURCE => "DISABLE",
    DYNCFG => "DISABLE",
    FIN => "24.000000",
    FEEDBK_MODE => "NORMAL",
    FEEDBK_PATH => "CLKC0_EXT",
    STDBY_ENABLE => "DISABLE",
    PLLRST_ENA => "ENABLE",
    SYNC_ENABLE => "DISABLE",
    GMC_GAIN => 2,
    ICP_CURRENT => 9,
    KVCO => 2,
    LPF_CAPACITOR => 1,
    LPF_RESISTOR => 8,
    REFCLK_DIV => 1,
    FBCLK_DIV => 5,
    CLKC0_ENABLE => "ENABLE",
    CLKC0_DIV => 8,
    CLKC0_CPHASE => 7,
    CLKC0_FPHASE => 0,
    PU_INTP => "DISABLE",
    INTPI => 0,
    HIGH_SPEED_EN => "DISABLE",
    SSC_ENABLE => "DISABLE",
    SSC_MODE => "Down",
    SSC_AMP => 0.0000,
    SSC_FREQ_DIV => 0,
    SSC_RNGE => 0,
    FREQ_OFFSET => 0.0000,
    OFFSET_MODE => "EXT",
    FREQ_OFFSET_INT => 0,
    CLKC0_DUTY => 0.5000,
    CLKC0_DUTY_INT => 4,
    CLKC0_DUTY50 => "ENABLE" 
  )
  PORT MAP (
    refclk => refclk,
    reset => reset,
    stdby => '0',
    extlock => extlock,
    load_reg => '0',
    psclk => '0',
    psdown => '0',
    psstep => '0',
    psclksel => b"000",
    dclk => '0',
    dcs => '0',
    dwe => '0',
    di => b"00000000",
    daddr => b"000000",
    fbclk => fbk_wire,
    clkc => clkc_wire,
    ssc_en => '0',
    frac_offset_valid => '0',
    dsm_refclk => '0',
    dsm_rst => reset 
  );

  clk0_out <= fbk_wire;
  clk0_buf <= clkc_wire(0);

END rtl;

