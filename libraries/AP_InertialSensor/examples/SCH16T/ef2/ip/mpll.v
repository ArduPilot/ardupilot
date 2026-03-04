///////////////////////////////////////////////////////////////////////////////
//	Input frequency:               24.000000MHz
//	Clock multiplication factor: 5
//	Clock division factor:       1
//	Clock information:
//		Clock name	| Frequency 	| Phase shift
//		C0        	| 120.000000MHZ	| 0.0000  DEG  
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ns / 100 fs

module mpll (
  refclk,
  reset,
  extlock,
  clk0_out 
);

  input refclk;
  input reset;
  output extlock;
  output clk0_out;

  wire clk0_buf;

  EF2_LOGIC_BUFG bufg_feedback (
    .i(clk0_buf),
    .o(clk0_out) 
  );

  EF2_PHY_PLL #(
    .DPHASE_SOURCE("DISABLE"),
    .DYNCFG("DISABLE"),
    .FIN("24.000000"),
    .FEEDBK_MODE("NORMAL"),
    .FEEDBK_PATH("CLKC0_EXT"),
    .STDBY_ENABLE("DISABLE"),
    .PLLRST_ENA("ENABLE"),
    .SYNC_ENABLE("DISABLE"),
    .GMC_GAIN(2),
    .ICP_CURRENT(9),
    .KVCO(2),
    .LPF_CAPACITOR(1),
    .LPF_RESISTOR(8),
    .REFCLK_DIV(1),
    .FBCLK_DIV(5),
    .CLKC0_ENABLE("ENABLE"),
    .CLKC0_DIV(8),
    .CLKC0_CPHASE(7),
    .CLKC0_FPHASE(0),
    .PU_INTP("DISABLE"),
    .INTPI(0),
    .HIGH_SPEED_EN("DISABLE"),
    .SSC_ENABLE("DISABLE"),
    .SSC_MODE("Down"),
    .SSC_AMP(0.0000),
    .SSC_FREQ_DIV(0),
    .SSC_RNGE(0),
    .FREQ_OFFSET(0.0000),
    .OFFSET_MODE("EXT"),
    .FREQ_OFFSET_INT(0),
    .CLKC0_DUTY(0.5000),
    .CLKC0_DUTY_INT(4),
    .CLKC0_DUTY50("ENABLE") 
  ) pll_inst (
    .refclk(refclk),
    .reset(reset),
    .stdby(1'b0),
    .extlock(extlock),
    .load_reg(1'b0),
    .psclk(1'b0),
    .psdown(1'b0),
    .psstep(1'b0),
    .psclksel(3'b000),
    .psdone(open),
    .dclk(1'b0),
    .dcs(1'b0),
    .dwe(1'b0),
    .di(8'b00000000),
    .daddr(6'b000000),
    .do({open, open, open, open, open, open, open, open}),
    .fbclk(clk0_out),
    .clkc({open, open, open, open, open, open, clk0_buf}),
    .ssc_en(1'b0),
    .frac_offset_valid(1'b0),
    .dsm_refclk(1'b0),
    .dsm_rst(reset) 
  );

endmodule

