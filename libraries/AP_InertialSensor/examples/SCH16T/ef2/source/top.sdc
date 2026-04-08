create_clock -name inclk -period 40 -waveform {0 20} [get_ports {inclk}]
create_clock -name spiclk -period 125 -waveform {0 62.5} [get_ports {spisck}]
derive_pll_clocks
set_clock_groups -exclusive -group [get_clocks {spiclk}]
set_clock_groups -exclusive -group [get_clocks {sysclk}]
set_clock_groups -exclusive -group [get_clocks {inclk}]
rename_clock -name {sysclk} -source [get_ports {inclk}] -master_clock {inclk} [get_pins {upll/pll_inst.clkc[0]}]