# Clock Signal
set_property PACKAGE_PIN X2 [get_ports clk_hz]
set_property IOSTANDARD LVCMOS33 [get_ports clk_hz]

# FIXED: Added name 'sys_clk_pin' and corrected waveform math
create_clock -add -name sys_clk_pin -period 10.000 -waveform {0 5.000} [get_ports clk_hz]

# Buttons (Inputs) - ONLY PL BUTTONS
set_property PACKAGE_PIN K2 [get_ports {btn[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {btn[0]}]

set_property PACKAGE_PIN K3 [get_ports {btn[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {btn[1]}]

# LEDs (Outputs) - ONLY PL LEDS
set_property PACKAGE_PIN D1 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]

set_property PACKAGE_PIN D2 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]

set_property PACKAGE_PIN D3 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]