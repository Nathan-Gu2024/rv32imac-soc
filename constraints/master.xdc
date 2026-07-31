# ## Master Constraints for RV32 CPU on Hamgeek Zynq-7020

# ## 1. System Clock (50 MHz)
# set_property PACKAGE_PIN K17 [get_ports clk] 
# set_property IOSTANDARD LVCMOS33 [get_ports clk]
# create_clock -period 20.000 -name clk [get_ports clk]

# ## 2. System Reset (Active Low)
# set_property PACKAGE_PIN M19 [get_ports rst_n]
# set_property IOSTANDARD LVCMOS33 [get_ports rst_n]

# ## 3. LEDs (For debugging CPU state)
# set_property PACKAGE_PIN W13 [get_ports {led[0]}]
# set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
# set_property PACKAGE_PIN V12 [get_ports {led[1]}]
# set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
# set_property PACKAGE_PIN U12 [get_ports {led[2]}]
# set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
# set_property PACKAGE_PIN T12 [get_ports {led[3]}]
# set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]

## Hamgeek Zynq-7020 minimal CPU constraints

## 50 MHz board clock
set_property PACKAGE_PIN K17 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -period 20.000 -name clk [get_ports clk]

## Active-low reset
set_property PACKAGE_PIN M19 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]

## LEDs
set_property PACKAGE_PIN W13 [get_ports {leds[0]}]
set_property PACKAGE_PIN V12 [get_ports {leds[1]}]
set_property PACKAGE_PIN U12 [get_ports {leds[2]}]
set_property PACKAGE_PIN T12 [get_ports {leds[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[*]}]