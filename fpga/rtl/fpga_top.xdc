## Hamgeek Zynq-7020 minimal CPU constraints for block design top

## LEDs only - adjust name if get_ports shows leds_0 instead of leds
set_property PACKAGE_PIN W13 [get_ports {leds[0]}]
set_property PACKAGE_PIN V12 [get_ports {leds[1]}]
set_property PACKAGE_PIN U12 [get_ports {leds[2]}]
set_property PACKAGE_PIN T12 [get_ports {leds[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {leds[*]}]