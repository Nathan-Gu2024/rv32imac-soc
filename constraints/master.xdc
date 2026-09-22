## Hamgeek Zynq-7020 minimal CPU constraints for block design top
set_property PACKAGE_PIN T12 [get_ports {led[3]}]
set_property PACKAGE_PIN U12 [get_ports {led[2]}]
set_property PACKAGE_PIN V12 [get_ports {led[1]}]
set_property PACKAGE_PIN W13 [get_ports {led[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]

set_property PACKAGE_PIN T16 [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]

set_property PACKAGE_PIN T17 [get_ports uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]

set_property MAX_FANOUT 32 [get_nets -hierarchical -filter {NAME =~ "*CPU_CORE/ICACHE/core/cache_req_addr*"}]


#create_debug_core u_ila_0 ila
#set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_0]
#set_property ALL_PROBE_SAME_MU_CNT 4 [get_debug_cores u_ila_0]
#set_property C_ADV_TRIGGER true [get_debug_cores u_ila_0]
#set_property C_DATA_DEPTH 1024 [get_debug_cores u_ila_0]
#set_property C_EN_STRG_QUAL false [get_debug_cores u_ila_0]
#set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_0]
#set_property C_TRIGIN_EN false [get_debug_cores u_ila_0]
#set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0]
#set_property port_width 1 [get_debug_ports u_ila_0/clk]
#connect_debug_port u_ila_0/clk [get_nets [list design_1_i/processing_system7_0/inst/FCLK_CLK0]]
#set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe0]
#set_property port_width 32 [get_debug_ports u_ila_0/probe0]
#connect_debug_port u_ila_0/probe0 [get_nets [list {design_1_i/fpga_top_0/inst/debug_instr[0]} {design_1_i/fpga_top_0/inst/debug_instr[1]} {design_1_i/fpga_top_0/inst/debug_instr[2]} {design_1_i/fpga_top_0/inst/debug_instr[3]} {design_1_i/fpga_top_0/inst/debug_instr[4]} {design_1_i/fpga_top_0/inst/debug_instr[5]} {design_1_i/fpga_top_0/inst/debug_instr[6]} {design_1_i/fpga_top_0/inst/debug_instr[7]} {design_1_i/fpga_top_0/inst/debug_instr[8]} {design_1_i/fpga_top_0/inst/debug_instr[9]} {design_1_i/fpga_top_0/inst/debug_instr[10]} {design_1_i/fpga_top_0/inst/debug_instr[11]} {design_1_i/fpga_top_0/inst/debug_instr[12]} {design_1_i/fpga_top_0/inst/debug_instr[13]} {design_1_i/fpga_top_0/inst/debug_instr[14]} {design_1_i/fpga_top_0/inst/debug_instr[15]} {design_1_i/fpga_top_0/inst/debug_instr[16]} {design_1_i/fpga_top_0/inst/debug_instr[17]} {design_1_i/fpga_top_0/inst/debug_instr[18]} {design_1_i/fpga_top_0/inst/debug_instr[19]} {design_1_i/fpga_top_0/inst/debug_instr[20]} {design_1_i/fpga_top_0/inst/debug_instr[21]} {design_1_i/fpga_top_0/inst/debug_instr[22]} {design_1_i/fpga_top_0/inst/debug_instr[23]} {design_1_i/fpga_top_0/inst/debug_instr[24]} {design_1_i/fpga_top_0/inst/debug_instr[25]} {design_1_i/fpga_top_0/inst/debug_instr[26]} {design_1_i/fpga_top_0/inst/debug_instr[27]} {design_1_i/fpga_top_0/inst/debug_instr[28]} {design_1_i/fpga_top_0/inst/debug_instr[29]} {design_1_i/fpga_top_0/inst/debug_instr[30]} {design_1_i/fpga_top_0/inst/debug_instr[31]}]]
#create_debug_port u_ila_0 probe
#set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe1]
#set_property port_width 32 [get_debug_ports u_ila_0/probe1]
#connect_debug_port u_ila_0/probe1 [get_nets [list {design_1_i/fpga_top_0/inst/debug_raw_pc[0]} {design_1_i/fpga_top_0/inst/debug_raw_pc[1]} {design_1_i/fpga_top_0/inst/debug_raw_pc[2]} {design_1_i/fpga_top_0/inst/debug_raw_pc[3]} {design_1_i/fpga_top_0/inst/debug_raw_pc[4]} {design_1_i/fpga_top_0/inst/debug_raw_pc[5]} {design_1_i/fpga_top_0/inst/debug_raw_pc[6]} {design_1_i/fpga_top_0/inst/debug_raw_pc[7]} {design_1_i/fpga_top_0/inst/debug_raw_pc[8]} {design_1_i/fpga_top_0/inst/debug_raw_pc[9]} {design_1_i/fpga_top_0/inst/debug_raw_pc[10]} {design_1_i/fpga_top_0/inst/debug_raw_pc[11]} {design_1_i/fpga_top_0/inst/debug_raw_pc[12]} {design_1_i/fpga_top_0/inst/debug_raw_pc[13]} {design_1_i/fpga_top_0/inst/debug_raw_pc[14]} {design_1_i/fpga_top_0/inst/debug_raw_pc[15]} {design_1_i/fpga_top_0/inst/debug_raw_pc[16]} {design_1_i/fpga_top_0/inst/debug_raw_pc[17]} {design_1_i/fpga_top_0/inst/debug_raw_pc[18]} {design_1_i/fpga_top_0/inst/debug_raw_pc[19]} {design_1_i/fpga_top_0/inst/debug_raw_pc[20]} {design_1_i/fpga_top_0/inst/debug_raw_pc[21]} {design_1_i/fpga_top_0/inst/debug_raw_pc[22]} {design_1_i/fpga_top_0/inst/debug_raw_pc[23]} {design_1_i/fpga_top_0/inst/debug_raw_pc[24]} {design_1_i/fpga_top_0/inst/debug_raw_pc[25]} {design_1_i/fpga_top_0/inst/debug_raw_pc[26]} {design_1_i/fpga_top_0/inst/debug_raw_pc[27]} {design_1_i/fpga_top_0/inst/debug_raw_pc[28]} {design_1_i/fpga_top_0/inst/debug_raw_pc[29]} {design_1_i/fpga_top_0/inst/debug_raw_pc[30]} {design_1_i/fpga_top_0/inst/debug_raw_pc[31]}]]
#create_debug_port u_ila_0 probe
#set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe2]
#set_property port_width 32 [get_debug_ports u_ila_0/probe2]
#connect_debug_port u_ila_0/probe2 [get_nets [list {design_1_i/fpga_top_0/inst/debug_pc[0]} {design_1_i/fpga_top_0/inst/debug_pc[1]} {design_1_i/fpga_top_0/inst/debug_pc[2]} {design_1_i/fpga_top_0/inst/debug_pc[3]} {design_1_i/fpga_top_0/inst/debug_pc[4]} {design_1_i/fpga_top_0/inst/debug_pc[5]} {design_1_i/fpga_top_0/inst/debug_pc[6]} {design_1_i/fpga_top_0/inst/debug_pc[7]} {design_1_i/fpga_top_0/inst/debug_pc[8]} {design_1_i/fpga_top_0/inst/debug_pc[9]} {design_1_i/fpga_top_0/inst/debug_pc[10]} {design_1_i/fpga_top_0/inst/debug_pc[11]} {design_1_i/fpga_top_0/inst/debug_pc[12]} {design_1_i/fpga_top_0/inst/debug_pc[13]} {design_1_i/fpga_top_0/inst/debug_pc[14]} {design_1_i/fpga_top_0/inst/debug_pc[15]} {design_1_i/fpga_top_0/inst/debug_pc[16]} {design_1_i/fpga_top_0/inst/debug_pc[17]} {design_1_i/fpga_top_0/inst/debug_pc[18]} {design_1_i/fpga_top_0/inst/debug_pc[19]} {design_1_i/fpga_top_0/inst/debug_pc[20]} {design_1_i/fpga_top_0/inst/debug_pc[21]} {design_1_i/fpga_top_0/inst/debug_pc[22]} {design_1_i/fpga_top_0/inst/debug_pc[23]} {design_1_i/fpga_top_0/inst/debug_pc[24]} {design_1_i/fpga_top_0/inst/debug_pc[25]} {design_1_i/fpga_top_0/inst/debug_pc[26]} {design_1_i/fpga_top_0/inst/debug_pc[27]} {design_1_i/fpga_top_0/inst/debug_pc[28]} {design_1_i/fpga_top_0/inst/debug_pc[29]} {design_1_i/fpga_top_0/inst/debug_pc[30]} {design_1_i/fpga_top_0/inst/debug_pc[31]}]]
#create_debug_port u_ila_0 probe
#set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe3]
#set_property port_width 27 [get_debug_ports u_ila_0/probe3]
#connect_debug_port u_ila_0/probe3 [get_nets [list {design_1_i/fpga_top_0/inst/heartbeat[0]} {design_1_i/fpga_top_0/inst/heartbeat[1]} {design_1_i/fpga_top_0/inst/heartbeat[2]} {design_1_i/fpga_top_0/inst/heartbeat[3]} {design_1_i/fpga_top_0/inst/heartbeat[4]} {design_1_i/fpga_top_0/inst/heartbeat[5]} {design_1_i/fpga_top_0/inst/heartbeat[6]} {design_1_i/fpga_top_0/inst/heartbeat[7]} {design_1_i/fpga_top_0/inst/heartbeat[8]} {design_1_i/fpga_top_0/inst/heartbeat[9]} {design_1_i/fpga_top_0/inst/heartbeat[10]} {design_1_i/fpga_top_0/inst/heartbeat[11]} {design_1_i/fpga_top_0/inst/heartbeat[12]} {design_1_i/fpga_top_0/inst/heartbeat[13]} {design_1_i/fpga_top_0/inst/heartbeat[14]} {design_1_i/fpga_top_0/inst/heartbeat[15]} {design_1_i/fpga_top_0/inst/heartbeat[16]} {design_1_i/fpga_top_0/inst/heartbeat[17]} {design_1_i/fpga_top_0/inst/heartbeat[18]} {design_1_i/fpga_top_0/inst/heartbeat[19]} {design_1_i/fpga_top_0/inst/heartbeat[20]} {design_1_i/fpga_top_0/inst/heartbeat[21]} {design_1_i/fpga_top_0/inst/heartbeat[22]} {design_1_i/fpga_top_0/inst/heartbeat[23]} {design_1_i/fpga_top_0/inst/heartbeat[24]} {design_1_i/fpga_top_0/inst/heartbeat[25]} {design_1_i/fpga_top_0/inst/heartbeat[26]}]]
#create_debug_port u_ila_0 probe
#set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe4]
#set_property port_width 1 [get_debug_ports u_ila_0/probe4]
#connect_debug_port u_ila_0/probe4 [get_nets [list design_1_i/fpga_top_0/inst/debug_tcm_d_ready]]
#create_debug_port u_ila_0 probe
#set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe5]
#set_property port_width 1 [get_debug_ports u_ila_0/probe5]
#connect_debug_port u_ila_0/probe5 [get_nets [list design_1_i/fpga_top_0/inst/debug_tcm_d_req]]
#set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
#set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
#set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
#connect_debug_port dbg_hub/clk [get_nets clk]

