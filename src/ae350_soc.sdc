//Copyright (C)2014-2024 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.9.03 (64-bit) 
//Created Time: 2024-05-14 15:38:25
create_clock -name clk50m -period 20 -waveform {0 10} [get_ports {CLK}]
create_clock -name ae350_ddr_clk -period 10 -waveform {0 5} [get_pins {u_Gowin_PLL_AE350/PLL_inst/CLKOUT0}]
create_clock -name ae350_ahb_clk -period 10 -waveform {0 5} [get_pins {u_Gowin_PLL_AE350/PLL_inst/CLKOUT2}]
create_clock -name ae350_apb_clk -period 10 -waveform {0 5} [get_pins {u_Gowin_PLL_AE350/PLL_inst/CLKOUT3}]
create_clock -name flash_sysclk -period 20 -waveform {0 10} [get_nets {u_RiscV_AE350_SOC_Top/FLASH_SPI_CLK_in}]
create_clock -name ddr3_clkin -period 20 -waveform {0 10} [get_pins {u_Gowin_PLL_DDR/PLL_inst/CLKOUT0}]
create_clock -name ddr3_memory_clk -period 4 -waveform {0 2} [get_pins {u_Gowin_PLL_DDR/PLL_inst/CLKOUT2}]
# create_clock -name ddr3_sysclk -period 10 -waveform {0 5} [get_pins {u_RiscV_AE350_SOC_Top/u_RiscV_AE350_SOC/u_riscv_ae350_ddr3_top/u_ddr3_memory_ahb_top/u_ddr3/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]
create_generated_clock -name ddr3_sysclk -source [get_pins {u_Gowin_PLL_DDR/PLL_inst/CLKOUT2}] -master_clock ddr3_memory_clk -divide_by 4 -multiply_by 1 -add [get_pins {u_RiscV_AE350_SOC_Top/u_RiscV_AE350_SOC/u_riscv_ae350_ddr3_top/u_ddr3_memory_ahb_top/u_ddr3/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]
create_clock -name gtx_clk -period 8 -waveform {0 4} [get_pins {u_Gowin_PLL/PLL_inst/CLKOUT1}]
create_clock -name rgmii_rxclk -period 20 -waveform {0 10} [get_ports {RGMII_RXC}]

set_clock_groups -exclusive -group [get_clocks {rgmii_rxclk}] -group [get_clocks {gtx_clk}] -group [get_clocks {flash_sysclk}] -group [get_clocks {ae350_ahb_clk}] -group [get_clocks {ae350_apb_clk}] -group [get_clocks {ae350_ddr_clk}]
set_clock_groups -asynchronous -group [get_clocks {ddr3_clkin}] -group [get_clocks {ddr3_sysclk}]
set_clock_groups -asynchronous -group [get_clocks {ddr3_sysclk}] -group [get_clocks {ddr3_memory_clk}]
set_clock_groups -exclusive -group [get_clocks {ddr3_memory_clk}] -group [get_clocks {ddr3_clkin}]
set_clock_groups -exclusive -group [get_clocks {clk50m}] -group [get_clocks {ddr3_sysclk}]
set_operating_conditions -grade c -model slow -speed 2 -setup -hold
