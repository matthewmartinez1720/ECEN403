
## 12 MHz onboard oscillator
set_property -dict { PACKAGE_PIN M9 IOSTANDARD LVCMOS33 } [get_ports { clk_12mhz }]
create_clock -name sys_clk -period 83.333 [get_ports { clk_12mhz }]

## USB-UART bridge
## PC TXD -> FPGA RX on K15
## FPGA TX -> PC RXD on L12
set_property -dict { PACKAGE_PIN K15 IOSTANDARD LVCMOS33 } [get_ports { uart_rx_i }]
set_property -dict { PACKAGE_PIN L12 IOSTANDARD LVCMOS33 } [get_ports { uart_tx_o }]

## Pushbutton used as reset (active-high)
set_property -dict { PACKAGE_PIN D2 IOSTANDARD LVCMOS33 } [get_ports { rst_btn }]

## User LEDs (active-high)
set_property -dict { PACKAGE_PIN E2 IOSTANDARD LVCMOS33 } [get_ports { led_o[0] }]
set_property -dict { PACKAGE_PIN K1 IOSTANDARD LVCMOS33 } [get_ports { led_o[1] }]
set_property -dict { PACKAGE_PIN J1 IOSTANDARD LVCMOS33 } [get_ports { led_o[2] }]
set_property -dict { PACKAGE_PIN E1 IOSTANDARD LVCMOS33 } [get_ports { led_o[3] }]

## RGB LED (active-low in logic, but still standard LVCMOS33 pins)
set_property -dict { PACKAGE_PIN F2 IOSTANDARD LVCMOS33 } [get_ports { rgb_r_o }]
set_property -dict { PACKAGE_PIN D3 IOSTANDARD LVCMOS33 } [get_ports { rgb_g_o }]
set_property -dict { PACKAGE_PIN F1 IOSTANDARD LVCMOS33 } [get_ports { rgb_b_o }]

## I2C
set_property -dict { PACKAGE_PIN H3 IOSTANDARD LVCMOS33 } [get_ports { i2c_scl_i }]
set_property -dict { PACKAGE_PIN H1 IOSTANDARD LVCMOS33 } [get_ports { i2c_sda_i }]

## SPI
set_property -dict { PACKAGE_PIN J2 IOSTANDARD LVCMOS33 } [get_ports { spi_sclk_i }]
set_property -dict { PACKAGE_PIN H4 IOSTANDARD LVCMOS33 } [get_ports { spi_cs_n_i }]
set_property -dict { PACKAGE_PIN H2 IOSTANDARD LVCMOS33 } [get_ports { spi_mosi_i }]
set_property -dict { PACKAGE_PIN F3 IOSTANDARD LVCMOS33 } [get_ports { spi_miso_i }]

## UART
set_property -dict { PACKAGE_PIN G1 IOSTANDARD LVCMOS33 } [get_ports { probe_uart_rx_i }]

## CAN 
set_property -dict { PACKAGE_PIN F4 IOSTANDARD LVCMOS33 } [get_ports { probe_can_rx_i }]

## timing cleanup
set_false_path -from [get_ports { rst_btn }]
set_false_path -to   [get_ports { led_o[0] led_o[1] led_o[2] led_o[3] rgb_r_o rgb_g_o rgb_b_o }]

# set_false_path -from [get_ports { i2c_scl_i i2c_sda_i spi_sclk_i spi_cs_n_i spi_mosi_i spi_miso_i }]
