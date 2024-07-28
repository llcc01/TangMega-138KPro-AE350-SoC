
// ae350_soc_top
module ae350_soc_top
(
    input CLK,
    input RSTN,
    // GPIO
    inout [2:0] LED,     // 2:0
    inout [2:0] KEY,     // 5:3
    // UART2
    output UART1_TXD,
    input  UART1_RXD,
    // UART2
    output UART2_TXD,
    input  UART2_RXD,
    // SPI Flash Memory
    inout FLASH_SPI_CSN,
    inout FLASH_SPI_MISO,
    inout FLASH_SPI_MOSI,
    inout FLASH_SPI_CLK,
    inout FLASH_SPI_HOLDN,
    inout FLASH_SPI_WPN,
    // DDR3 Memory
    output DDR3_INIT,
    output [2:0] DDR3_BANK,
    output DDR3_CS_N,
    output DDR3_RAS_N,
    output DDR3_CAS_N,
    output DDR3_WE_N,
    output DDR3_CK,
    output DDR3_CK_N,
    output DDR3_CKE,
    output DDR3_RESET_N,
    output DDR3_ODT,
    output [13:0] DDR3_ADDR,
    output [1:0] DDR3_DM,
    inout [15:0] DDR3_DQ,
    inout [1:0] DDR3_DQS,
    inout [1:0] DDR3_DQS_N,
    //SD
    inout SD_CLK,
    inout SD_CS,
    inout SD_CHECKOUT,
    inout SD_DATAOUT,
    inout SD_DATAIN,
    // Ethernet
    output PHY_CLK,
    output PHY_RST_N,
    output RGMII_TXC,
    output RGMII_TX_CTL,
    output [3:0] RGMII_TXD,
    input RGMII_RXC,
    input RGMII_RX_CTL,
    input [3:0] RGMII_RXD,
    output MDC,
    inout MDIO,
    // JTAG
    input TCK_IN,
    input TMS_IN,
    input TRST_IN,
    input TDI_IN,
    output TDO_OUT
);


wire CORE_CLK;
wire DDR_CLK;
wire AHB_CLK;
wire APB_CLK;
wire RTC_CLK;

wire DDR3_MEMORY_CLK;
wire DDR3_CLK_IN;
wire DDR3_LOCK;
wire DDR3_STOP;

wire GTX_CLK;
wire ETH_CLK;


// Gowin_PLL_AE350 instantiation
Gowin_PLL_AE350 u_Gowin_PLL_AE350
(
    .clkout0(DDR_CLK),          // 100MHz
    .clkout1(CORE_CLK),         // 800MHz
    .clkout2(AHB_CLK),          // 100MHz
    .clkout3(APB_CLK),          // 100MHz
    .clkout4(RTC_CLK),          // 10MHz
    .clkin(CLK),
    .enclk0(1'b1),
    .enclk1(1'b1),
    .enclk2(1'b1),
    .enclk3(1'b1),
    .enclk4(1'b1)
);

// Gowin_PLL instantiation
Gowin_PLL_DDR u_Gowin_PLL_DDR
(
    .lock(DDR3_LOCK),
    .clkout0(DDR3_CLK_IN),          // 50MHz
    .clkout2(DDR3_MEMORY_CLK),      // 250MHz
    .clkin(CLK),
    .reset(1'b0),                   // Enforce
    .enclk0(1'b1),
    .enclk2(DDR3_STOP)
);

// Gowin_PLL instantiation
Gowin_PLL u_Gowin_PLL
(
    .clkout1(GTX_CLK),              // 125MHz
    .clkout3(ETH_CLK),              // 25MHz
    .clkin(CLK),
    .reset(1'b0),                   // Enforce
    .enclk0(1'b1),
    .enclk1(1'b1),
    .enclk3(1'b1)
);


wire ae350_rstn;                    // AE350 power on and hardware reset in
wire ddr3_rstn;                     // DDR3 memory reset in
wire ddr3_init_completed;           // DDR3 memory initialized completed

assign DDR3_INIT = ~ddr3_init_completed;


// key_debounce instantiation
// DDR3 memory reset in key debounce
key_debounce u_key_debounce_ddr3
(
    .out(ddr3_rstn),
    .in(RSTN),
    .clk(CLK),      // 50MHz
    .rstn(1'b1)
);


// key_debounce instantiation
// AE350 power on and hardware reset in key debounce
key_debounce u_key_debounce_ae350
(
    .out(ae350_rstn),
    .in(ddr3_init_completed),
    .clk(CLK),      // 50MHz
    .rstn(1'b1)
);

// ethernet

wire [31:0] EXTS_HRDATA;
wire EXTS_HREADYIN;
wire EXTS_HRESP;
wire [31:0] EXTS_HADDR;
wire [2:0] EXTS_HBURST;
wire [3:0] EXTS_HPROT;
wire EXTS_HSEL;
wire [2:0] EXTS_HSIZE;
wire [1:0] EXTS_HTRANS;
wire [31:0] EXTS_HWDATA;
wire EXTS_HWRITE;
wire EXTS_HCLK;
wire EXTS_HRSTN;

wire eth_rstn;
wire miim_clk;
wire speedis1000;
wire speedis10;
wire duplex_status;
wire tx_mac_clk;
wire tx_mac_valid;
wire [7:0] tx_mac_data;
wire tx_mac_last;
wire tx_mac_error;
wire tx_mac_ready;
wire tx_collision;
wire tx_retransmit;
wire tx_pause_req;
wire [15:0] tx_pause_val;
wire [47:0] tx_pause_source_addr;
wire tx_statistics_valid;
wire [28:0] tx_statistics_vector;
wire intr_ethernet;

wire rx_mac_clk;
wire rx_mac_valid;
wire [7:0] rx_mac_data;
wire rx_mac_last;
wire rx_mac_error;
wire rx_statistics_valid;
wire [26:0] rx_statistics_vector;
wire rx_pause_req;
wire [15:0] rx_pause_val;

wire [4:0] miim_phyad;
wire [4:0] miim_regad;
wire [15:0] miim_wrdata;
wire miim_wren;
wire miim_rden;
wire [15:0] miim_rddata;
wire miim_rddata_valid;
wire miim_busy;
wire mdio_in;
wire mdio_out;
wire mdio_oen;

assign miim_clk = EXTS_HCLK;
assign eth_rstn = EXTS_HRSTN;


// gw_ahb_ethernet_top instantiation
gw_ahb_ethernet_top u_gw_ahb_ethernet_top
(
    .rstn(eth_rstn),
    .tx_mac_clk(tx_mac_clk),
    .tx_mac_valid(tx_mac_valid),
    .tx_mac_data(tx_mac_data),
    .tx_mac_last(tx_mac_last),
    .tx_mac_error(tx_mac_error),
    .tx_mac_ready(tx_mac_ready),
    .tx_collision(tx_collision),
    .tx_retransmit(tx_retransmit),
    .tx_pause_req(tx_pause_req),
    .tx_pause_val(tx_pause_val),
    .tx_pause_source_addr(tx_pause_source_addr),
    .tx_statistics_valid(tx_statistics_valid),
    .tx_statistics_vector(tx_statistics_vector),
    .rx_mac_clk(rx_mac_clk),
    .rx_mac_valid(rx_mac_valid),
    .rx_mac_data(rx_mac_data),
    .rx_mac_last(rx_mac_last),
    .rx_mac_error(rx_mac_error),
    .rx_statistics_valid(rx_statistics_valid),
    .rx_statistics_vector(rx_statistics_vector),
    .rx_pause_req(rx_pause_req),
    .rx_pause_val(rx_pause_val),
    .clk(miim_clk),
    .miim_phyad(miim_phyad),
    .miim_regad(miim_regad),
    .miim_wrdata(miim_wrdata),
    .miim_wren(miim_wren),
    .miim_rden(miim_rden),
    .miim_rddata(miim_rddata),
    .miim_rddata_valid(miim_rddata_valid),
    .miim_busy(miim_busy),
    .hclk(EXTS_HCLK),
    .hresetn(EXTS_HRSTN),
    .haddr(EXTS_HADDR),
    .htrans(EXTS_HTRANS),
    .hwrite(EXTS_HWRITE),
    .hsize(EXTS_HSIZE),
    .hburst(EXTS_HBURST),
    .hwdata(EXTS_HWDATA),
    .hsel(EXTS_HSEL),
    .hreadyin(EXTS_HREADYIN),
    .hrdata(EXTS_HRDATA),
    .hresp({GND, EXTS_HRESP}),
    .hready(EXTS_HREADYIN),
    .eth_int(intr_ethernet),
    .speedis1000(speedis1000),
    .speedis10(speedis10),
    .duplex_status(duplex_status)
);


// Triple_Speed_Ethernet_MAC_Top instantiation
Triple_Speed_Ethernet_MAC_Top u_Triple_Speed_Ethernet_MAC_Top
(
    .rgmii_rxc(RGMII_RXC),
    .rgmii_rx_ctl(RGMII_RX_CTL),
    .rgmii_rxd(RGMII_RXD),
    .gtx_clk(GTX_CLK),
    .rgmii_txc(RGMII_TXC),
    .rgmii_tx_ctl(RGMII_TX_CTL),
    .rgmii_txd(RGMII_TXD),
    .speedis1000(speedis1000),
    .speedis10(speedis10),
    .duplex_status(duplex_status),
    .rstn(eth_rstn),
    .rx_mac_clk(rx_mac_clk),
    .rx_mac_valid(rx_mac_valid),
    .rx_mac_data(rx_mac_data),
    .rx_mac_last(rx_mac_last),
    .rx_mac_error(rx_mac_error),
    .rx_statistics_valid(rx_statistics_valid),
    .rx_statistics_vector(rx_statistics_vector),
    .rx_pause_req(rx_pause_req),
    .rx_pause_val(rx_pause_val),
    .tx_mac_clk(tx_mac_clk),
    .tx_mac_valid(tx_mac_valid),
    .tx_mac_data(tx_mac_data),
    .tx_mac_last(tx_mac_last),
    .tx_mac_error(tx_mac_error),
    .tx_mac_ready(tx_mac_ready),
    .tx_collision(tx_collision),
    .tx_retransmit(tx_retransmit),
    .tx_pause_req(tx_pause_req),
    .tx_pause_val(tx_pause_val),
    .tx_pause_source_addr(tx_pause_source_addr),
    .tx_statistics_valid(tx_statistics_valid),
    .tx_statistics_vector(tx_statistics_vector),
    .rx_fcs_fwd_ena(1'b0),
    .rx_jumbo_ena(1'b0),
    .tx_fcs_fwd_ena(1'b0),
    .tx_ifg_delay_ena(1'b0),
    .tx_ifg_delay(8'b0),
    .clk(miim_clk),
    .miim_phyad(miim_phyad),
    .miim_regad(miim_regad),
    .miim_wrdata(miim_wrdata),
    .miim_wren(miim_wren),
    .miim_rden(miim_rden),
    .miim_rddata(miim_rddata),
    .miim_rddata_valid(miim_rddata_valid),
    .miim_busy(miim_busy),
    .mdc(MDC),
    .mdio_in(mdio_in),
    .mdio_out(mdio_out),
    .mdio_oen(mdio_oen)
);

assign mdio_in = MDIO;
assign MDIO = (!mdio_oen) ? mdio_out : 1'bz;

assign PHY_CLK = ETH_CLK;
assign PHY_RST_N = RSTN;

wire [31:0] gpio_pin;

assign gpio_pin[0] = LED[0];
assign gpio_pin[1] = LED[1];
assign gpio_pin[2] = LED[2];
assign gpio_pin[3] = KEY[0];
assign gpio_pin[4] = KEY[1];
assign gpio_pin[5] = KEY[2];
assign gpio_pin[20] = SD_CS;


// RiscV_AE350_SOC_Top instantiation
RiscV_AE350_SOC_Top u_RiscV_AE350_SOC_Top
(
    .FLASH_SPI_CSN(FLASH_SPI_CSN),
    .FLASH_SPI_MISO(FLASH_SPI_MISO),
    .FLASH_SPI_MOSI(FLASH_SPI_MOSI),
    .FLASH_SPI_CLK(FLASH_SPI_CLK),
    .FLASH_SPI_HOLDN(FLASH_SPI_HOLDN),
    .FLASH_SPI_WPN(FLASH_SPI_WPN),

    .DDR3_MEMORY_CLK(DDR3_MEMORY_CLK),
    .DDR3_CLK_IN(DDR3_CLK_IN),
    .DDR3_RSTN(ddr3_rstn),              // DDR3 memory reset in, 0 is reset state
    .DDR3_LOCK(DDR3_LOCK),
    .DDR3_STOP(DDR3_STOP),
    .DDR3_INIT(ddr3_init_completed),    // DDR3 memory initialized completed
    .DDR3_BANK(DDR3_BANK),
    .DDR3_CS_N(DDR3_CS_N),
    .DDR3_RAS_N(DDR3_RAS_N),
    .DDR3_CAS_N(DDR3_CAS_N),
    .DDR3_WE_N(DDR3_WE_N),
    .DDR3_CK(DDR3_CK),
    .DDR3_CK_N(DDR3_CK_N),
    .DDR3_CKE(DDR3_CKE),
    .DDR3_RESET_N(DDR3_RESET_N),
    .DDR3_ODT(DDR3_ODT),
    .DDR3_ADDR(DDR3_ADDR),
    .DDR3_DM(DDR3_DM),
    .DDR3_DQ(DDR3_DQ),
    .DDR3_DQS(DDR3_DQS),
    .DDR3_DQS_N(DDR3_DQS_N),

    .TCK_IN(TCK_IN),
    .TMS_IN(TMS_IN),
    .TRST_IN(TRST_IN),
    .TDI_IN(TDI_IN),
    .TDO_OUT(TDO_OUT),
    .TDO_OE(),

    .EXT_INT(intr_ethernet),
    .EXTS_HRDATA(EXTS_HRDATA),
    .EXTS_HREADYIN(EXTS_HREADYIN),
    .EXTS_HRESP(EXTS_HRESP),
    .EXTS_HADDR(EXTS_HADDR),
    .EXTS_HBURST(EXTS_HBURST),
    .EXTS_HPROT(EXTS_HPROT),
    .EXTS_HSEL(EXTS_HSEL),
    .EXTS_HSIZE(EXTS_HSIZE),
    .EXTS_HTRANS(EXTS_HTRANS),
    .EXTS_HWDATA(EXTS_HWDATA),
    .EXTS_HWRITE(EXTS_HWRITE),
    .EXTS_HCLK(EXTS_HCLK),
    .EXTS_HRSTN(EXTS_HRSTN),

    .UART1_TXD(UART1_TXD),
    .UART1_RTSN(),
    .UART1_RXD(UART1_RXD),
    .UART1_CTSN(),
    .UART1_DSRN(),
    .UART1_DCDN(),
    .UART1_RIN(),
    .UART1_DTRN(),
    .UART1_OUT1N(),
    .UART1_OUT2N(),

    .UART2_TXD(UART2_TXD),
    .UART2_RTSN(),
    .UART2_RXD(UART2_RXD),
    .UART2_CTSN(),
    .UART2_DCDN(),
    .UART2_DSRN(),
    .UART2_RIN(),
    .UART2_DTRN(),
    .UART2_OUT1N(),
    .UART2_OUT2N(),

    .SPI_HOLDN(), //inout SPI_HOLDN
    .SPI_WPN(), //inout SPI_WPN
    .SPI_CLK(SD_CLK), //inout SPI_CLK
    .SPI_CSN(), //inout SPI_CSN
    .SPI_MISO(SD_DATAOUT), //inout SPI_MISO
    .SPI_MOSI(SD_DATAIN), //inout SPI_MOSI

    .GPIO(gpio_pin),

    .CORE_CLK(CORE_CLK),
    .DDR_CLK(DDR_CLK),
    .AHB_CLK(AHB_CLK),
    .APB_CLK(APB_CLK),
    .RTC_CLK(RTC_CLK),
    .POR_RSTN(ae350_rstn),              // AE350 CPU core power on reset, 0 is reset state
    .HW_RSTN(ae350_rstn)                // AE350 hardware reset, 0 is reset state
);

endmodule
