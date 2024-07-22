//********************************************************************
// <File>     : gw_ahb_ethernet_top.v
// <Author>   : GOWIN EMB
// <Function> : AHB Ethernet top level
// <Version>  : 1.2
// <Date>     : 2024
//********************************************************************


// gw_ahb_ethernet_top
module gw_ahb_ethernet_top
(
    // Triple speed ethernet mac
    input rstn,
    input tx_mac_clk,
    output tx_mac_valid,
    output [7:0] tx_mac_data,
    output tx_mac_last,
    output tx_mac_error,
    input tx_mac_ready,
    input tx_collision,
    input tx_retransmit,
    output tx_pause_req,
    output [15:0] tx_pause_val,
    output [47:0] tx_pause_source_addr,   
    input tx_statistics_valid,
    input [28:0] tx_statistics_vector,
    input rx_mac_clk,
    input rx_mac_valid,
    input [7:0] rx_mac_data,
    input rx_mac_last,
    input rx_mac_error,
    input rx_statistics_valid,
    input [26:0] rx_statistics_vector,
    input rx_pause_req,
    input [15:0] rx_pause_val,
    input clk,
    output [4:0] miim_phyad,
    output [4:0] miim_regad,
    output [15:0] miim_wrdata,
    output miim_wren,
    output miim_rden,
    input [15:0] miim_rddata,
    input miim_rddata_valid,
    input miim_busy,
    
    // AHB
    input hclk,
    input hresetn,
    input [31:0] haddr,
    input [1:0] htrans,
    input hwrite,
    input [2:0] hsize,
    input [2:0] hburst,
    input [31:0] hwdata,
    input hsel,
    input hreadyin,
    output [31:0] hrdata,
    output [1:0] hresp,
    output hready,
    
    // Interrupt
    output eth_int,
    
    // Configuration
    output speedis1000,
    output speedis10,
    output duplex_status
);

// gw_ahb_ethernet instantiation
gw_ahb_ethernet u_gw_ahb_ethernet
(
    .rstn(rstn),
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
    .clk(clk),
    .miim_phyad(miim_phyad),
    .miim_regad(miim_regad),
    .miim_wrdata(miim_wrdata),
    .miim_wren(miim_wren),
    .miim_rden(miim_rden),
    .miim_rddata(miim_rddata),
    .miim_rddata_valid(miim_rddata_valid),
    .miim_busy(miim_busy),
    .hclk(hclk),
    .hresetn(hresetn),
    .haddr(haddr),
    .htrans(htrans),
    .hwrite(hwrite),
    .hsize(hsize),
    .hburst(hburst),
    .hwdata(hwdata),
    .hsel(hsel),
    .hreadyin(hreadyin),
    .hrdata(hrdata),
    .hresp(hresp),
    .hready(hready),
    .eth_int(eth_int),
    .speedis1000(speedis1000),
    .speedis10(speedis10),
    .duplex_status(duplex_status)
);

endmodule