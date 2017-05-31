//--------------------------------------------------------------------------//
// Title:       golden_top.v                                                //
// Rev:         Rev B                                                       //
//--------------------------------------------------------------------------//
// Description: All Arria V GX FPGA Dev Kit I/O signals and settings      //
//              such as termination, drive strength, etc...                 //
//              Some toggle_rate=0 where needed for fitter rules. (TR=0)    // 
//--------------------------------------------------------------------------//
// Revision History:                                                        //
// Rev B:       final fpga 1 of 2
//----------------------------------------------------------------------------
//------ 1 ------- 2 ------- 3 ------- 4 ------- 5 ------- 6 ------- 7 ------7
//------ 0 ------- 0 ------- 0 ------- 0 ------- 0 ------- 0 ------- 0 ------8
//----------------------------------------------------------------------------
//Copyright 2012 Altera Corporation. All rights reserved.  Altera products  
//are protected under numerous U.S. and foreign patents, maskwork rights,     
//copyrights and other intellectual property laws.                            
//                                                                            
//This reference design file, and your use thereof, is subject to and         
//governed by the terms and conditions of the applicable Altera Reference     
//Design License Agreement.  By using this reference design file, you         
//indicate your acceptance of such terms and conditions between you and       
//Altera Corporation.  In the event that you do not agree with such terms and 
//conditions, you may not use the reference design file. Please promptly      
//destroy any copies you have made.                                           
//                                                                            
//This reference design file being provided on an "as-is" basis and as an     
//accommodation and therefore all warranties, representations or guarantees   
//of any kind (whether express, implied or statutory) including, without      
//limitation, warranties of merchantability, non-infringement, or fitness for 
//a particular purpose, are specifically disclaimed.  By making this          
//reference design file available, Altera expressly does not recommend,       
//suggest or require that this reference design file be used in combination   
//with any other product not provided by Altera.                              
//


//`define DDR3
//`define FM
//`define ETHERNET
//`define HSMA
`define USER
`define SFP
`define C2C


module a5gx_golden_top 
(

//GPLL-CLK-----------------------------//8 pins
   input          clkina_50,            //1.8V    //50 MHz, also to EPM2210F324 and second fpga via a buffer.
   input   [1:0]  clkintopa_p,      //LVDS    // clkintopa_p[0] = 100 MHz prog osc, clkintopa_p[1] = 125 MHz prog osc, External Term.
   input   [1:0]  clkinbota_p,      //LVDS    //100 MHz prog osc clkinbota_p[0], clkinbota_p[1] = 625MHz, External Term.
   input  		   clka_125_p,           //LVDS    //125 MHz GPLL-req's External Term.
//XCVR-REFCLK--------------------------//16 pins //req's ALTGXB instatiation
//	input				refclk0_a_qr0_p,	//Default 100MHz (GX)
//	input				refclk2_a_qr1_p,	//Default 625MHz  (GX)
//	input				refclk4_a_qr2_p,	//Default 125MHz (GT)
//	input				refclk1_a_ql0_p,	//Default 100MHz  (GX)
//	input				refclk2_a_ql1_p,	//Default 156.25MHz (GX)
//	input				refclk3_a_ql1_p,	//Default 125MHz (GX)
	input				refclk4_a_ql2_p,	//Default 125MHz  (GT)

`ifdef DDR3
//DDR3 Devices-x72--------------------------//125 pins //--------------------------
   output  [13:0] ddr3a_a,           //SSTL15  //Address
   output  [2:0]  ddr3a_ba,          //SSTL15  //Bank Address
   output         ddr3a_casn,        //SSTL15  //Column Address Strobe
   output         ddr3a_clk_n,        //SSTL15  //Diff Clock - Neg
   output         ddr3a_clk_p,        //SSTL15  //Diff Clock - Pos
   output         ddr3a_cke,         //SSTL15  //Clock Enable
   output         ddr3a_csn,         //SSTL15  //Chip Select
   output  [8:0]  ddr3a_dm,          //SSTL15  //Data Write Mask
   inout   [71:0] ddr3a_dq,          //SSTL15  //Data Bus
   inout   [8:0]  ddr3a_dqs_n,       //SSTL15  //Diff Data Strobe - Neg
   inout   [8:0]  ddr3a_dqs_p,       //SSTL15  //Diff Data Strobe - Pos
   output         ddr3a_odt,         //SSTL15  //On-Die Termination Enable
   output         ddr3a_rasn,        //SSTL15  //Row Address Strobe
   output         ddr3a_resetn,        //SSTL15  //Reset
   output         ddr3a_wen,         //SSTL15  //Write Enable
  input	  rzqin_1_5v,			//OCT Pin in Bank TBD

`endif

`ifdef FM
//FM-Shared-Bus---(Flash/Max)----//57 pins //--------------------------
   output  [26:0] fm_a,               //1.8V    //Address
   inout   [15:0] fm_d,               //1.8V    //Data
   output         flash_advn,          //1.8V    //Flash Address Valid
   output         flash_cen,           //1.8V    //Flash Chip Enable
   output         flash_clk,           //1.8V    //Flash Clock
   output         flash_oen,           //1.8V    //Flash Output Enable
   input          flash_rdybsyn,       //1.8V    //Flash Ready/Busy
   output         flash_resetn,        //1.8V    //Flash Reset
   output         flash_wen,           //1.8V    //Flash Write Enable

   output   [3:0] max_ben,            //1.5V    //Max II Byte Enable Per Byte
   inout          max_clk,            //1.5V    //Max II Clk
   output         max_csn,            //1.5V    //Max II Chip Select
   output         max_oen,            //1.5V    //Max II Output Enable
   output         max_wen,            //1.5V    //Max II Write Enable
   
   inout   [2:0]  max_ctl,            //2.5V    //Max II 
   inout         flash_accessn,      //2.5V    //Flash Access
	
`endif

`ifdef ETHERNET	
//Ethernet-10/100/1000---RGMII-----------------//16 pins  //--------------------------
   input          enet_intn,           //2.5V    //MDIO Interrupt (TR=0)
   output         enet_mdc,            //2.5V    //MDIO Clock (TR=0)
   inout          enet_mdio,           //2.5V    //MDIO Data (TR=0)
   output         enet_resetn,         //2.5V    //Device Reset (TR=0)
   input          enet_rx_p,           //LVDS NEED EXTERNAL TERM //RGMII Receive-req's OCT
   output         enet_tx_p,           //LVDS    //RGMII Transmit
   output	  enet_gtx_clk,	       //2.5V
   input	     enet_rx_clk,	       //2.5V
   output         enet_tx_en, 	       //2.5V
   output[3:0]	  enet_tx_d,           //2.5V
   input	  enet_rx_dv,          //2.5V
   input [3:0]	  enet_rx_d,           //2.5V
   inout	  enet_led_link1000,   //2.5V

`endif

`ifdef HSMA
//HSMC-Port-A--------------------------//107pins //--------------------------
//   input  [7:0] hsma_rx_p,           //PCML14  //HSMA Receive Data-req's OCT
//   output [7:0] hsma_tx_p,           //PCML14  //HSMA Transmit Data
 //Enable below for CMOS HSMC        
   //inout  [79:0]  hsma_d,            //2.5V    //HSMA CMOS Data Bus
 //Enable below for LVDS HSMC        
   input          hsma_clk_in0,        //2.5V    //Primary single-ended CLKIN
   input          hsma_clk_in_p1,      //LVDS    //Secondary diff. CLKIN
   input          hsma_clk_in_p2,      //LVDS    //Primary Source-Sync CLKIN
   output         hsma_clk_out0,       //2.5V    //Primary single-ended CLKOUT
   output         hsma_clk_out_p1,     //LVDS    //Secondary diff. CLKOUT
   output         hsma_clk_out_p2,     //LVDS    //Primary Source-Sync CLKOUT
   inout    [3:0] hsma_d,              //2.5V    //Dedicated CMOS IO
   input          hsma_prsntn,         //2.5V    //HSMC Presence Detect Input
   input   [16:0] hsma_rx_d_p,         //LVDS    //LVDS Sounce-Sync Input
   output  [16:0] hsma_tx_d_p,         //LVDS    //LVDS Sounce-Sync Output
   output         hsma_rx_led,         //2.5V    //User LED - Labeled RX
   output         hsma_scl,            //2.5V    //SMBus Clock
   inout          hsma_sda,            //2.5V    //SMBus Data
   output         hsma_tx_led,         //2.5V    //User LED - Labeled TX
	
`endif

`ifdef USER
//User-IO------------------------------//28 pins //--------------------------
   input    [7:0] user1_dipsw,          //2.5V //User DIP Switches (TR=0)
   output   [7:0] user1_led_g,          //2.5V	//Green User LEDs
   output   [7:0] user1_led_r,          //2.5V/1.8V    //Red User LEDs
   input    [2:0] user1_pb,             //2.5V   //User Pushbuttons (TR=0)
   input   	  cpu1_resetn,          //2.5V   //CPU Reset Pushbutton (TR=0)

`endif

`ifdef SFP
//SFP+------------------------------//16 pins //--------------------------

inout	[2:1]	sfp_scl,
inout	[2:1]	sfp_sda,
output[2:1]	sfp_tx_dis,
output[2:1]	sfp_tx_rs0,
output[2:1]	sfp_tx_rs1,
input	[2:1]	sfp_op_rx_los,
input	[2:1]	sfp_op_tx_flt,
input	[2:1]	sfp_mod_abs,
input       sfp_rx_p1,
output      sfp_tx_p1,

`endif


`ifdef C2C
//Chip-to-chip -----------------------//120 pins  //--------------------------
	input		[28:0]	c2c_din_p, 	         //2.5V
	output	[28:0]	c2c_dout_p, 	      //2.5V
	output				c2c_fpga2_clkin_p, 	//LVDS
	input					c2c_fpga1_clkin_p 	//LVDS

`endif

);
	parameter FREQ_50MHZ =  50*1000*1000;
	parameter UNIT_1MHZ  = 0.1*1000*1000;
	
	// reset
	reg [9:0] r_asbuf_rst_cnt;
	reg [2:0] r_asbuf_reset_n;
	reg       reset_n;
	
	always @(posedge clkina_50) begin
		if(r_asbuf_rst_cnt==1000)begin
			r_asbuf_rst_cnt <= 0;
			r_asbuf_reset_n <= {r_asbuf_reset_n[1:0],cpu1_resetn};
			reset_n         <= (r_asbuf_reset_n[2]==r_asbuf_reset_n[1]) ? r_asbuf_reset_n[2] : reset_n;
		end else begin
			r_asbuf_rst_cnt <= r_asbuf_rst_cnt+1'b1;
			r_asbuf_reset_n <= r_asbuf_reset_n;
			reset_n         <= reset_n;
		end
	end
			
	// SFP+ TX
	wire        xgmii_tx_clk; // 156.26MHz
	wire        w_tx_ready;
	wire [71:0] w_xgmii_tx_dc_0;
	wire [7:0]  w_tx0_data;
	wire [7:0]  w_tx1_data;
	wire [7:0]  w_tx2_data;
	wire [7:0]  w_tx3_data;
	wire [7:0]  w_tx4_data;
	wire [7:0]  w_tx5_data;
	wire [7:0]  w_tx6_data;
	wire [7:0]  w_tx7_data;
	
	reg[4:0] r_counter_tx;
	
	assign xgmii_tx_clk = xgmii_rx_clk; // must
	
	reg [31:0] r_clkcnt1;
	wire[15:0] freq_xgmii_tx_clk;
	always @(posedge xgmii_tx_clk) begin
		if(r_clkcnt1==32'd0)begin
			r_clkcnt1 <= 125*1000*1000-1;
		end else begin
			r_clkcnt1 <= r_clkcnt1-1'b1;
		end
	end
	assign user1_led_r[1] = (r_clkcnt1[31:16]==32'd0) ? 1'b0 : 1'b1;
	clkmon #(.P_CLK_FREQ(FREQ_50MHZ), .P_MON_UNIT(UNIT_1MHZ)) u_clkmon_xgmii_tx (
		.CLK     (clkina_50),
		.RESET   (~reset_n ),
		.MON_CLK (xgmii_tx_clk      ),
		.MON_FREQ(freq_xgmii_tx_clk )
	);
			
	// Tx Data
	reg [7:0]  w_tx_en;
	reg [63:0] w_tx_data;
	always @(posedge xgmii_tx_clk) begin
		if(w_tx_ready) begin
			r_counter_tx <= r_counter_tx + 1'b1;
		end else begin
			r_counter_tx <= r_counter_tx;
		end
	end	
	
	always @(*)begin
		case (r_counter_tx)
		0 : begin w_tx_en <= 8'hFF; w_tx_data <= 64'h07070707_07070707; end
		1 : begin w_tx_en <= 8'h1F; w_tx_data <= 64'h555555FB_07070707; end
		2 : begin w_tx_en <= 8'h00; w_tx_data <= 64'h00000000_d5555555; end
		3 : begin w_tx_en <= 8'h00; w_tx_data <= 64'h00000001_00000000; end
		4 : begin w_tx_en <= 8'h00; w_tx_data <= 64'h00000002_00000000; end
		5 : begin w_tx_en <= 8'h00; w_tx_data <= 64'h00000003_00000000; end
		6 : begin w_tx_en <= 8'h00; w_tx_data <= 64'h00000004_00000000; end
		7 : begin w_tx_en <= 8'h00; w_tx_data <= 64'h00000005_00000000; end
		8 : begin w_tx_en <= 8'h00; w_tx_data <= 64'h00000006_00000000; end
		9 : begin w_tx_en <= 8'h00; w_tx_data <= 64'h00000007_00000000; end
		10: begin w_tx_en <= 8'h00; w_tx_data <= 64'h3dc288af_00000000; end
		11: begin w_tx_en <= 8'hFF; w_tx_data <= 64'h07070707_070707FD; end
		default: 
			 begin w_tx_en <= 8'hFF; w_tx_data <= 64'h07070707_07070707; end
		endcase
	end
	
	assign w_tx0_data = w_tx_data[ 7: 0];
	assign w_tx1_data = w_tx_data[15: 8];
	assign w_tx2_data = w_tx_data[23:16];
	assign w_tx3_data = w_tx_data[31:24];
	assign w_tx4_data = w_tx_data[39:32];
	assign w_tx5_data = w_tx_data[47:40];
	assign w_tx6_data = w_tx_data[55:48];
	assign w_tx7_data = w_tx_data[63:56];
	
	
	assign w_xgmii_tx_dc_0 = {
		w_tx_en[7],w_tx7_data,
		w_tx_en[6],w_tx6_data,
		w_tx_en[5],w_tx5_data,
		w_tx_en[4],w_tx4_data,
		w_tx_en[3],w_tx3_data,
		w_tx_en[2],w_tx2_data,
		w_tx_en[1],w_tx1_data,
		w_tx_en[0],w_tx0_data
	};
	
	// SFP+ RX
	wire        xgmii_rx_clk; // 156.25MHz
	wire        w_rx_ready;
	wire        w_rx_data_ready;
	wire [71:0] w_xgmii_rx_dc_0;
	wire [7:0]  w_rx_en;
	wire [7:0]  w_rx0_data;
	wire [7:0]  w_rx1_data;
	wire [7:0]  w_rx2_data;
	wire [7:0]  w_rx3_data;
	wire [7:0]  w_rx4_data;
	wire [7:0]  w_rx5_data;
	wire [7:0]  w_rx6_data;
	wire [7:0]  w_rx7_data;
		
	assign w_rx7_data = w_xgmii_rx_dc_0[70:63];
	assign w_rx6_data = w_xgmii_rx_dc_0[61:54];
	assign w_rx5_data = w_xgmii_rx_dc_0[52:45];
	assign w_rx4_data = w_xgmii_rx_dc_0[43:36];
	assign w_rx3_data = w_xgmii_rx_dc_0[34:27];
	assign w_rx2_data = w_xgmii_rx_dc_0[25:18];
	assign w_rx1_data = w_xgmii_rx_dc_0[16:9];
	assign w_rx0_data = w_xgmii_rx_dc_0[7:0];
	
	
	assign w_rx_en[7] = w_xgmii_rx_dc_0[71];
	assign w_rx_en[6] = w_xgmii_rx_dc_0[62];
	assign w_rx_en[5] = w_xgmii_rx_dc_0[53];
	assign w_rx_en[4] = w_xgmii_rx_dc_0[44];
	assign w_rx_en[3] = w_xgmii_rx_dc_0[35];
	assign w_rx_en[2] = w_xgmii_rx_dc_0[26];
	assign w_rx_en[1] = w_xgmii_rx_dc_0[17];
	assign w_rx_en[0] = w_xgmii_rx_dc_0[8];	
		
	reg [31:0] r_clkcnt2;
	wire[15:0] freq_xgmii_rx_clk;
	always @(posedge xgmii_rx_clk) begin
		if(r_clkcnt2==32'd0)begin
			r_clkcnt2 <= 156.25*1000*1000-1;
		end else begin
			r_clkcnt2 <= r_clkcnt2-1'b1;
		end
	end
	assign user1_led_r[2] = (r_clkcnt2[31:16]==32'd0) ? 1'b0 : 1'b1;
	clkmon #(.P_CLK_FREQ(FREQ_50MHZ), .P_MON_UNIT(UNIT_1MHZ)) u_clkmon_xgmii_rx (
		.CLK     (clkina_50),
		.RESET   (~reset_n ),
		.MON_CLK (xgmii_rx_clk      ),
		.MON_FREQ(freq_xgmii_rx_clk )
	);
	
	// SFP+ Status
	wire        w_pll_locked;
	wire        pll_ref_clk;             // 644.53125MHz
	wire        rx_recovered_clk;        //
	
	assign pll_ref_clk = refclk4_a_ql2_p;
   assign user1_led_r[0] = w_pll_locked;

	reg [31:0] r_clkcnt3;
	wire[15:0] freq_rx_recovered_clk;
	always @(posedge rx_recovered_clk) begin
		if(r_clkcnt3==32'd0)begin
			r_clkcnt3 <= 156.25*1000*1000-1;
		end else begin
			r_clkcnt3 <= r_clkcnt3-1'b1;
		end
	end
	assign user1_led_r[3] = (r_clkcnt3[31:16]==32'd0) ? 1'b0 : 1'b1;
	clkmon #(.P_CLK_FREQ(FREQ_50MHZ), .P_MON_UNIT(UNIT_1MHZ)) u_clkmon_rx_recovered (
		.CLK     (clkina_50),
		.RESET   (~reset_n ),
		.MON_CLK (rx_recovered_clk      ),
		.MON_FREQ(freq_rx_recovered_clk )
	);
	
	// Avalon-MM
	wire        phy_mgmt_clk;            // 37.5-50MHz
	wire        phy_mgmt_clk_reset;
	wire [8:0]  w_phy_mgmt_address;    
	wire        w_phy_mgmt_read;
	wire [31:0] w_phy_mgmt_readdata;
	wire        w_phy_mgmt_write;
	wire [31:0] w_phy_mgmt_writedata;
	wire        w_phy_mgmt_waitrequest;
	
	assign phy_mgmt_clk         = clkina_50; // 50MHz
	assign phy_mgmt_clk_reset   = ~reset_n;
	//assign w_phy_mgmt_address   = 9'd0;
	//assign w_phy_mgmt_read      = 1'b0;
	//assign w_phy_mgmt_write     = 1'b0;
	//assign w_phy_mgmt_writedata = 32'h0000_0000;

	reg [31:0] r_clkcnt4;
	wire[15:0] freq_phy_mgmt_clk;
	always @(posedge phy_mgmt_clk) begin
		if(r_clkcnt4==32'd0)begin
			r_clkcnt4 <= 50*1000*1000-1;
		end else begin
			r_clkcnt4 <= r_clkcnt4-1'b1;
		end
	end
	assign user1_led_r[4] = (r_clkcnt4[31:16]==32'd0) ? 1'b0 : 1'b1;
	clkmon #(.P_CLK_FREQ(FREQ_50MHZ), .P_MON_UNIT(UNIT_1MHZ)) u_clkmon_phy_mgmt (
		.CLK     (clkina_50),
		.RESET   (~reset_n ),
		.MON_CLK (phy_mgmt_clk      ),
		.MON_FREQ(freq_phy_mgmt_clk )
	);
	
	// Transceiver Reconfiguration Controller
	wire        mgmt_clk_clk;
	wire        mgmt_rst_reset;
	wire [6:0]  w_reconfig_mgmt_address;
	wire        w_reconfig_mgmt_read;
	wire [31:0] w_reconfig_mgmt_readdata;
	wire        w_reconfig_mgmt_waitrequest;
	wire        w_reconfig_mgmt_write;
	wire [31:0] w_reconfig_mgmt_writedata;

	assign mgmt_clk_clk              = clkina_50;
	assign mgmt_rst_reset            = ~reset_n;
	//assign w_reconfig_mgmt_address   = 7'd0;
	//assign w_reconfig_mgmt_read      = 1'b0;
	//assign w_reconfig_mgmt_write     = 1'b0;
	//assign w_reconfig_mgmt_writedata = 32'h0000_0000;
	
	reg [31:0] r_clkcnt5;
	wire[15:0] freq_mgmt_clk_clk;
	always @(posedge mgmt_clk_clk) begin
		if(r_clkcnt5==32'd0)begin
			r_clkcnt5 <= 50*1000*1000-1;
		end else begin
			r_clkcnt5 <= r_clkcnt5-1'b1;
		end
	end
	assign user1_led_r[5] = (r_clkcnt5[31:16]==32'd0) ? 1'b0 : 1'b1;
	clkmon #(.P_CLK_FREQ(FREQ_50MHZ), .P_MON_UNIT(UNIT_1MHZ)) u_clkmon_mgmt_clk (
		.CLK     (clkina_50),
		.RESET   (~reset_n ),
		.MON_CLK (mgmt_clk_clk      ),
		.MON_FREQ(freq_mgmt_clk_clk )
	);
	
	// Reconfiguration I/F
	wire [91:0]  w_reconfig_from_xcvr;
	wire [139:0] w_reconfig_to_xcvr;
		
	// SFP+ PHY 
   SFP1 u_sfp1 (
		// Transceiver Serial Data
		.rx_serial_data_0     (sfp_rx_p1             ), // input  wire           rx_serial_data_0.export // PCML
		.tx_serial_data_0     (sfp_tx_p1             ), // output wire [0:0]     tx_serial_data_0.export // PCML

		// Avalon-ST SDR XGMII-TX
		.xgmii_tx_clk         (xgmii_tx_clk          ), // input  wire               xgmii_tx_clk.clk
		.tx_ready             (w_tx_ready            ), // output wire                   tx_ready.export
		.xgmii_tx_dc_0        (w_xgmii_tx_dc_0       ), // input  wire [71:0]       xgmii_tx_dc_0.data

		// Avalon-ST SDR XGMII-RX
		.xgmii_rx_clk         (xgmii_rx_clk          ), // output wire               xgmii_rx_clk.clk
		.rx_ready             (w_rx_ready            ), // output wire                   rx_ready.export
		.rx_data_ready        (w_rx_data_ready       ), // output wire [0:0]        rx_data_ready.export
		.xgmii_rx_dc_0        (w_xgmii_rx_dc_0       ), // output wire [71:0]       xgmii_rx_dc_0.data

		// Reference Clock
		.pll_ref_clk          (pll_ref_clk           ), // input  wire                pll_ref_clk.clk
		.rx_recovered_clk     (rx_recovered_clk      ), // output wire [0:0]     rx_recovered_clk.clk

		// Avalon-MM PHY Management Interface
		.phy_mgmt_clk         (phy_mgmt_clk          ), // input  wire               phy_mgmt_clk.clk
		.phy_mgmt_clk_reset   (phy_mgmt_clk_reset    ), // input  wire         phy_mgmt_clk_reset.reset
		.phy_mgmt_address     (w_phy_mgmt_address    ), // input  wire [8:0]             phy_mgmt.address
		.phy_mgmt_read        (w_phy_mgmt_read       ), // input  wire                           .read
		.phy_mgmt_readdata    (w_phy_mgmt_readdata   ), // output wire [31:0]                    .readdata
		.phy_mgmt_write       (w_phy_mgmt_write      ), // input  wire                           .write
		.phy_mgmt_writedata   (w_phy_mgmt_writedata  ), // input  wire [31:0]                    .writedata
		.phy_mgmt_waitrequest (w_phy_mgmt_waitrequest), // output wire                           .waitrequest

		// Dynamic Reconfiguration
		.pll_locked           (w_pll_locked          ), // output wire                 pll_locked.export
		.reconfig_from_xcvr   (w_reconfig_from_xcvr  ), // output wire [91:0]  reconfig_from_xcvr.reconfig_from_xcvr
		.reconfig_to_xcvr     (w_reconfig_to_xcvr    )  // input  wire [139:0]   reconfig_to_xcvr.reconfig_to_xcvr
	);	
	
	// Transceiver Reconfiguration Controller
	TR_RECONF_CTRL u_tr_reconf_ctrl (
		.reconfig_busy             (reconfig_busy               ), // output wire              reconfig_busy.reconfig_busy
		.mgmt_clk_clk              (mgmt_clk_clk                ), // input  wire               mgmt_clk_clk.clk
		.mgmt_rst_reset            (mgmt_rst_reset              ), // input  wire             mgmt_rst_reset.reset
		.reconfig_mgmt_address     (w_reconfig_mgmt_address     ), // input  wire [6:0]        reconfig_mgmt.address
		.reconfig_mgmt_read        (w_reconfig_mgmt_read        ), // input  wire                           .read
		.reconfig_mgmt_readdata    (w_reconfig_mgmt_readdata    ), // output wire [31:0]                    .readdata
		.reconfig_mgmt_waitrequest (w_reconfig_mgmt_waitrequest ), // output wire                           .waitrequest
		.reconfig_mgmt_write       (w_reconfig_mgmt_write       ), // input  wire                           .write
		.reconfig_mgmt_writedata   (w_reconfig_mgmt_writedata   ), // input  wire [31:0]                    .writedata
		.reconfig_to_xcvr          (w_reconfig_to_xcvr          ), // output wire [139:0]   reconfig_to_xcvr.reconfig_to_xcvr
		.reconfig_from_xcvr        (w_reconfig_from_xcvr        ), // input  wire [91:0]  reconfig_from_xcvr.reconfig_from_xcvr
	);

	wire[7:0] w_led_export;
	
	systop u0 (
		.clk50m_clk                (clkina_50),                   //        clk50m.clk
		.reset_reset_n             (reset_n),                     //         reset.reset_n
		.led_export                (w_led_export),                //           led.export
		.reconfig_mgmt_address     (w_reconfig_mgmt_address),     // reconfig_mgmt.address
		.reconfig_mgmt_write       (w_reconfig_mgmt_write),       //              .write
		.reconfig_mgmt_read        (w_reconfig_mgmt_read),        //              .read
		.reconfig_mgmt_readdata    (w_reconfig_mgmt_readdata),    //              .readdata
		.reconfig_mgmt_writedata   (w_reconfig_mgmt_writedata),   //              .writedata
		.reconfig_mgmt_waitrequest (w_reconfig_mgmt_waitrequest), //              .waitrequest
		.phy_mgmt_address          (w_phy_mgmt_address),          //      phy_mgmt.address
		.phy_mgmt_write            (w_phy_mgmt_write),            //              .write
		.phy_mgmt_read             (w_phy_mgmt_read),             //              .read
		.phy_mgmt_readdata         (w_phy_mgmt_readdata),         //              .readdata
		.phy_mgmt_writedata        (w_phy_mgmt_writedata),        //              .writedata
		.phy_mgmt_waitrequest      (w_phy_mgmt_waitrequest)       //              .waitrequest
	);
	
	// LED
   assign user1_led_g[5:0] = w_led_export[5:0];
	
	// 刺さってれば緑
	assign user1_led_r[6]  = ~sfp_mod_abs[1];
	assign user1_led_r[7]  = ~sfp_mod_abs[2];
	assign user1_led_g[6]  =  sfp_mod_abs[1];
	assign user1_led_g[7]  =  sfp_mod_abs[2];  
	
	// SFP+モジュール設定
	assign sfp_tx_dis[1]   = 1'b0; // SFP+ J10
	assign sfp_tx_dis[2]   = 1'b0; // SFP+ J15
	assign sfp_tx_rs0[2:1] = 2'bzz; // SFP+ Rx rate (3.3V PullUp)
	assign sfp_tx_rs1[2:1] = 2'bzz; // SFP+ Tx rate (3.3V PullUp)
	
endmodule
