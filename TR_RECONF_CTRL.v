// megafunction wizard: %Transceiver Reconfiguration Controller v16.1%
// GENERATION: XML
// TR_RECONF_CTRL.v

// Generated using ACDS version 16.1 196

`timescale 1 ps / 1 ps
module TR_RECONF_CTRL (
		output wire         reconfig_busy,             //      reconfig_busy.reconfig_busy
		input  wire         mgmt_clk_clk,              //       mgmt_clk_clk.clk
		input  wire         mgmt_rst_reset,            //     mgmt_rst_reset.reset
		input  wire [6:0]   reconfig_mgmt_address,     //      reconfig_mgmt.address
		input  wire         reconfig_mgmt_read,        //                   .read
		output wire [31:0]  reconfig_mgmt_readdata,    //                   .readdata
		output wire         reconfig_mgmt_waitrequest, //                   .waitrequest
		input  wire         reconfig_mgmt_write,       //                   .write
		input  wire [31:0]  reconfig_mgmt_writedata,   //                   .writedata
		output wire [139:0] reconfig_to_xcvr,          //   reconfig_to_xcvr.reconfig_to_xcvr
		input  wire [91:0]  reconfig_from_xcvr         // reconfig_from_xcvr.reconfig_from_xcvr
	);

	alt_xcvr_reconfig #(
		.device_family                 ("Arria V"),
		.number_of_reconfig_interfaces (2),
		.enable_offset                 (1),
		.enable_lc                     (0),
		.enable_dcd                    (0),
		.enable_dcd_power_up           (1),
		.enable_analog                 (1),
		.enable_eyemon                 (0),
		.enable_ber                    (0),
		.enable_dfe                    (0),
		.enable_adce                   (0),
		.enable_mif                    (0),
		.enable_pll                    (0)
	) tr_reconf_ctrl_inst (
		.reconfig_busy             (reconfig_busy),             //      reconfig_busy.reconfig_busy
		.mgmt_clk_clk              (mgmt_clk_clk),              //       mgmt_clk_clk.clk
		.mgmt_rst_reset            (mgmt_rst_reset),            //     mgmt_rst_reset.reset
		.reconfig_mgmt_address     (reconfig_mgmt_address),     //      reconfig_mgmt.address
		.reconfig_mgmt_read        (reconfig_mgmt_read),        //                   .read
		.reconfig_mgmt_readdata    (reconfig_mgmt_readdata),    //                   .readdata
		.reconfig_mgmt_waitrequest (reconfig_mgmt_waitrequest), //                   .waitrequest
		.reconfig_mgmt_write       (reconfig_mgmt_write),       //                   .write
		.reconfig_mgmt_writedata   (reconfig_mgmt_writedata),   //                   .writedata
		.reconfig_to_xcvr          (reconfig_to_xcvr),          //   reconfig_to_xcvr.reconfig_to_xcvr
		.reconfig_from_xcvr        (reconfig_from_xcvr),        // reconfig_from_xcvr.reconfig_from_xcvr
		.tx_cal_busy               (),                          //        (terminated)
		.rx_cal_busy               (),                          //        (terminated)
		.cal_busy_in               (1'b0),                      //        (terminated)
		.reconfig_mif_address      (),                          //        (terminated)
		.reconfig_mif_read         (),                          //        (terminated)
		.reconfig_mif_readdata     (16'b0000000000000000),      //        (terminated)
		.reconfig_mif_waitrequest  (1'b0)                       //        (terminated)
	);

endmodule
// Retrieval info: <?xml version="1.0"?>
//<!--
//	Generated by Altera MegaWizard Launcher Utility version 1.0
//	************************************************************
//	THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
//	************************************************************
//	Copyright (C) 1991-2017 Altera Corporation
//	Any megafunction design, and related net list (encrypted or decrypted),
//	support information, device programming or simulation file, and any other
//	associated documentation or information provided by Altera or a partner
//	under Altera's Megafunction Partnership Program may be used only to
//	program PLD devices (but not masked PLD devices) from Altera.  Any other
//	use of such megafunction design, net list, support information, device
//	programming or simulation file, or any other related documentation or
//	information is prohibited for any other purpose, including, but not
//	limited to modification, reverse engineering, de-compiling, or use with
//	any other silicon devices, unless such use is explicitly licensed under
//	a separate agreement with Altera or a megafunction partner.  Title to
//	the intellectual property, including patents, copyrights, trademarks,
//	trade secrets, or maskworks, embodied in any such megafunction design,
//	net list, support information, device programming or simulation file, or
//	any other related documentation or information provided by Altera or a
//	megafunction partner, remains with Altera, the megafunction partner, or
//	their respective licensors.  No other licenses, including any licenses
//	needed under any third party's intellectual property, are provided herein.
//-->
// Retrieval info: <instance entity-name="alt_xcvr_reconfig" version="16.1" >
// Retrieval info: 	<generic name="device_family" value="Arria V" />
// Retrieval info: 	<generic name="number_of_reconfig_interfaces" value="2" />
// Retrieval info: 	<generic name="gui_split_sizes" value="" />
// Retrieval info: 	<generic name="enable_offset" value="1" />
// Retrieval info: 	<generic name="enable_dcd" value="0" />
// Retrieval info: 	<generic name="enable_dcd_power_up" value="1" />
// Retrieval info: 	<generic name="enable_analog" value="1" />
// Retrieval info: 	<generic name="enable_eyemon" value="0" />
// Retrieval info: 	<generic name="ber_en" value="0" />
// Retrieval info: 	<generic name="enable_dfe" value="0" />
// Retrieval info: 	<generic name="enable_adce" value="0" />
// Retrieval info: 	<generic name="enable_mif" value="0" />
// Retrieval info: 	<generic name="gui_enable_pll" value="0" />
// Retrieval info: 	<generic name="gui_cal_status_port" value="false" />
// Retrieval info: </instance>
// IPFS_FILES : TR_RECONF_CTRL.vo
// RELATED_FILES: TR_RECONF_CTRL.v, altera_xcvr_functions.sv, av_xcvr_h.sv, alt_xcvr_resync.sv, alt_xcvr_reconfig_h.sv, alt_xcvr_reconfig.sv, alt_xcvr_reconfig_cal_seq.sv, alt_xreconf_cif.sv, alt_xreconf_uif.sv, alt_xreconf_basic_acq.sv, alt_xcvr_reconfig_analog.sv, alt_xcvr_reconfig_analog_av.sv, alt_xreconf_analog_datactrl_av.sv, alt_xreconf_analog_rmw_av.sv, alt_xreconf_analog_ctrlsm.sv, alt_xcvr_reconfig_offset_cancellation.sv, alt_xcvr_reconfig_offset_cancellation_av.sv, alt_xcvr_reconfig_eyemon.sv, alt_xcvr_reconfig_dfe.sv, alt_xcvr_reconfig_adce.sv, alt_xcvr_reconfig_dcd.sv, alt_xcvr_reconfig_dcd_av.sv, alt_xcvr_reconfig_dcd_cal_av.sv, alt_xcvr_reconfig_dcd_control_av.sv, alt_xcvr_reconfig_mif.sv, av_xcvr_reconfig_mif.sv, av_xcvr_reconfig_mif_ctrl.sv, av_xcvr_reconfig_mif_avmm.sv, alt_xcvr_reconfig_pll.sv, av_xcvr_reconfig_pll.sv, av_xcvr_reconfig_pll_ctrl.sv, alt_xcvr_reconfig_soc.sv, alt_xcvr_reconfig_cpu_ram.sv, alt_xcvr_reconfig_direct.sv, alt_arbiter_acq.sv, alt_xcvr_reconfig_basic.sv, av_xrbasic_l2p_addr.sv, av_xrbasic_l2p_ch.sv, av_xrbasic_l2p_rom.sv, av_xrbasic_lif_csr.sv, av_xrbasic_lif.sv, av_xcvr_reconfig_basic.sv, alt_xcvr_arbiter.sv, alt_xcvr_m2s.sv, altera_wait_generate.v, alt_xcvr_csr_selector.sv, sv_reconfig_bundle_to_basic.sv, av_reconfig_bundle_to_basic.sv, av_reconfig_bundle_to_xcvr.sv, alt_xcvr_reconfig_cpu.v, alt_xcvr_reconfig_cpu_reconfig_cpu.v, alt_xcvr_reconfig_cpu_reconfig_cpu_test_bench.v, alt_xcvr_reconfig_cpu_mm_interconnect_0.v, alt_xcvr_reconfig_cpu_irq_mapper.sv, altera_reset_controller.v, altera_reset_synchronizer.v, altera_merlin_master_translator.sv, altera_merlin_slave_translator.sv, altera_merlin_master_agent.sv, altera_merlin_slave_agent.sv, altera_merlin_burst_uncompressor.sv, altera_avalon_sc_fifo.v, alt_xcvr_reconfig_cpu_mm_interconnect_0_router.sv, alt_xcvr_reconfig_cpu_mm_interconnect_0_router_001.sv, alt_xcvr_reconfig_cpu_mm_interconnect_0_router_002.sv, alt_xcvr_reconfig_cpu_mm_interconnect_0_router_003.sv, alt_xcvr_reconfig_cpu_mm_interconnect_0_cmd_demux.sv, alt_xcvr_reconfig_cpu_mm_interconnect_0_cmd_demux_001.sv, alt_xcvr_reconfig_cpu_mm_interconnect_0_cmd_mux.sv, altera_merlin_arbitrator.sv, alt_xcvr_reconfig_cpu_mm_interconnect_0_cmd_mux_001.sv, alt_xcvr_reconfig_cpu_mm_interconnect_0_rsp_mux.sv, alt_xcvr_reconfig_cpu_mm_interconnect_0_rsp_mux_001.sv, alt_xcvr_reconfig_cpu_mm_interconnect_0_avalon_st_adapter.v, alt_xcvr_reconfig_cpu_mm_interconnect_0_avalon_st_adapter_error_adapter_0.sv
