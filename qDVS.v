`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: UCSD ISN Lab
// Engineer: Rajkumar Kubendran
// 
// Create Date:    17:18:27 01/08/2020 
// Design Name: 
// Module Name:    qDVS 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: Program for Opal Kelly interface to communicate with qDVS v1.2 chip 
//
// Dependencies: 
//
// Revision: 1.0
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module qDVS(
	 // Opal Kelly Interface
	 input  wire [4:0]   okUH,
	 output wire [2:0]   okHU, //testbench
	 inout  wire [31:0]  okUHU,
	 inout  wire         okAA,

	 output wire [7:0]  led,
	 
	 // Clock input
	 //input sysclk_p,
	 //input sysclk_n,
	 
	 // Chip Interface
	 // AQ_Imager input signals
    output reg ROW_CLK,
    output reg ROW_CLR,
    output COL_CLK,
    output reg COL_CLR,
    output reg CODE_INC,
    output reg CODE_RST,
    output reg GATE_NBIAS,
    output reg EN_PULSE,
    output reg EN_STRING,
    output reg VDAC_PROBE,
    output reg _GLOAD,
    output reg GSTROBE,
    output reg DETECT,
    output reg AD_ENABLE,
    output reg SEL_POL,
    output reg RST_POL,
    output reg RST_SW,
    output reg RST_FB,
    output reg _RST_ALL,
    output reg _RST_SAMPLE,
	 output reg EN_EVENT_DET,
	 output reg EN_VMID,
	 output reg ROW_SEL_GLOBAL,
	 output reg EVENT_RST,
	 output DEMO_CLK,//fifo_full
	 output reg strobe_done,
	 output frame_done,//fifo_empty
	 output epA3read,
	 output [5:0] DATA_OUT_PROBE,
	 output TEST_COL_LOAD128_BUF,
    output TEST_COMP_STATE128_BUF,
    output TEST_RST_COL128_BUF,
	 
	 // DAC control signals
	 //voltage for current bias
	 output wire FPGA_BIAS_DAC_CS,
	 output wire FPGA_BIAS_DAC_SCK,
	 output wire FPGA_BIAS_DAC_SDI,
	
	 //voltage bias
	 output wire FPGA_DAC_CS,
	 output wire FPGA_DAC_SCK,
	 output wire FPGA_DAC_SDI,
	 
	 //only for testbench
	 //input wire RESET,
	 //input [31:0] dfu_din,
	 //input clktest,
	 //input data_valid,
	
	 // AQ_Imager output signals
    input [5:0] DATA_OUT,
    //input TEST_COL_LOAD0,
    //input TEST_RST_AMP0,
    //input TEST_COMP_STATE0,
    //input TEST_RST_COL0,
    input TEST_COL_LOAD128,
    //input TEST_RST_AMP128,
    input TEST_COMP_STATE128,
    input TEST_RST_COL128,
	 input EVENT
    );

 // Opal Kelly Interface
 // Target interface bus:
 wire         okClk;
 wire [112:0] okHE;
 wire [64:0]  okEH;

 // Endpoint connections: 
 wire [31:0]  ep00wire, ep10wire, ep11wire, ep12wire, ep13wire; 
 wire [31:0]  ep20wire, ep21wire, ep22wire, ep23wire;
 wire [31:0]  ep40wire, ep41wire;
 wire [31:0]  epA3pipe, epA4pipe;
 wire epA3read, epA4read;
 
 reg [9:0] row_clk_div_ctr;
 reg [9:0] row_clk_pause_ctr; 
 reg [7:0] demo_clk_div_ctr;
 reg [7:0] row_ctr;
 reg [7:0] col_ctr;
 reg [4:0] dac_ctr;
 wire RESET; // testbench
 wire CLK; //wire CLK;
 reg okClkby2, okClkby4, okClkby8, okClkby16, okClkby32, okClkby64, okClkby128, CLKby2;
 wire RD_CLK; // FIFO Read Clock
 reg SAMPLE_CLK; // FIFO Write Clock
 reg CLK_DAC;
 wire [7:0] mode;
 reg LOGMODE, EVENTMODE, SAMPLE_RESET, ALL_RESET, CLOCK_OFF;
 reg [6:0] string_ctr;
 reg [4:0] current_ctr;
 reg [10:0] frame_scan_ctr;
 //reg [14:0] bias_dac_value;
 reg pause_ctr;
 reg [7:0] pause_row_addr;
 reg [7:0] pause_col_addr;
 reg [31:0] frame_rst_delay;
 reg [7:0] frame_cnt;
 reg [31:0] frame_delay_cntr;
 
 reg DEMO_CLK;
 reg frame_done;
 reg frame_scan_wait;
 reg row_scan_skip;
 
 //////////////////////////////
// DAC control
//

wire daisy_full;
wire daisy_wr_en;
wire [31:0] daisy_din;
wire daisy_done;
wire [1:0] dac_cont_mux;   

wire [31:0] dfu_dout;
wire dfu_empty;
wire dfu_valid;
wire dfu_rd_en;
//reg VDAC_EXT_STRING;
//reg [31:0] dout_low, dout_high;
//wire CODE_INC;
//assign CODE_INC = CLK;
//reg strobe_done;
reg fifo_wr_clk_en;

wire BIAS_DAC_CS;
wire BIAS_DAC_SCK;
wire BIAS_DAC_SDI;
wire DAC_CS;
wire DAC_SCK;
wire DAC_SDI;
wire [5:0] DATA_OUT_BUF;
//IBUFG  hi_clk_bufg  (.I(okClk), .O(CLK));

assign RESET = ep00wire[0]; //only for testbench
assign RD_CLK = ~COL_CLK; //fifo_wr_clk_en ? CLK : 1; //
//assign CLK = ~CLOCK_OFF & ((ep12wire[1:0] == 2'b11) ? okClkby8 : ((ep12wire[1:0] == 2'b10) ? okClkby4 : ((ep12wire[1:0] == 2'b01) ? okClkby2 : okClk)));//CLK_DAC; 
assign CLK = ~CLOCK_OFF & ((ep12wire[1:0] == 2'b11) ? okClkby8 : ((ep12wire[1:0] == 2'b10) ? okClkby8 : ((ep12wire[1:0] == 2'b01) ? okClkby16 : okClkby8)));//CLK_DAC; 
//assign CLK_DAC = CLK;
assign COL_CLK = ~CLOCK_OFF & (strobe_done&((col_ctr != pause_col_addr)|(~pause_ctr)) ? CLKby2 : 1); //&
assign DATA_OUT_BUF[5:0] = ~DATA_OUT[5:0];//(row_ctr == pause_row_addr) ? DATA_OUT[5:0] : 0;//
/*assign TEST_COL_LOAD0_BUF = ~(~TEST_COL_LOAD0);
assign TEST_RST_AMP0_BUF = ~(~TEST_RST_AMP0);
assign TEST_COMP_STATE0_BUF = ~(~TEST_COMP_STATE0);
assign TEST_RST_COL0_BUF = ~(~TEST_RST_COL0);*/
assign TEST_COL_LOAD128_BUF = ~(~TEST_COL_LOAD128);
//assign TEST_RST_AMP128_BUF = ~(~TEST_RST_AMP128);
assign TEST_COMP_STATE128_BUF = ~(~TEST_COMP_STATE128);
assign TEST_RST_COL128_BUF = ~(~TEST_RST_COL128);

assign DATA_OUT_PROBE[5:0] = ((row_ctr == 125)&&(col_ctr == 125)) ? ~DATA_OUT[5:0] : 0;

//assign ep20wire   = {8'd0, ~DATA_OUT[5:0]}; // row_clk_div_ctr
assign ep20wire[5:0] = DATA_OUT_BUF[5:0];
assign ep21wire   = {8'd0, row_ctr};//pause_ctr ? pause_row_addr : row_ctr};  
assign ep22wire   = {8'd0, col_ctr};
//assign epA3pipe   = {row_ctr, col_ctr};
//assign epA4pipe   = {2'd0, DATA_OUT[5:0]};
assign dfu_empty = 1'b0; //ep00wire[2];
assign dfu_valid = 1'b1;//ep00wire[3];//data_valid; 
assign dfu_dout[15:0] = ep10wire; //VDAC_EXT_STRING ? dout_low : ep10wire; //dfu_din[15:0]; //only for testbench
assign dfu_dout[31:16] = ep11wire; //VDAC_EXT_STRING ? dout_high : ep11wire; //dfu_din[31:16]; //only for testbench
/*
always @(posedge okClk or posedge RESET)
begin
	if(RESET) begin
		CLK <= 0;
	end
	else begin
		CLK <= ~CLK;
	end
end

always @(negedge okClk or posedge RESET)
begin
	if(RESET) begin
		RD_CLK <= 0;
	end
	else begin
		RD_CLK <= ~RD_CLK;
	end
end
*/
always @(posedge CLK or posedge RESET)
begin
	if(RESET) begin
		CLKby2 <= 0;
	end
	else begin
		CLKby2 <= ~CLKby2;
	end
end

always @(posedge okClkby2 or posedge RESET)
begin
	if(RESET) begin
		okClkby4 <= 0;
	end
	else begin
		okClkby4 <= ~okClkby4;
	end
end

always @(posedge okClkby4 or posedge RESET)
begin
	if(RESET) begin
		okClkby8 <= 0;
	end
	else begin
		okClkby8 <= ~okClkby8;
	end
end

always @(posedge okClkby8 or posedge RESET)
begin
	if(RESET) begin
		okClkby16 <= 0;
	end
	else begin
		okClkby16 <= ~okClkby16;
	end
end

always @(posedge okClkby16 or posedge RESET)
begin
	if(RESET) begin
		okClkby32 <= 0;
	end
	else begin
		okClkby32 <= ~okClkby32;
	end
end

always @(posedge okClkby32 or posedge RESET)
begin
	if(RESET) begin
		okClkby64 <= 0;
	end
	else begin
		okClkby64 <= ~okClkby64;
	end
end

always @(posedge okClkby64 or posedge RESET)
begin
	if(RESET) begin
		okClkby128 <= 0;
	end
	else begin
		okClkby128 <= ~okClkby128;
	end
end

// Clock for DAC = 0.25*CLK frequency
always @(posedge okClk or posedge RESET)
begin
	if(RESET) begin
		CLK_DAC <= 0;
		okClkby2 <= 0;
		//SAMPLE_CLK <= 0;
		dac_ctr <= 0;
	end
	else begin
		dac_ctr <= dac_ctr + 1;
		okClkby2 <= ~okClkby2;
		case(dac_ctr)/*
		5: begin//if (dac_ctr == 5) begin
			//CLK <= ~CLK;
			//okClkby2 <= ~okClkby2;
			end
		7: begin
			SAMPLE_CLK <= ~SAMPLE_CLK;
			end
		10: begin//if (dac_ctr == 10) begin
			CLK <= ~CLK;
			end
		12: begin
			SAMPLE_CLK <= ~SAMPLE_CLK;
			end
		15: begin//if (dac_ctr == 15) begin
			CLK <= ~CLK;
			end
		17: begin
			SAMPLE_CLK <= ~SAMPLE_CLK;
			end*/
		10: begin//if (dac_ctr == 20) begin
			CLK_DAC <= ~CLK_DAC;
			//CLK <= ~CLK;	
			dac_ctr <= 0;
			end/*
		22: begin
			SAMPLE_CLK <= ~SAMPLE_CLK;
			dac_ctr <= 0;
			end*/
		endcase
	end
end

// Control Signals for selecting Mode of operation
always @(posedge CLK or posedge RESET)
begin
	if(RESET) begin
		//AD_ENABLE <= 0;
		LOGMODE <= 1;
		VDAC_PROBE <= 0;
		EVENTMODE <= 0;
		//EN_PULSE <= 0;
		SAMPLE_RESET <= 0;
		ALL_RESET <= 0;
		CLOCK_OFF <= 0;
		pause_ctr <= 0;
		pause_row_addr <= 0;
		pause_col_addr <= 0;
		frame_rst_delay <= 0;
	end
	else begin
		//AD_ENABLE <= ep00wire[4];
		CLOCK_OFF <= ep00wire[4];
		LOGMODE <= ep00wire[5];
		VDAC_PROBE <= ep00wire[8];
		EVENTMODE <= ~ep00wire[9];
		//EN_PULSE <= ep00wire[10];
		SAMPLE_RESET <= ep00wire[6];
		ALL_RESET <= ep00wire[7];
		pause_ctr <= 0;//ep12wire[2];
		pause_row_addr <= ep12wire[11:4];
		pause_col_addr <= ep12wire[19:12];
		frame_rst_delay <= ep13wire[31:0];
	end
end

assign FPGA_BIAS_DAC_CS = BIAS_DAC_CS;
assign FPGA_BIAS_DAC_SCK = BIAS_DAC_SCK; 
assign FPGA_BIAS_DAC_SDI = BIAS_DAC_SDI;
assign FPGA_DAC_CS = DAC_CS;
assign FPGA_DAC_SCK = DAC_SCK;
assign FPGA_DAC_SDI = DAC_SDI;
/*
always @(posedge CLK or posedge RESET)
begin
	if(RESET) begin
		//ROW_CLR <= 1;
		//COL_CLR <= 1;
		FPGA_BIAS_DAC_CS <= 0;
		FPGA_BIAS_DAC_SCK <= 0;
		FPGA_BIAS_DAC_SDI <= 0;
		FPGA_DAC_CS <= 0;
		FPGA_DAC_SCK <= 0;
		FPGA_DAC_SDI <= 0;
		
	end
	else begin
		//ROW_CLR <= 0;
		//COL_CLR <= 0;
		FPGA_BIAS_DAC_CS <= BIAS_DAC_CS;
		FPGA_BIAS_DAC_SCK <= BIAS_DAC_SCK; 
		FPGA_BIAS_DAC_SDI <= BIAS_DAC_SDI;
		FPGA_DAC_CS <= DAC_CS;
		FPGA_DAC_SCK <= DAC_SCK;
		FPGA_DAC_SDI <= DAC_SDI;
	end
end
*/
//GATE NBIAS Generation for testing ONLY (constant illumination)
/*
always @(posedge frame_done or posedge RESET)
begin
	if(RESET) begin
		DEMO_CLK <= 0;
		//GATE_NBIAS <= 0;
	end
	else begin
		DEMO_CLK <= ~DEMO_CLK;
		//GATE_NBIAS <= ~GATE_NBIAS;
	end
end
*/
always @(posedge CLK or posedge RESET)
begin
	if(RESET) begin
		ROW_CLK <= 1;
		row_clk_div_ctr <= 0;
		row_clk_pause_ctr <= 0;
		frame_scan_ctr <= 0;
		demo_clk_div_ctr <= 0;
		frame_done <= 1;
		frame_cnt <= 0;
		DEMO_CLK <= 0;
		//GATE_NBIAS <= 0;
		row_ctr <= 0;
		frame_delay_cntr <= 0;
		//_RST_ALL <= 1;
		ROW_CLR <= 1;
		frame_scan_wait <= 0;
		row_scan_skip <= 0;
	end
	else begin
		if (EVENTMODE == 1) begin
			if ((frame_rst_delay > 0) && (frame_delay_cntr < frame_rst_delay))
				frame_scan_wait <= 1;
			else
				frame_scan_wait <= 0;
				
			if (frame_scan_wait) begin
				row_clk_div_ctr <= 0;
				if (frame_delay_cntr < frame_rst_delay)
					frame_delay_cntr <= frame_delay_cntr + 1;
			end
			else begin
				row_clk_div_ctr <= row_clk_div_ctr + 1;
				if ((demo_clk_div_ctr == 255) && (row_clk_div_ctr == 519))
					frame_delay_cntr <= 0;
			end
			
			
			if (demo_clk_div_ctr == 0) begin
				if (row_clk_div_ctr == 5) begin
					//_RST_ALL	<= 0;
					ROW_CLR <= 1;
					//COL_CLR <= 1;
				end
				if (row_clk_div_ctr == 10) begin
					//_RST_ALL	<= 1;
					ROW_CLR <= 0;
					//COL_CLR <= 0;
				end
			end
			
			if (row_clk_div_ctr == 519) begin 
				row_clk_div_ctr <= 0;
				
				if (demo_clk_div_ctr == 127) begin
					frame_done <= 0;//~frame_done;
					//DEMO_CLK <= ~DEMO_CLK;
					//demo_clk_div_ctr <= 0;
				end
				if (demo_clk_div_ctr == 255) begin
					DEMO_CLK <= ~DEMO_CLK;
					//GATE_NBIAS <= ~GATE_NBIAS;
					frame_done <= 1;//~frame_done;
					frame_cnt <= frame_cnt + 1;
					//demo_clk_div_ctr <= 0;
				end
				demo_clk_div_ctr <= demo_clk_div_ctr + 1;
			end
			
			if (frame_scan_wait) begin
				ROW_CLK <= 1;
			end
			else begin
				if (row_clk_div_ctr == 260) begin //511
					ROW_CLK <= 0;//~ROW_CLK;
				end
				if (row_clk_div_ctr == 435) begin //511
					row_ctr <= row_ctr + 1;
				end
				if (row_clk_div_ctr == 519) begin 
					ROW_CLK <= 1;//~ROW_CLK;
					//row_clk_div_ctr <= 0;
					//row_ctr <= row_ctr + 1;
					//row_clk_pause_ctr <= 0;
				end
			end
			
			if (row_ctr < pause_row_addr || row_ctr > pause_col_addr) begin
				row_scan_skip <= row_ctr[2];
			end
			else begin
				row_scan_skip <= 0;
			end
		end
		else begin
			frame_scan_ctr <= frame_scan_ctr + 1;
			if (demo_clk_div_ctr == 0) begin
				if (frame_scan_ctr == 0) begin
					ROW_CLR <= 1;
				end
				if (frame_scan_ctr == 5) begin
					ROW_CLR <= 0;
				end
			end
			if (frame_scan_ctr == 2047) begin 
				frame_scan_ctr <= 0;
				demo_clk_div_ctr <= demo_clk_div_ctr + 1;
				if (demo_clk_div_ctr == 127) begin
					frame_done <= 0;//~frame_done;
					DEMO_CLK <= ~DEMO_CLK;
				end
				if (demo_clk_div_ctr == 255) begin
					DEMO_CLK <= ~DEMO_CLK;
					//GATE_NBIAS <= ~GATE_NBIAS;
					frame_done <= 1;//~frame_done;
					frame_cnt <= frame_cnt + 1;
					//demo_clk_div_ctr <= 0;
				end
			end
			
			if (frame_scan_ctr == 1023) begin 
				/*if (pause_ctr) begin
					ROW_CLK <= 0;
				end
				else begin*/
					ROW_CLK <= 0;//~ROW_CLK;
				//end
			end
			if (frame_scan_ctr == 2047) begin 
				/*if (pause_ctr) begin
					ROW_CLK <= 0;
				end
				else begin*/
					ROW_CLK <= 1;//~ROW_CLK;
					//frame_scan_ctr <= 0;
				//end
				row_ctr <= row_ctr + 1;
				/*if (row_ctr == 255) begin
					GATE_NBIAS <= ~GATE_NBIAS;
				end*/
			end
		end
	end
end
/*
always @(posedge ROW_CLK or posedge RESET)
begin
	if(RESET) begin
		row_ctr <= 0;
	end
	else begin
		row_ctr <= row_ctr + 1;
	end
end

always @(posedge ROW_CLK or posedge RESET)
begin
	if(RESET) begin
		row_ctr <= 0;
//		GATE_NBIAS <= 0;
	end
	else begin
		row_ctr <= row_ctr + 1;
//		if (row_ctr == 255) begin
	//		GATE_NBIAS <= ~GATE_NBIAS;
		//end
	end
end
*/
always @(posedge COL_CLK or posedge RESET)
begin
	if(RESET) begin
		col_ctr <= 0;
	end
	else begin
		col_ctr <= col_ctr + 1;
	end
end

/*
//DEMO CLK - LED toggle once per 2 full frame
always @(posedge ROW_CLK or posedge RESET)
begin
	if(RESET) begin
		DEMO_CLK <= 0;
		demo_clk_div_ctr <= 0;
		frame_done <= 0;
	end
	else begin
		demo_clk_div_ctr <= demo_clk_div_ctr + 1;
		if (demo_clk_div_ctr == 255) begin
			frame_done <= ~frame_done;
		end
		if (demo_clk_div_ctr == 511) begin
			DEMO_CLK <= ~DEMO_CLK;
			frame_done <= ~frame_done;
			demo_clk_div_ctr <= 0;
		end
	end
end
*/
//assign EN_VMID = 0;
//assign EN_EVENT_DET = 0;
//assign ROW_SEL_GLOBAL = 1;
//assign EVENT_RST = CODE_RST;

always @(posedge CLK or posedge RESET)
begin
	if(RESET) begin
	   CODE_INC <= 0;
		SEL_POL <= 0;
		RST_POL <= 0;
		RST_SW <= 1;
		RST_FB <= 1;
		_RST_ALL <= 1;
		_RST_SAMPLE <= 1;
		GATE_NBIAS <= 1;
		DETECT <= 0;
		_GLOAD <= 1;
		GSTROBE <= 0;
		strobe_done <= 0;
		fifo_wr_clk_en <= 0;
		CODE_RST <= 1;
		AD_ENABLE <= 0;
		string_ctr <= 0;
		current_ctr <= 0;
		//VDAC_PROBE <= 0;
		EN_PULSE <= 0;
		EN_STRING <= 0;
		COL_CLR <= 1;
		//row_ctr <= 0;
		/*VDAC_EXT_STRING <= 0;
		dout_low <= 0;
		dout_high <= 0;
		bias_dac_value <= 0;*/
		EN_VMID <= 1;
		EN_EVENT_DET <= 0;
		ROW_SEL_GLOBAL <= 1;
		EVENT_RST <= 1;
		
	end
	else begin
		//RST_SW <= 1;
		//RST_FB <= 1;
		//EN_STRING <= 0;
		//VDAC_PROBE <= 0;
		//EN_EVENT_DET <= 0;
		//AD_ENABLE <= 0;
		
		//_RST_SAMPLE <= 1;
		//_RST_ALL <= 1;
		if (row_scan_skip == 1) begin
			//if (row_ctr == pause_col_addr+1) begin
				// RESET OUTPUT BUS TO 0
				if (row_clk_div_ctr == 10) begin
					CODE_RST	<= 1;
					EVENT_RST <= 1;
					DETECT <= 1;
					SEL_POL <= 1;
					RST_POL <= 1;
				end
				if (row_clk_div_ctr == 20) begin
					CODE_RST <= 0;
					EVENT_RST <= 0;
					_GLOAD <= 0;
				end
				if (row_clk_div_ctr == 25) begin
					SEL_POL <= 0;
					RST_POL <= 0;
				end
				if (row_clk_div_ctr == 30) begin
					_GLOAD <= 1;
					DETECT <= 0;
				end	
			//end
		end
		else begin
			if (EVENTMODE == 1) begin
				EN_STRING <= 0;
				AD_ENABLE <= 0;
				GATE_NBIAS <= 1;
				/*if (~pause_ctr) begin
					if (LOGMODE == 1) begin
						GATE_NBIAS <= 1;
					end
					else begin
						if (row_clk_div_ctr == 0) begin
							GATE_NBIAS <= 0;
						end
						if (row_clk_div_ctr == 515) begin // 515
							GATE_NBIAS <= 1;
						end
					end
				end
				else begin
					if (row_clk_div_ctr == 260) begin
						GATE_NBIAS <= 0;
					end
					if (row_clk_div_ctr == 515) begin // 515
						GATE_NBIAS <= 1;
					end
				end
				*/
				// RESET ALL PIXELS EVERY NEW ROW
				if (ALL_RESET) begin
					//if (demo_clk_div_ctr == 0) begin
					if (row_clk_div_ctr == 440) begin //440
						_RST_ALL	<= 0;
						//_RST_SAMPLE <= 0;
						RST_SW <= 1;
						RST_FB <= 1;
					end/*
					if (row_clk_div_ctr == 445) begin
						_RST_ALL	<= 1;
					end
					if (row_clk_div_ctr == 450) begin
						_RST_ALL	<= 0;
					end
					if (row_clk_div_ctr == 455) begin
						_RST_ALL	<= 1;
					end
					if (row_clk_div_ctr == 460) begin
						_RST_ALL	<= 0;
					end
					if (row_clk_div_ctr == 465) begin
						_RST_ALL	<= 1;
					end
					if (row_clk_div_ctr == 470) begin
						_RST_ALL	<= 0;
					end
					if (row_clk_div_ctr == 475) begin
						_RST_ALL	<= 1;
					end
					if (row_clk_div_ctr == 460) begin
						_RST_ALL	<= 0;
					end
					if (row_clk_div_ctr == 470) begin
						_RST_ALL	<= 1;
					end
					if (row_clk_div_ctr == 480) begin
						_RST_ALL	<= 0;
					end
					if (row_clk_div_ctr == 490) begin
						_RST_ALL	<= 1;
					end
					if (row_clk_div_ctr == 500) begin
						_RST_ALL	<= 0;
					end
					if (row_clk_div_ctr == 510) begin
						_RST_ALL	<= 1;
					end
					if (row_clk_div_ctr == 510) begin
						_RST_ALL	<= 0;
					end*/
					if (row_clk_div_ctr == 465) begin //515
						//_RST_ALL	<= 1;
						RST_SW <= 0;
						//RST_FB <= 0;
					end/*
					if (row_clk_div_ctr == 480) begin //519
						_RST_ALL	<= 1;
						//RST_FB <= 0;
						//ROW_SEL_GLOBAL <= 0;
					end*/
					if (row_clk_div_ctr == 500) begin //519
						_RST_ALL	<= 1;
						//_RST_SAMPLE <= 1;
						RST_FB <= 0;
						//ROW_SEL_GLOBAL <= 0;
					end
					//end
				end
				else
					_RST_ALL <= 1;
					
				
				// RESET OUTPUT BUS TO 0
				if (row_clk_div_ctr == 10) begin
					CODE_RST	<= 1;
					EVENT_RST <= 1;
					DETECT <= 1;
				end
				if (row_clk_div_ctr == 20) begin
					CODE_RST <= 0;
					EVENT_RST <= 0;
					_GLOAD <= 0;
					ROW_SEL_GLOBAL <= 1;
				end
				if (row_clk_div_ctr == 25) begin
					SEL_POL <= 0;
					RST_POL <= 0;
				end
				if (row_clk_div_ctr == 30) begin
					_GLOAD <= 1;
					DETECT <= 0;
					//CODE_RST <= 1;
					EVENT_RST <= 1;
				end	
				if (row_clk_div_ctr == 35) begin
					//CODE_RST <= 0;
					EVENT_RST <= 0;
					EN_VMID <= 0;
					//VDAC_PROBE <= 0;
					EN_PULSE <= 1;
				end
				// DETECT POSITIVE EVENTS
				if (row_clk_div_ctr == 40) begin
					CODE_INC <= 1;
					//DETECT <= 1;
				end/*
				if (row_clk_div_ctr == 75) begin
					_GLOAD <= 0;
				end
				if (row_clk_div_ctr == 42) begin
					CODE_INC <= 0;
				end
				if (row_clk_div_ctr == 45) begin
					CODE_INC <= 1;
				end
				if (row_clk_div_ctr == 47) begin
					CODE_INC <= 0;
				end
				if (row_clk_div_ctr == 50) begin
					CODE_INC <= 1;
				end*/
				if (row_clk_div_ctr == 45) begin
					CODE_INC <= 0;
					//_GLOAD <= 1;
				end
				if (row_clk_div_ctr == 160) begin // +50
					DETECT <= 1;
				end
				if (row_clk_div_ctr == 180) begin //110
					_GLOAD <= 0;
				end
				if (row_clk_div_ctr == 200) begin //130
					_GLOAD <= 1;
					if (ALL_RESET) begin
						DETECT <= 0;
					end
				end
				if (SAMPLE_RESET) begin
					if (row_clk_div_ctr == 200) begin //140
						//VDAC_PROBE <= 1;
						EN_VMID <= 1;
						EN_PULSE <= 0;
					end
					if (row_clk_div_ctr == 205) begin //150 +50
						_RST_SAMPLE <= 0;
						RST_SW <= 1;
						RST_FB <= 1;
					end
					if (row_clk_div_ctr == 210) begin //170
						RST_SW <= 0;
					end
					if (row_clk_div_ctr == 215) begin //200
						_RST_SAMPLE <= 1;
						//RST_SW <= 0;
						RST_FB <= 0;
						EN_VMID <= 0;
						EN_PULSE <= 1;	
					end
				end
					
				// DETECT NEGATIVE EVENTS
				if (row_clk_div_ctr == 215) begin //210
					if (SAMPLE_RESET) begin
						DETECT <= 0;
					end
					//CODE_RST <= 1;
					EVENT_RST <= 1;
				end
				if (row_clk_div_ctr == 220) begin
					//CODE_RST <= 0;
					EVENT_RST <= 0;
					SEL_POL <= 1;
					RST_POL <= 1;
				end
				if (row_clk_div_ctr == 230) begin
					CODE_INC <= 1;		
				end
				if (row_clk_div_ctr == 240) begin
					CODE_INC <= 0;
				end
				if (row_clk_div_ctr == 250) begin
					CODE_INC <= 1;
				end
				if (row_clk_div_ctr == 260) begin
					CODE_INC <= 0;
					//DETECT <= 1;
				end/*
				if (row_clk_div_ctr == 270) begin
					CODE_INC <= 1;
					_GLOAD <= 0;
				end
				if (row_clk_div_ctr == 280) begin
					CODE_INC <= 0;
					_GLOAD <= 1;
				end*/
				if (row_clk_div_ctr == 380) begin //+50
					DETECT <= 1;
				end
				if (row_clk_div_ctr == 410) begin //320
					_GLOAD <= 0;
				end/*
				if (row_clk_div_ctr == 420) begin //340
					_GLOAD <= 1;
					if (ALL_RESET) begin
						DETECT <= 0;
					end
				end*/
				if (row_clk_div_ctr == 420) begin //340
					_GLOAD <= 1;
					if (ALL_RESET) begin
						DETECT <= 0;
						EN_PULSE <= 0;
						EN_VMID <= 1;	
					end
				end
				if (SAMPLE_RESET) begin
					if (row_clk_div_ctr == 430) begin //350
						//_GLOAD <= 1;
						//VDAC_PROBE <= 1;
						EN_VMID <= 1;
						EN_PULSE <= 0;
					end
					if (row_clk_div_ctr == 435) begin //360 +50
						_RST_SAMPLE <= 0;
						RST_SW <= 1;
						RST_FB <= 1;
					end
					if (row_clk_div_ctr == 440) begin //380
						RST_SW <= 0;
					end
					if (row_clk_div_ctr == 445) begin //410
						_RST_SAMPLE <= 1;
						//RST_SW <= 0;
						RST_FB <= 0;
					end
				end
				if (row_clk_div_ctr == 450) begin //410
					if (SAMPLE_RESET) begin
						DETECT <= 0;
						EN_PULSE <= 0;
						EN_VMID <= 1;	
					end
					
				end/*
				if (row_clk_div_ctr == 370) begin
					VDAC_PROBE <= 1;
					EN_PULSE <= 0;
				end*/
				
				
				/*
				if (row_clk_div_ctr == 236) begin				
					fifo_wr_clk_en <= 1;
				end
				if (row_clk_div_ctr == 232) begin				
					fifo_wr_clk_en <= 0;
				end*/
				
				// STROBE ALL COLUMNS TO READOUT DATA
				if (row_clk_div_ctr == 440) begin				
					strobe_done <= 1; //column readout for 256 cycles
				end
				if (row_clk_div_ctr == 431) begin				
					strobe_done <= 0; //column readout for 256 cycles
					/*CODE_RST	<= 1;
					if (SAMPLE_RESET) begin
						RST_FB <= 0;
					end*/
				end
				if (row_clk_div_ctr == 431) begin //451
					//strobe_done <= 0;
					GSTROBE <= 1;
					//EN_EVENT_DET <= 1;
					if (~pause_ctr) begin
						COL_CLR <= 1;
						//row_ctr <= row_ctr + 1;
					end
				end
				// RESET COLUMN COUNTER
				if (row_clk_div_ctr == 435) begin
					GSTROBE <= 0;	
					EN_EVENT_DET <= 1;
					//CODE_RST	<= 0;
					
					if (~pause_ctr) begin
						COL_CLR <= 0;
						//row_ctr <= row_ctr + 1;
					end			
				end
				if (row_clk_div_ctr == 519) begin 
					ROW_SEL_GLOBAL <= 0;
					EN_EVENT_DET <= 0;
				end/*
				if (pause_ctr) begin
					if (~((row_ctr == pause_row_addr)&&(pause_ctr))) begin
						if (~((frame_delay_cntr < frame_rst_delay) && frame_done)) begin
							row_ctr <= row_ctr +1;
						end
					end
				end*/
			end
			else begin
				if (LOGMODE == 1) begin
					GATE_NBIAS <= 1;
				end
				else begin
					if (frame_scan_ctr == 0) begin
						GATE_NBIAS <= 0;
					end
					if (frame_scan_ctr == 2040) begin
						GATE_NBIAS <= 1;
					end
				end
				SEL_POL <= ep00wire[10];
				RST_POL <= ep00wire[10];
				EN_PULSE <= 0;
				EN_EVENT_DET <= 0;
				AD_ENABLE <= 1;	
				//EN_STRING <= 1;			
				//EN_VMID <= 0;
				
				//DAC EXT STRING Programming
				if (frame_scan_ctr == 0) begin
					CODE_RST	<= 1;
					EVENT_RST <= 1;
				end
				if (frame_scan_ctr == 1) begin //1
					/*_RST_ALL	<= 0;
					RST_SW <= 1;
					RST_FB <= 1;*/
					ROW_SEL_GLOBAL <= 1;
				end
				if (frame_scan_ctr == 4) begin //4
					/*_RST_ALL	<= 1;
					RST_SW <= 1;
					RST_FB <= 1;*/
					CODE_RST	<= 0;
					EVENT_RST <= 0;
					string_ctr <= 0;
					EN_VMID <= 0;
					EN_STRING <= 1;
					current_ctr <= frame_scan_ctr[4:0];
					//SEL_POL <= ep00wire[10];
					//RST_POL <= ep00wire[10];
				end

				if ((string_ctr <= 63) && (frame_scan_ctr > 4)) begin
					if (frame_scan_ctr[4:0] == 5) begin
						CODE_INC <= 1;
						EVENT_RST <= 1;
						SEL_POL <= ep00wire[10];
						RST_POL <= ep00wire[10];
						string_ctr <= string_ctr + 1;
					end
					if (frame_scan_ctr[4:0] == 9) begin
						CODE_INC <= 0;
						EVENT_RST <= 0;
					end
					//if (string_ctr[3:0] == 15) begin
					if (frame_scan_ctr[4:0] == 16) begin
						DETECT <= 1;
					end
					if (frame_scan_ctr[4:0] == 22) begin
						_GLOAD <= 0;
					end/*
					if (frame_scan_ctr[4:0] == 23) begin
						if (string_ctr == row_ctr[5:0]) begin
							SEL_POL <= ~SEL_POL;
							RST_POL <= ~RST_POL;
						end
					end*/
					if (frame_scan_ctr[4:0] == 24) begin
						_GLOAD <= 1;
						//EVENT_RST <= 1;
						if (SAMPLE_RESET) begin
							RST_SW <= 1;
							RST_FB <= 1;
							EN_VMID <= 1;
							EN_STRING <= 0;
							_RST_SAMPLE <= 0;
						end
					end
					if (frame_scan_ctr[4:0] == 29) begin
						DETECT <= 0;
						//EVENT_RST <= 0;
						//SEL_POL <= ep00wire[10];
						//RST_POL <= ep00wire[10];
						if (SAMPLE_RESET) begin
							RST_SW <= 0;
							RST_FB <= 0;
							_RST_SAMPLE <= 1;
							EN_VMID <= 0;
							EN_STRING <= 1;
						end
						//string_ctr <= string_ctr + 1;
						//current_ctr <= 0;
					end
					/*
					if (frame_scan_ctr[4:0] == current_ctr + 21) begin
						CODE_RST <= 1;
					end
					if (frame_scan_ctr[4:0] == current_ctr + 27) begin
						CODE_RST <= 0;
						string_ctr <= string_ctr + 1;
						current_ctr <= frame_scan_ctr;//[4:0];
					end
					if (string_ctr == 31) begin
						CODE_RST <= 1;
					end
					if (string_ctr == 32) begin
						CODE_RST <= 0;
					end*/
				end
				if (frame_scan_ctr == 1975) begin //1775
					GSTROBE <= 1;
					COL_CLR <= 1;
				end
				if (frame_scan_ctr == 1980) begin //1780
					GSTROBE <= 0;
					COL_CLR <= 0;
					
				end
				if (frame_scan_ctr == 1982) begin //1782
					strobe_done <= 1; 
				end
				if (frame_scan_ctr == 445) begin //237
					strobe_done <= 0; //column readout for 512 cycles
				end
				if (frame_scan_ctr == 2010) begin
					//VDAC_PROBE <= 0;
					EN_VMID <= 1;
					EN_STRING <= 0;
				end
				if (ALL_RESET) begin
					if (frame_scan_ctr == 2020) begin //2040
						_RST_ALL <= 0;
						RST_SW <= 1;
						RST_FB <= 1;
					end
					if (frame_scan_ctr == 2025) begin //2042
						//_RST_ALL <= 1;
						RST_SW <= 0;
						//RST_FB <= 1;
					end
					if (frame_scan_ctr == 2040) begin //2042
						_RST_ALL <= 1;
						//RST_SW <= 0;
						RST_FB <= 0;
					end
				end
				if (frame_scan_ctr == 2045) begin
					ROW_SEL_GLOBAL <= 0;
					//row_ctr <= row_ctr + 1;
				end
			end
		end
	end
end




//////////////////////////////////////////
/// DAC daisy chain control
///
wire dac0_program, dac0_update;
wire dac0_fifo_full, dac0_fifo_ack;
wire dac1_program, dac1_update;
wire dac1_fifo_full, dac1_fifo_ack;

assign dac0_program = ep40wire[0];
assign dac0_update = ep41wire[0];
assign ep23wire[0] = dac0_fifo_full;
assign ep23wire[1] = dac0_fifo_ack;

assign dac1_program = ep40wire[1];
assign dac1_update = ep41wire[1];
assign ep23wire[2] = dac1_fifo_full;
assign ep23wire[3] = dac1_fifo_ack;

dac_daisy_control dac0_ctrl(
	.clk(okClk),
	.clk_dac(CLK_DAC),
	.rst(RESET),
	.program(dac0_program),
	.update(dac0_update),
	.ep_din(dfu_dout),
	.fifo_full(dac0_fifo_full),
	.fifo_wr_done(dac0_fifo_ack),
	.dac_sck(BIAS_DAC_SCK),
	.dac_cs_b(BIAS_DAC_CS),
	.dac_sdi(BIAS_DAC_SDI)
	);

dac_daisy_control dac1_ctrl(
	.clk(okClk),
	.clk_dac(CLK_DAC),
	.rst(RESET),
	.program(dac1_program),
	.update(dac1_update),
	.ep_din(dfu_dout),
	.fifo_full(dac1_fifo_full),
	.fifo_wr_done(dac1_fifo_ack),
	.dac_sck(DAC_SCK),
	.dac_cs_b(DAC_CS),
	.dac_sdi(DAC_SDI)
	);
/*
dac_daisy_conts DAC_DAISY_CONTS (
    .clk(CLK), 
    .clk_dac(CLK_DAC), 
    .rst(RESET), 
	
    .daisy_full(daisy_full), 
    .daisy_wr_en(daisy_wr_en), 
    .daisy_din(daisy_din), 
    .daisy_done(daisy_done), 
    
	.dac_cont_mux(dac_cont_mux), 
    
	.dac_daisy0_sck(BIAS_DAC_SCK), 
    .dac_daisy0_cs(BIAS_DAC_CS), 
    .dac_daisy0_sdi(BIAS_DAC_SDI), 
    
	.dac_daisy1_sck(DAC_SCK), 
    .dac_daisy1_cs(DAC_CS), 
    .dac_daisy1_sdi(DAC_SDI)
    );
	 */
//////////////////////////////////////////
/// CMD interpreter
///
cmd_interpreter CMD_INTERPRETER (
    .clk(CLK), 
    .rst(RESET), 
	
    .dfu_rd_en(dfu_rd_en), 
    .dfu_dout(dfu_dout), 
    .dfu_empty(dfu_empty), 
    .dfu_valid(dfu_valid), 
	
    .daisy_full(daisy_full), 
    .daisy_wr_en(daisy_wr_en), 
    .daisy_din(daisy_din), 
    .daisy_done(daisy_done), 
    
	.dac_cont_mux(dac_cont_mux), 
    
	.mode(mode)

    );

wire [15:0] pipe_data_count;
wire [15:0] wr_addr;
reg [15:0] rd_addr;
reg pipe_wr_en;
reg pipe_rd_en;
reg start;
wire [21:0] din;
wire [21:0] ram_dout;
wire ram_rd_clk;
wire epA4ready;
wire epA4strobe;
//wire fifo_full,fifo_empty;
wire almost_full,almost_empty;
reg wait_epa3read_high;
wire valid_event;

assign valid_event = 1;//(DATA_OUT_BUF[1:0] != 2'b00);// || (row_ctr == 255);

assign din[7:0] = valid_event ? ~col_ctr[7:0] : 0; //rd_addr[7:0]; //
assign din[15:8] = valid_event ? ~row_ctr[7:0] : 0; //rd_addr[15:8]; //frame_cnt
//assign din[16] = frame_done; //col_ctr[6];
//assign din[17] = epA3read; //row_ctr[6];
assign din[21:16] = ~DATA_OUT_BUF[5:0]; //EVENTMODE ? ~DATA_OUT_BUF[1:0] : ~DATA_OUT_BUF[1:0]; //(col_ctr < 32) ? 1 : 
assign wr_addr[15:0] = {row_ctr, col_ctr}; //din[15:0];
//assign wr_addr[7:0] = col_ctr[7:0];
//assign wr_addr[15:8] = row_ctr[7:0];
assign ram_rd_clk = start? okClk : 0;
//assign din[2] = row_ctr[7];
//assign din[1:0] = DATA_OUT_BUF[1:0];
assign ep20wire[21:6] = pipe_data_count[15:0];
//assign ep23wire[15:0] = pipe_data_count[15:0];
//assign ep23wire[0] = fifo_full;
assign epA4ready = 1;//~fifo_empty;
//assign pipe_wr_en = ep12wire[0];
//assign start = ep12wire[0];
assign epA3pipe = (epA3read & ~almost_empty) ? ram_dout : 0; //(wr_addr > rd_addr) ? ram_dout : 0; //

/////////////////////////////
 // FPGA Pipeout FIFO
FIFO_pipeout pipefifo (
  .rst(RESET), // input rst
  .wr_clk(RD_CLK), // input wr_clk
  .rd_clk(okClk), // input rd_clk
  .din(din), // input [21 : 0] din
  .wr_en(valid_event & strobe_done), // input wr_en
  .rd_en(epA3read & ~almost_empty), // input rd_en
  .dout(ram_dout), // output [21 : 0] dout
  .full(fifo_full), // output full
  .almost_full(almost_full), //output almost_full
  .empty(fifo_empty), // output empty
  .almost_empty(almost_empty), // output almost_empty
  .rd_data_count(pipe_data_count) // output [15 : 0] rd_data_count
);/*
framebuffer framebuffer (
  .clka(RD_CLK), // input clka RD_CLK default
  .wea(strobe_done), // input [0 : 0] wea
  .addra(wr_addr), // input [15 : 0] addra
  .dina(din), // input [17 : 0] dina
  .clkb(okClk), // input clkb ram_rd_clk default
  .rstb(RESET), // input rstb
  .enb(frame_done), // input enb
  .addrb(int_addr), // input [15 : 0] addrb
  .doutb(dint) // output [17 : 0] doutb
);

piperam piperam (
  .clka(RD_CLK), // input clka RD_CLK default
  .wea(strobe_done), // input [0 : 0] wea
  .addra(wr_addr), // input [15 : 0] addra
  .dina(din), // input [17 : 0] dina
  .clkb(okClk), // input clkb ram_rd_clk default
  .rstb(RESET), // input rstb
  .enb(epA3read), // input enb
  .addrb(rd_addr), // input [15 : 0] addrb
  .doutb(ram_dout) // output [17 : 0] doutb
);

always @(posedge okClk or posedge epA3read)
begin
	if(epA3read && start==0) begin
		//rd_addr <= rd_addr + 1;
		start <= 1;
	end
	else begin
		if (rd_addr < 4'hFFFF) begin
			rd_addr <= rd_addr + 1;
		end//
		if (rd_addr >= 4'hFFFF) begin
			start <= 0;
		end
	end
end


always @(posedge okClk or posedge epA3read)
begin
	if(epA3read && start==0) begin
		//rd_addr <= rd_addr + 1;
		start <= 1;
	end
	else begin
		if (rd_addr < 4'hFFFF) begin
			rd_addr <= rd_addr + 1;
		end//
		if (rd_addr >= 4'hFFFF) begin
			start <= 0;
		end
	end
end
*/
always @(posedge okClk or posedge RESET)
//always @(posedge ram_rd_clk or posedge RESET)
begin
	if(RESET) begin
		rd_addr <= 0;
	end
	else begin
		if ((epA3read)&&(wr_addr > rd_addr)) begin
			rd_addr <= rd_addr + 1;
		end/*
		else begin
			rd_addr <= 0;
		end*/
	end
end

/*Block Pipeout
always @(posedge okClk or posedge fifo_full)
begin
	if(fifo_full)
		epA4ready = 1;
	else if(fifo_empty)
		epA4ready = 0;
end
*/

//epA3read && ~wait_epa3read_high
always @(posedge okClk or posedge RESET)
begin
	if(RESET)
		wait_epa3read_high <=1'b0;
	else if (epA3read && rd_addr>=4'hFFFF)
		wait_epa3read_high <= 1'b1;
	else if (!epA3read)
		wait_epa3read_high <=1'b0;
end

always @(posedge okClk)  begin 
	if (~fifo_full && strobe_done) begin
		pipe_wr_en <= 1'b1;
	end  
	else begin
	pipe_wr_en <= 1'b0;
	end
end

always @(posedge okClk)  begin  // using okClk to make data reading fast
	if (~fifo_empty && epA3read) begin
		pipe_rd_en<=1'b1;
	end  
	else begin
	  pipe_rd_en<=1'b0;
	end
end	
	
// Instantiate the okHost and connect endpoints.
wire [65*5-1:0]  okEHx;
okHost okHI(
	.okUH(okUH),
	.okHU(okHU),
	.okUHU(okUHU),
	.okAA(okAA),
	.okClk(okClk),
	.okHE(okHE), 
	.okEH(okEH)
);

okWireOR # (.N(5)) wireOR (okEH, okEHx);

okWireIn     wi00(.okHE(okHE),                           .ep_addr(8'h00), .ep_dataout(ep00wire));
okWireIn     wi10(.okHE(okHE),                           .ep_addr(8'h10), .ep_dataout(ep10wire));
okWireIn     wi11(.okHE(okHE),                           .ep_addr(8'h11), .ep_dataout(ep11wire));
okWireIn     wi12(.okHE(okHE),                           .ep_addr(8'h12), .ep_dataout(ep12wire));
okWireIn     wi13(.okHE(okHE),                           .ep_addr(8'h13), .ep_dataout(ep13wire));
okWireOut    wo20(.okHE(okHE), .okEH(okEHx[ 0*65 +: 65 ]), .ep_addr(8'h20), .ep_datain(ep20wire));
okWireOut    wo21(.okHE(okHE), .okEH(okEHx[ 1*65 +: 65 ]), .ep_addr(8'h21), .ep_datain(ep21wire));
okWireOut    wo22(.okHE(okHE), .okEH(okEHx[ 2*65 +: 65 ]), .ep_addr(8'h22), .ep_datain(ep22wire));
okWireOut    wo23(.okHE(okHE), .okEH(okEHx[ 3*65 +: 65 ]), .ep_addr(8'h23), .ep_datain(ep23wire));

okPipeOut pipeOutA3 (.okHE(okHE), .okEH(okEHx[ 4*65 +: 65 ]),.ep_addr(8'ha3), .ep_datain(epA3pipe), .ep_read(epA3read));
//okBTPipeOut pipeOutA4 (.okHE(okHE), .okEH(okEHx[ 5*65 +: 65 ]),.ep_addr(8'ha4), .ep_datain(epA4pipe), .ep_read(epA4read), .ep_blockstrobe(epA4strobe), .ep_ready(epA4ready));

okTriggerIn  ti40(.okHE(okHE),                           .ep_addr(8'h40), .ep_clk(okClk), .ep_trigger(ep40wire));
okTriggerIn  ti41(.okHE(okHE),                           .ep_addr(8'h41), .ep_clk(CLK_DAC), .ep_trigger(ep41wire));
	 
//testbench
endmodule


