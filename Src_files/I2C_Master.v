// I2C Master
// I2C Master Verilog Implementation (I2C_Master.v)
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: DE1-SoC Board

//-----------------------------------------------------------------------------

module I2C_Master (clk, reset, address, byteenable, chipselect, writedata, readdata, write, read,i2c_sda,i2c_scl,ref_clk_test, debug_port);

    // Clock, reset
    input   clk, reset;

    // Avalon MM interface 
    input             	read, write, chipselect	;
    input 	[1:0] 	address			;
    input 	[3:0]  	byteenable		;
    input 	[31:0] 	writedata		;
    output reg 	[31:0] 	readdata		;
    
    //I2C BUS 
    inout 	       	i2c_sda			;
    output		i2c_scl			;
    output 		ref_clk_test		;
    
    // debug interface
    output   	[14:0]  debug_port		;

    
    // register map
    // ofs  fn
    //   0  address 	(r/w)
    //   4  data 	(r/w)
    //   8  status 	(r/w1c)
    //  12  control 	(r/w)
    
    // register numbers
    parameter ADDRESS           	= 2'b00	;
    parameter DATA              	= 2'b01	;
    parameter STATUS            	= 2'b10	;
    parameter CONTROL       		= 2'b11	;
    
    //Internal signals 
    reg [31:0] slave_address					;
	 
    reg  [31:0] tx_fifo_data_in				;
    wire [31:0] tx_fifo_data_out				;
	 reg			tx_fifo_wr_req					;
	 wire       tx_fifo_rd_req					;
	 wire			tx_fifo_full					;
	 wire			tx_fifo_empty					;
	 wire			tx_fifo_overflow				;
	 wire			clr_tx_fifo_overflow			;
	 wire [5:0] tx_fifo_wr_index				;
	 wire [5:0] tx_fifo_rd_index				;
	 
    wire [31:0]rx_fifo_data_in				;
	 wire			rx_fifo_wr_req					;
	 wire       rx_fifo_rd_req					;
    reg [31:0] rx_fifo_data_out				;
	 wire			rx_fifo_full					;
	 wire			rx_fifo_empty					;
	 wire [5:0] rx_fifo_wr_index				;
	 wire [5:0] rx_fifo_rd_index				;
	 wire			rx_fifo_overflow				;
	 wire			clr_rx_fifo_overflow			;
	 wire[31:0] rx_tx_fifo_status				;
    reg [31:0] rx_tx_fifo_status_service	;	//used this signal when STATUS register write from HPS
    reg [31:0] control_reg						;
	 wire 		BUSY								;
	 wire			ACK_ERROR						;
	 
	 reg 			chipselect_del									;
	 wire			chipselect_rsg									;
	 wire    	[47:0]  debug_port_serial_interface		;
    
    // read register
    always @ (*)
    begin
        if (read && chipselect)
            case (address)
                ADDRESS: 
                    readdata = slave_address			;
                DATA:
                    readdata = rx_fifo_data_out		;
                STATUS: 
                    readdata = rx_tx_fifo_status	;
                CONTROL: 
                    readdata = control_reg			;
            endcase
        else
            readdata = 32'b0;
    end 

    // write register
    always @ (posedge clk)
    begin
        if (reset)
        begin
    	    slave_address 				<= 32'h0			;
    	    tx_fifo_data_in	  			<= 32'h0			;
    	    control_reg	  				<= 32'h0			;	
			 tx_fifo_wr_req				<=	1'b0			;//This ensures write_req will be for one clock cycle 	
			 rx_tx_fifo_status_service	<=	32'h0			;//This ensures all  the clear requests will be for one clock cycle 
				
        end
        else
        begin
            if (write && chipselect)
            begin
                case (address)
                    ADDRESS: 
                        slave_address <= writedata						;
                    DATA:
						  begin
                        tx_fifo_data_in 	<= writedata				;
								tx_fifo_wr_req		<=	1'b1						;
						  end 
                    STATUS: 
                        rx_tx_fifo_status_service <= writedata		;
                    CONTROL: 
                        control_reg <= writedata						;
                endcase
            end
				else
				begin
					tx_fifo_wr_req					<=	1'b0						;//This ensures write_req will be for one clock cycle 	
					rx_tx_fifo_status_service	<=	32'h00000000			;//This ensures all  the clear requests will be for one clock cycle 
				end 
					
        end
    end
	 
	 //generating RX fifo read request 
	 //As the READ bus cycle will take two clock cycles ,finding the rising edge for chipselect ensures fifo rd_req for 1 clk cycle
	 //There will be one clock cyle delay between the fifo read req and fifo output.
	 //As the read bus cycle is alive for two clock cycles,there will be no issue 
	 always @(posedge clk)
	 begin
		chipselect_del	<=	chipselect	;
	 end 
	 
		assign chipselect_rsg 			= chipselect & (~chipselect_del)	;	 
		assign rx_fifo_rd_req 			= ((chipselect_rsg == 1'b1)&& (read == 1'b1) && (address == DATA))?1'b1 :1'b0						; 
		assign clr_tx_fifo_overflow	=	rx_tx_fifo_status_service[3]	;
		assign clr_rx_fifo_overflow	=	rx_tx_fifo_status_service[0]	;
		
Sync_FIFO TX_FIFO_inst
(
	.clock(clk) 												,	// input  clock_sig
	.reset(reset) 												,	// input  reset_sig
	.wr_req(tx_fifo_wr_req) 								,	// input  wr_req_sig
	.wr_data(tx_fifo_data_in[7:0])						,	// input [data_width-1:0] wr_data_sig
	.rd_req(tx_fifo_rd_req) 								,	// input  rd_req_sig
	.rd_data(tx_fifo_data_out[7:0]) 						,	// output [data_width-1:0] rd_data_sig
	.full(tx_fifo_full) 										,	// output  full_sig
	.empty(tx_fifo_empty) 									,	// output  empty_sig
	.overflow(tx_fifo_overflow) 							,	// output  overflow_sig
	.clear_overflow_request(clr_tx_fifo_overflow)	,	// input  clear_overflow_request_sig
	.wr_index(tx_fifo_wr_index) 							,	// output [fifo_depth_bits-1:0] wr_index_sig
	.rd_index(tx_fifo_rd_index) 								// output [fifo_depth_bits-1:0] rd_index_sig
);

defparam TX_FIFO_inst.data_width = 8;
defparam TX_FIFO_inst.fifo_depth = 64;
defparam TX_FIFO_inst.fifo_depth_bits = 6;

		
Sync_FIFO RX_FIFO_inst
(
	.clock(clk) 												,	// input  clock_sig
	.reset(reset) 												,	// input  reset_sig
	.wr_req(rx_fifo_wr_req) 								,	// input  wr_req_sig
	.wr_data(rx_fifo_data_in[7:0])						,	// input [data_width-1:0] wr_data_sig
	.rd_req(rx_fifo_rd_req) 								,	// input  rd_req_sig
	.rd_data(rx_fifo_data_out[7:0]) 						,	// output [data_width-1:0] rd_data_sig
	.full(rx_fifo_full) 										,	// output  full_sig
	.empty(rx_fifo_empty) 									,	// output  empty_sig
	.overflow(rx_fifo_overflow) 							,	// output  overflow_sig
	.clear_overflow_request(clr_rx_fifo_overflow)	,	// input  clear_overflow_request_sig
	.wr_index(rx_fifo_wr_index) 							,	// output [fifo_depth_bits-1:0] wr_index_sig
	.rd_index(rx_fifo_rd_index) 								// output [fifo_depth_bits-1:0] rd_index_sig
);

defparam TX_FIFO_inst.data_width = 8;
defparam TX_FIFO_inst.fifo_depth = 64;
defparam TX_FIFO_inst.fifo_depth_bits = 6;

I2C_Serial_Interface I2C_Serial_Interface_inst
(
	.clk(clk) 													,	// input  clk_sig
	.reset(reset) 												,	// input  reset_sig
	.SLAVE_ADDRESS_r(slave_address[6:0])				,
	.data_from_tx_fifo(tx_fifo_data_out[7:0]) 		,	// input [7:0] data_from_tx_fifo_sig
	.tx_fifo_rd_en(tx_fifo_rd_req) 						,	// output  tx_fifo_rd_en_sig
	.control_reg(control_reg) 								,	// input [31:0] control_reg_sig
	.data_to_rx_fifo(rx_fifo_data_in[7:0]) 			,	// output  data_to_rx_fifo_sig
	.rx_fifo_wr_en(rx_fifo_wr_req) 						,	// output  rx_fifo_wr_en_sig
	.i2c_sda(i2c_sda) 										,	// inout  i2c_sda_sig
	.i2c_scl(i2c_scl) 										,	// output  i2c_scl_sig
	.ref_clk_test(ref_clk_test) 							,	// output  ref_clk_test_sig
	.debug_port(debug_port_serial_interface) 				// output [14:0] debug_port_sig
);

assign ACK_ERROR 	= debug_port_serial_interface[15]	;
assign BUSY			= debug_port_serial_interface[41]	;

assign rx_tx_fifo_status  = {rx_fifo_rd_index,rx_fifo_wr_index,tx_fifo_rd_index,tx_fifo_wr_index,BUSY,ACK_ERROR,tx_fifo_empty,tx_fifo_full,tx_fifo_overflow,rx_fifo_empty,rx_fifo_full,rx_fifo_overflow}	;
//assign debug_port = {write,read,address,chipselect,writedata[4:0],readdata[4:0]}	;
assign debug_port = {rx_fifo_empty,rx_fifo_full,tx_fifo_full,rx_fifo_rd_index[2:0],rx_fifo_wr_index[2:0],tx_fifo_rd_index[2:0],tx_fifo_wr_index[2:0]}	;
 
endmodule
