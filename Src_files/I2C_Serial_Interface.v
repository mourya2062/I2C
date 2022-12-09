// I2C Master
// I2C Master Verilog Implementation (I2C_Master.v)
// Mourya Chandra

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: DE1-SoC Board

//-----------------------------------------------------------------------------

module I2C_Serial_Interface (clk, reset, SLAVE_ADDRESS_r,data_from_tx_fifo,tx_fifo_rd_en,control_reg,data_to_rx_fifo,rx_fifo_wr_en,i2c_sda,i2c_scl,ref_clk_test, debug_port);

    	// Clock, reset
    	input   clk, reset;
	 
    	//SLAVE ADDRESS
	input [6:0]  	SLAVE_ADDRESS_r				;
	 
	 //TX_RX_FIFO INTERFACE
	input [7:0] 	data_from_tx_fifo			;
	output 		tx_fifo_rd_en				;
	output reg [7:0]data_to_rx_fifo				;
	output		rx_fifo_wr_en				;
	 
	 //I2C control reg
	 input [31:0] 	control_reg				;
    
    	//I2C BUS 
    	inout 	   	i2c_sda					;
    	output		i2c_scl					;
    	output 		ref_clk_test				;
    
    	// debug interface
    	output [47:0] debug_port				;
    
    // FSM  states
    parameter IDLE	      		 			= 4'b0000		;
	 parameter I2C_START							= 4'b0001		;
    parameter SEND_SLAVE_REG_ADDR_DATA 	= 4'b0010		;
    parameter WAIT_STATE			 			= 4'b0011		;
    parameter BUS_RELEASE			 			= 4'b0100		;
    parameter ACK_CHECK		        			= 4'b0101		; 
    parameter RD_DATA		        			= 4'b0110		;
	 parameter RD_ACK_NACK						= 4'b0111		;
    parameter STOP		       				= 4'b1000		;
    
    //Internal signals 
	 reg [3:0] 	  	state												;
	 reg	   	  	ref_clk											;//200 KHz reference enable 
	 reg [7:0] 	  	ref_clk_counter								;
	 wire 		  	sda_in											;
	 reg 			  	release_bus										;
	 reg          	sda_change_flag								;
	 reg 			  	i2c_scl_cpy										;
	 reg 			  	sda_out											;
	 wire	[47:0]	debug_port_ELA									;
	 
	 //cotrol reg related signlas 
	 reg 				R_W_N												;
	 reg 	[5:0]		BYTE_COUNT										;
	 reg 				USE_REGISTER									;
	 reg	[7:0] 	REGISTER											;
	 reg 				USE_REPEATED_START							;
	 reg				START												;
	 reg 				TEST_OUT											;
	 reg	[6:0] 	SLAVE_ADDRESS									;
	 reg 				DEBUG_OUT										;
	 reg	[7:0] 	RX_DATA											;
	 
	 //registers to latch at the rising edge of the start 
	 reg 				R_W_N_r											;
	 reg 	[5:0]		BYTE_COUNT_r									;
	 reg 				USE_REGISTER_r									;
	 reg	[7:0] 	REGISTER_r										;
	 reg 				USE_REPEATED_START_r							;
	 
	 //registers used in FSM 
	 reg 	[7:0] 	Address_reg_data_I2C							;
	 reg  [2:0] 	shift_counter 									;
	 
	 //flags to use in FSM 
	 reg 	[1:0]	Addr_data_sel										;//selects between the slave address and register address and data 
	 reg   		tx_fifo_rd_en_sig									;//this signal is used to latch the data from FIFO 
	 reg			tx_fifo_rd_en_sig_del							;
	 reg 			start_fsm											;
	 reg			clr_start_fsm										;
	 reg 	[2:0] dummy_counter										;
	 reg			ack_error											;
	 reg			Rd_data_flag										;
	 reg 			rx_fifo_wr_en_sig									;
	 reg			rx_fifo_wr_en_sig_del							;
	 reg 			BUSY													;
	
	
	 //generating 200 KHz enable from 50KHz clock 
	always @ (posedge(clk))
	begin
		if (ref_clk_counter < 125)
		begin
			ref_clk_counter <= ref_clk_counter + 1				;
			ref_clk <= 1'b0											;
		end
		else
		begin
			ref_clk_counter <= 0										;
			ref_clk <= 1												;
		end
	end
	
	assign ref_clk_test	=	ref_clk								;
	
	//SCL and SDA control code 
	assign i2c_scl 	= (i2c_scl_cpy ==1'b1)?1'bz :1'b0	; 
	assign sda_in  = i2c_sda										;
	assign i2c_sda = (sda_out == 1'b1)?1'bz: 1'b0			;
	
	always @(posedge clk)
	begin
		START		<=	control_reg[17]								;
		if(clr_start_fsm == 1'b1)	
			start_fsm	<= 1'b0 										;
		else if((control_reg[17] && ~START) == 1'b1)	
			start_fsm  <=  1'b1										;
	end 
	
	 //extracting data from control reg 
	 always @(posedge clk)
	 begin
		if(start_fsm == 1'b1)
		begin
			R_W_N						<=	control_reg[0]				;
			BYTE_COUNT				<=	control_reg[6:1]			;
			USE_REGISTER			<=	control_reg[7]				;
			REGISTER					<=	control_reg[15:8]			;
			USE_REPEATED_START	<=	control_reg[16]			;
			TEST_OUT					<=	control_reg[18]			;
			SLAVE_ADDRESS			<=	control_reg[25:19]		;
			DEBUG_OUT				<=	control_reg[31:26]		;
		end 
		else if(start_fsm == 1'b0)
		begin
			R_W_N						<=	1'b0							;
			BYTE_COUNT				<=	5'd0							;
			USE_REGISTER			<=	1'd0							;
			REGISTER					<=	8'd0							;
			USE_REPEATED_START	<=	1'b0							;
			TEST_OUT					<=	1'b0							;
			SLAVE_ADDRESS			<=	7'd0							;
			DEBUG_OUT				<=	13'd0							;
		end 
	 end
	 
	 	
	 //generating tx_fifo_rd_en
	 always @(posedge clk)
	 begin
		tx_fifo_rd_en_sig_del	<=	tx_fifo_rd_en_sig			;
	 end 
	 
	 assign tx_fifo_rd_en		=	tx_fifo_rd_en_sig & (~tx_fifo_rd_en_sig_del)	;
	 
	 //MUX for address signal
	 always @(posedge clk)
	 begin
		if(Addr_data_sel == 2'b00)											//slave address with ~W
			Address_reg_data_I2C	<=	{SLAVE_ADDRESS_r,1'b0}		;
		else if(Addr_data_sel == 2'b01)								  	// slave address with R
			Address_reg_data_I2C	<=	{SLAVE_ADDRESS_r,1'b1}		;
		else if(Addr_data_sel == 2'b10)
			Address_reg_data_I2C	<=	REGISTER_r						;	//Register address 
		else 
			Address_reg_data_I2C	<=	data_from_tx_fifo				;	//Data 	
	 end 
	 
	 always @(posedge clk)
	 begin
		if(start_fsm == 1'b1)
			BUSY	<=	1'b1	;
		else if(state == STOP && BYTE_COUNT_r == 6'd0)
			BUSY	<=	1'b0	;
	 end 
			
			
   always @(posedge clk)
	begin
		if(reset==1'b1)
		begin
			i2c_scl_cpy					<= 1'b1								;
			sda_out						<= 1'b1								;
			R_W_N_r						<=	1'b0								;
			BYTE_COUNT_r				<=	5'd0								;
			USE_REGISTER_r				<=	1'd0								;
			REGISTER_r					<=	8'd0								;
			USE_REPEATED_START_r		<=	1'b0								;
			//i2c sda related signals
			clr_start_fsm			<=	1'b0									;
			sda_change_flag		<=	1'b1									;
			sda_out					<= 1'b0									;
			Addr_data_sel			<=	2'b00									; 
			tx_fifo_rd_en_sig		<=	1'b0									;
			shift_counter			<=	3'b111								;
			ack_error				<=	1'b0									;
			rx_fifo_wr_en_sig		<=	1'b0									;
			state						<=	IDLE									;	
		end 
		else if(ref_clk == 1'b1 )
		begin
			case(state)
				IDLE:
				begin
					i2c_scl_cpy		<= 1'b1											;	//copy of i2c_scl  and it is high in idle state 
					sda_out			<= 1'b1											;	//release data bus 
					ack_error		<=	1'b0											;
					if(start_fsm == 1'b1 || Rd_data_flag == 1'b1)				//registering all the control signals at start_fsm phase but not at read stage 
					begin
						if(Rd_data_flag == 1'b1)
						begin
							state				<=	I2C_START							;
						end
						else
						begin
							R_W_N_r					<= R_W_N							;
							BYTE_COUNT_r			<= BYTE_COUNT					;
							USE_REGISTER_r			<= USE_REGISTER				;	
							REGISTER_r				<= REGISTER						;
							USE_REPEATED_START_r	<= USE_REPEATED_START		;
							state						<=	I2C_START					;	
						end
					end 
					else
						state 					<=	IDLE								;
				end
				
				I2C_START:
				begin
					sda_change_flag		<=	1'b1									;//used this signal to toggle the scl(fix me actually this signal is not required)  
					sda_out					<= 1'b0									;//making sda to zero as a start condition
					shift_counter			<=	3'b111								;//counter for serialization 
					if(Rd_data_flag == 1'b1)
						Addr_data_sel			<=	2'b01								;//selecting slave address with read bit 
					else
					begin
						clr_start_fsm			<=	1'b1								;
						Addr_data_sel			<=	2'b00								;//selecting slave address with write bit 
						if(R_W_N_r == 1'b0)
							tx_fifo_rd_en_sig		<=	1'b1							;//send a read enable to tx fifo for data 
					end 
					
					state	<=	SEND_SLAVE_REG_ADDR_DATA							;
				end
						
				SEND_SLAVE_REG_ADDR_DATA:
				begin
						tx_fifo_rd_en_sig	<=	1'b0									;	//take care about this signal at the time of developing for Byte count gretaer than 1 
						if(sda_change_flag == 1'b1)
						begin
							shift_counter	<= shift_counter - 1'b1				;
							i2c_scl_cpy		<=	1'b0									;
							sda_out			<=	Address_reg_data_I2C[shift_counter]	;
							if(shift_counter == 3'b000)
							begin
								state				<=	WAIT_STATE						;
								if(Addr_data_sel == 2'b11)
									BYTE_COUNT_r		<=	BYTE_COUNT_r -1 		;
							end 
							else
								state				<=	SEND_SLAVE_REG_ADDR_DATA	;
								
							sda_change_flag	= 1'b0	;
						end
						else 
						begin
							sda_change_flag	<=	1'b1						;
							i2c_scl_cpy		<=	1'b1						;
							state			<=	SEND_SLAVE_REG_ADDR_DATA	;
						end
				end 
				
				WAIT_STATE:															//This state is introduced to let the slave to read LSB bit 
				begin
					i2c_scl_cpy		<=	1'b1			;
					state			<=	BUS_RELEASE		;
				end 
						
				BUS_RELEASE:														//This state is used to relase the bus for reading the acknowledge ment 
				begin
						i2c_scl_cpy		<=	1'b0		;
						sda_out			<=	1'b1		;							
						state			<=	ACK_CHECK	;
						if(Addr_data_sel == 2'b11 && BYTE_COUNT_r > 6'd0)
						begin
							tx_fifo_rd_en_sig	<=	1'b1 	;					 	//Taking one more sample from fifo if the bytecount is greater than zero 
						end 
				end 
				
				//The state of change from ACK_CHECK to the other state purely depends on what type of data ACK it is checking 
				ACK_CHECK:
				begin
					i2c_scl_cpy		<=	1'b1												;
					if(sda_in == 1'b0)
					begin
						sda_change_flag	<=	1'b1										;
						if(Addr_data_sel 	== 2'b00 && USE_REGISTER_r == 1'b1 )//After writing the address, if register is used then register address should send 
						begin
							Addr_data_sel	<=  2'b10									;//selecting the register data and going back to the send data state
							state				<=	SEND_SLAVE_REG_ADDR_DATA			;
						end 
						else if(Addr_data_sel != 2'b01 && R_W_N_r == 1'b0 && BYTE_COUNT_r > 6'd0)	//if the state change from address or use reg or data and byte count > 0 then send the remaining data 
						begin
							Addr_data_sel	<=  2'b11									;
							state				<=	SEND_SLAVE_REG_ADDR_DATA			;
						end 
						else if(Addr_data_sel != 2'b01 && R_W_N_r == 1'b0 && BYTE_COUNT_r == 6'd0)	//If there is nothing left to send go to the STOP state 
						begin
							sda_out			<=	1'b0										;
							state				<=	STOP										;
						end 
						else if(R_W_N_r == 1'b1 && Rd_data_flag == 1'b0)		//If it is read state then go to the stop and restart everythig using this Rd_data_flag  
						begin
							Rd_data_flag	<=	1'b1										;
							sda_out			<=	1'b0									;
							state				<=	STOP									;
						end
						else
						begin																	//After making all the arrangments to read then go to the READ STATE 
							Rd_data_flag	<=	1'b0										;
							sda_out			<=	1'b1										;
							state				<=	RD_DATA									;
						end 
					end 
					else
					begin
						ack_error			<=	1'b1										;
						state					<=	STOP										;
					end 
				end 
				
				RD_DATA:
				begin
					if(i2c_scl_cpy 		== 1'b1)
					begin
						sda_out		<=	1'b1												;
						i2c_scl_cpy	<=	1'b0												;
						state			<=	RD_DATA											;
					end
					else
					begin
						i2c_scl_cpy		<=	1'b1											;
						shift_counter	<= shift_counter - 1'b1						;
						RX_DATA[shift_counter]	<=	sda_in							;
							if(shift_counter == 3'b000)
							begin
								BYTE_COUNT_r	<=	BYTE_COUNT_r - 1'b1				;
								state				<=	RD_ACK_NACK							;
							end 
					end
				end 	
				
				RD_ACK_NACK:
				begin
					data_to_rx_fifo	<=	RX_DATA										;
					rx_fifo_wr_en_sig	<=	1'b1											;//write the sample to RX fifo soon after it was read 
					if(i2c_scl_cpy	==	1'b1)
					begin
						i2c_scl_cpy		<=	1'b0											;
						if(BYTE_COUNT_r > 6'd0)												
						begin
							sda_out		<=	1'b0											;//sending ACK to the slave for another byte 
							state			<=	RD_ACK_NACK									;
						end 
						else
						begin
							sda_out	<=	1'b1												;//sending NACK if there is nothing to read 
							state		<=	RD_ACK_NACK										;
						end
					end
					else	
					begin
						rx_fifo_wr_en_sig	<=	1'b0										;
						if(BYTE_COUNT_r > 6'd0)
						begin
							i2c_scl_cpy <= 1'b1											;
							state			<=	RD_DATA										;//Go back to the read state if more data is required after sending ACK 
						end 
						else
						begin
							i2c_scl_cpy <= 1'b1											;
							state			<=	STOP											;//Go to the STOP state if there is nothing to read 
						end
					end 
				end
					
				STOP:
				begin
					if(i2c_scl_cpy	== 1'b1)
					begin
						sda_out		 	<=	1'b0											;
						i2c_scl_cpy		<=	1'b0											;
						state 			<=	STOP											;
					end 
					else
					begin
						if(USE_REPEATED_START_r == 1'b1 && Rd_data_flag == 1'b1)
						begin 	
							sda_out		 	<=	1'b1											;
							i2c_scl_cpy		<=	1'b1											;
							clr_start_fsm 	<=	1'b0											;
							state				<=	I2C_START									;
						end
						else
						begin
							sda_out		 	<=	1'b0											;
							i2c_scl_cpy		<=	1'b1											;
							clr_start_fsm 	<=	1'b0											;
							state				<=	IDLE											;
						end 
					end 
				end 
				
			endcase
		end
	end 
	
	//RX FIFO WRITING CODE 
	always @(posedge clk)
	begin
		rx_fifo_wr_en_sig_del	<=	rx_fifo_wr_en_sig	;
	end 
	
	//assign data_to_rx_fifo = RX_DATA													;
	assign rx_fifo_wr_en	  = rx_fifo_wr_en_sig & ~rx_fifo_wr_en_sig_del	;
	
	
	I2C_Master_ELA I2C_serial_ELA_inst(
		.acq_clk(clk)									,     //    acq_clk.clk
		.acq_data_in(debug_port_ELA)				,    	//    tap.acq_data_in
		.acq_trigger_in(debug_port_ELA)			, 		//    acq_trigger_in
		.trigger_in(1'b1)      								// 	trigger_in.trigger_in
	);

	assign debug_port_ELA = {4'd0,rx_fifo_wr_en,rx_fifo_wr_en_sig_del,BUSY,rx_fifo_wr_en_sig,data_to_rx_fifo,Rd_data_flag,data_from_tx_fifo,tx_fifo_rd_en_sig_del,tx_fifo_rd_en_sig,BYTE_COUNT_r,ack_error,start_fsm,sda_in,ref_clk,sda_out,i2c_scl_cpy,Addr_data_sel,state,shift_counter}	;
	
	assign debug_port	= debug_port_ELA	; 
	
	
endmodule
