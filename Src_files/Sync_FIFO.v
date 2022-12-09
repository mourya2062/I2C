module Sync_FIFO
#(
	parameter data_width 		= 8		,	//specified as W in assignemnt 
	parameter fifo_depth 		= 64		,	//specified as N in assignment 
	parameter fifo_depth_bits	= 6			//work on this to automate based on fifo_depth 
)
	(
		input  							clock							,
		input  							reset            			,
		input  							wr_req 						,
		input  [data_width-1:0] 	wr_data						,
		input  							rd_req            		,
		output reg [data_width-1:0]rd_data						,
		output  							full            			,
		output  							empty            			,
		output  reg						overflow            		,
		input  							clear_overflow_request  ,
		output [fifo_depth_bits-1:0]wr_index   				,
		output [fifo_depth_bits-1:0]rd_index   
	);
	
	reg [data_width-1:0] memory [0:fifo_depth-1]					;
	reg [fifo_depth_bits:0] 	wr_index_counter   				; //+1 bit counters to make fifo status logic easier
	reg [fifo_depth_bits:0]  	rd_index_counter 					;
	wire full_sig  														;
	wire empty_sig 														;

	////////////////////
	//FIFO write logic//
	////////////////////
	
	always @(posedge clock)
	begin
		if(reset == 1'b1) 
		begin
			wr_index_counter	<= {fifo_depth_bits{1'b0}}								;
		end
		else if(wr_req == 1'b1 && full_sig == 1'b0) 
		begin
			memory[wr_index_counter[fifo_depth_bits-1:0]]	<=	wr_data					;
				//calculating next fifo write address .write address size is +1 to the specified bit size to detect the fifo full(MSB bit) and
				//clearing it when there is a read request (simultaneous  read and write request)
			if((wr_index_counter[fifo_depth_bits] == 1'b1) && (rd_index_counter[fifo_depth_bits] == 1'b1) && (rd_req == 1'b1))
				wr_index_counter 	<= {1'b0,wr_index_counter[fifo_depth_bits-1:0]+1}	;
			else
				wr_index_counter 	<=  wr_index_counter + 1'b1							;
		end 
			   //clearing fifo full indicator when there is a read request (exclusive  read and write request)
		else if((wr_index_counter[fifo_depth_bits] == 1'b1) && (rd_index_counter[fifo_depth_bits] == 1'b1) && (rd_req == 1'b1))
			wr_index_counter <= {1'b0,wr_index_counter[fifo_depth_bits-1:0]}			;
	end 
	
	
	////////////////////
	//FIFO read logic//
	////////////////////	
	
	always @(posedge clock)
	begin
		if(reset == 1'b1)
		begin
			rd_index_counter	<= {fifo_depth_bits{1'b0}}											;
			rd_data				<=	{data_width{1'b0}}												;
		end	
		else if(rd_req == 1'b1 && empty_sig != 1'b1) 
		begin
			rd_data				<=	memory[rd_index_counter[fifo_depth_bits-1:0]]			;
			if(rd_index_counter[fifo_depth_bits] == 1'b1)
				rd_index_counter 	<=  {1'b0,rd_index_counter[fifo_depth_bits-1:0]+1}		;//incrementing the next fifo read address
			else
				rd_index_counter 	<=  rd_index_counter + 1'b1									;
		end 
	end 
	
	
	///////////////////////
	//FIFO status signals//
	///////////////////////
	
	//assign full_sig 	= ((wr_index_counter[fifo_depth_bits] == 1)&& (rd_index_counter == 0))?1'b1 :1'b0		; 
	//assign empty_sig 	= ((wr_index_counter == 0)&& (rd_index_counter == 0))?1'b1 :1'b0						; 
	assign full_sig 	= ((wr_index_counter[fifo_depth_bits-1:0] == rd_index_counter)&& (wr_index_counter[fifo_depth_bits] == 1))?1'b1 :1'b0						; 
	assign empty_sig 	= ((wr_index_counter[fifo_depth_bits-1:0] == rd_index_counter)&& (wr_index_counter[fifo_depth_bits] != 1))?1'b1 :1'b0						; 
	assign wr_index 	= wr_index_counter[fifo_depth_bits-1:0]																		;
	assign rd_index 	= rd_index_counter																		;
	assign full 		= full_sig																				;
	assign empty 		= empty_sig 																			;
	
	always @(posedge clock)
	begin
		if(reset == 1'b1)  
			overflow <=	 1'b0 	;
		else if(clear_overflow_request == 1'b1)
			overflow <=	 1'b0 	;
		else if(full_sig == 1'b1 && wr_req == 1'b1) 
			overflow <=  1'b1	;
	end 
	
endmodule 
	
