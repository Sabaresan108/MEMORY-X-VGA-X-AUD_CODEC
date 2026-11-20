//0 to 100 RW code for SDRAM
module SDRAM_RW (input         CLOCK_50,
						 input  [17:0] SW,
						 output [17:0] LEDR,
						 output [8:0]  LEDG,
						// SDRAM (DE2-115 pin names)
						 output [12:0] DRAM_ADDR,
						 output [1:0]  DRAM_BA,
						 output        DRAM_CAS_N,
						 output        DRAM_CKE,
						 output        DRAM_CLK,
						 output        DRAM_CS_N,
						 inout  [31:0] DRAM_DQ,
						 output [3:0]  DRAM_DQM,
						 output        DRAM_RAS_N,
						 output        DRAM_WE_N);

// Commands {CS_N, RAS_N, CAS_N, WE_N} 
localparam CMD_NOP   = 4'b0111;
localparam CMD_ACT   = 4'b0011;
localparam CMD_READ  = 4'b0101;
localparam CMD_WRITE = 4'b0100;
localparam CMD_PRE   = 4'b0010;
localparam CMD_REF   = 4'b0001;
localparam CMD_MRS   = 4'b0000;

// Mode register (13 bits): A12..A10=000, A9=0, A8..A7=00, A6..A4=010 (CAS latency 2), A3=0 sequential, A2..A0=000 (burst length 1)
localparam [12:0] MODE_REG = 13'b000_0_00_010_0_000;

//  States
localparam WAIT       = 4'd0;
localparam PRECHARGE  = 4'd1;
localparam REFRESH_1  = 4'd2;
localparam REFRESH_2  = 4'd3;
localparam MODE_REG_S = 4'd4;
localparam IDLE       = 4'd5;
localparam READ_WRITE = 4'd6;
localparam WRITE    	 = 4'd7;
localparam CAPTURE    = 4'd8;

	 
wire [12:0] sw_row  = SW[15:3];
wire [1:0]  sw_bank = SW[2:1];
wire rst = SW[0] ;

assign DRAM_CLK = ~CLOCK_50;

// registers
reg [3:0]  state, next_state;
reg [15:0] count;
reg [3:0]  cmd;
reg [12:0] addr;
reg [1:0]  bank;
reg        oe;
reg [31:0] wdata;
reg [15:0] rdata;
reg [15:0] refresh_count;

reg        wr_done;
reg [12:0] wr_row;
reg [1:0]  wr_idx;
wire [1:0] wr_bank = ~wr_idx;      // banks are written in the order 3, 2, 1, 0
reg        is_write;
reg [12:0] req_row;
reg [1:0]  req_bank;
reg        match;

assign DRAM_CKE  = 1'b1;
assign DRAM_DQM  = 4'b0000;
assign {DRAM_CS_N, DRAM_RAS_N, DRAM_CAS_N, DRAM_WE_N} = cmd;
assign DRAM_ADDR = addr;
assign DRAM_BA   = bank;
assign DRAM_DQ   = oe ? wdata : 32'bz;

// LEDs
assign LEDG = {2'b00, rdata[6:0]};
//             17:16     15:12   11     10:0
assign LEDR = {req_bank, 4'b0000, match, 11'd0};

// controller
    always @(posedge CLOCK_50)
    begin
			 if (~rst)
			 begin
					 state       	<= WAIT;
					 count         <= 16'd10000;      // 200 us power-up delay
					 next_state    <= PRECHARGE;
					 cmd         	<= CMD_NOP;
					 addr        	<= 13'd0;
					 bank          <= 2'd0;
					 oe          	<= 1'b0;
					 wdata       	<= 32'd0;
					 rdata       	<= 16'd0;
					 refresh_count <= 16'd0;
					 wr_done     	<= 1'b0;
					 wr_row      	<= 13'd0;
					 wr_idx      	<= 2'd0;
					 is_write    	<= 1'b0;
					 req_row     	<= 13'd0;
					 req_bank    	<= 2'd0;
					 match       	<= 1'b0;
        end
        else
        begin
					cmd <= CMD_NOP;

					if (refresh_count != 16'hFFFF)
					begin
							refresh_count <= refresh_count + 16'd1;
					end
					
					case (state)

						 WAIT:
						 begin
								 if (count == 0)
								 begin
										 state <= next_state;
								 end
								 
								 else
								 begin
										count <= count - 16'd1;
								 end		
						 end

						 PRECHARGE:
						 begin
								 cmd   		<= CMD_PRE;
								 addr  		<= 13'b0010000000000;   // A10=1 : precharge all banks
								 state 		<= WAIT;
								 count   	<= 16'd2;
								 next_state <= REFRESH_1;
						 end

						 REFRESH_1:
						 begin
								 cmd   		 <= CMD_REF;
							    state 		 <= WAIT;
								 count   	 <= 16'd4;
								 next_state  <= REFRESH_2;
						 end

						 REFRESH_2:
						 begin
								 cmd   		 <= CMD_REF;
								 state 		 <= WAIT;
								 count   	 <= 16'd4;
								 next_state  <= MODE_REG_S;
						 end

						 MODE_REG_S:
						 begin
								 cmd       	 <= CMD_MRS;
								 bank        <= 2'b00;
							    addr      	 <= MODE_REG;
							    state     	 <= WAIT;
							    count       <= 16'd2;
							    next_state  <= IDLE;
						 end

						 IDLE:
						 begin
								 if (refresh_count >= 16'd360)         // 7.2 us
								 begin
										 cmd         	<= CMD_REF;
										 refresh_count <= 16'd0;
										 state       	<= WAIT;
										 count         <= 16'd4;
										 next_state 	<= IDLE;
								 end
								 
								 else if (!wr_done)
								 begin
										 cmd      	<= CMD_ACT;
										 bank     	<= wr_bank;
										 addr     	<= wr_row;
										 is_write 	<= 1'b1;
										 state    	<= WAIT;
										 count      <= 16'd1;
										 next_state <= READ_WRITE;               // tRCD
								end
								
								else
								begin
										cmd        <= CMD_ACT;
										bank       <= sw_bank;
										addr       <= sw_row;
										req_row    <= sw_row;
										req_bank   <= sw_bank;
										oe         <= 1'b1;                           
										wdata      <= {16'd0, ~{6'd0, sw_bank, 1'b0, sw_row[6:0]}}; 
										is_write   <= 1'b0;
										state      <= WAIT;
										count      <= 16'd1;
										next_state <= READ_WRITE;               // tRCD
							  end
						 end

						 READ_WRITE:
						 begin
								 addr <= 13'b0010000000000;          // A10=1 auto-precharge, column 0
								 
								 if (is_write)
								 begin
										 cmd   <= CMD_WRITE;
										 wdata <= {22'd0, wr_bank, 1'b0, wr_row[6:0]};   // data = {bank, row}
										 oe    <= 1'b1;
										 state <= WRITE;
								 end
								 
								 else
								 begin
										 cmd   		<= CMD_READ;
										 oe    		<= 1'b0;                      // release the bus long before the data arrives
										 state 		<= WAIT;
										 count   	<= 16'd3;                 // CAPTURE runs 5 clocks after this edge
										 next_state <= CAPTURE;
							    end
						 end

						 WRITE:
						 begin
								 oe <= 1'b0;
								
								 if (wr_row == 13'd100)
								 begin
										 wr_row <= 13'd0;
									
										 if (wr_idx == 2'd3)
										 begin	 
												 wr_done <= 1'b1;
										 end
									
										 else
										 begin	 
												 wr_idx <= wr_idx + 2'd1;
										 end		
								 end
							  
								 else
								 begin
										 wr_row <= wr_row + 13'd1;
								 end
								 
								 state 		<= WAIT;
								 count   	<= 16'd3;
								 next_state <= IDLE;                    // tWR + tRP
						 end

						 CAPTURE:
						 begin
								 rdata 	   <= DRAM_DQ;
								 match  		<= (DRAM_DQ == {6'd0, req_bank, 1'b0, req_row[6:0]}) && (req_row <= 13'd100);
								 state  		<= WAIT;
								 count    	<= 16'd3;
								 next_state <= IDLE;                   // tRP
						 end

						 default: state <= WAIT;
					endcase
        end
    end

endmodule
