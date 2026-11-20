//   SW[0] : reset (active low, up = run)
//   SW[1] : raise to record, lower to stop
//   SW[2] : raise to play back the recording, lower to stop
//
//   LEDG[1] SDRAM ready    LEDG[2] codec ready
//   LEDG[3] recording      LEDG[4] playing

module SDRAM_AUDIO_CODEC (input         CLOCK_50,
                          input  [17:0] SW,
                          output [17:0] LEDR,
                          output [8:0]  LEDG,
                          // Audio codec
                          input         AUD_ADCDAT,
                          output        AUD_XCK,
                          output        AUD_BCLK,
                          output        AUD_ADCLRCK,
                          output        AUD_DACLRCK,
                          output        AUD_DACDAT,
                          output        I2C_SCLK,
                          inout         I2C_SDAT,
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


// SDRAM commands {CS_N, RAS_N, CAS_N, WE_N}
localparam CMD_NOP   = 4'b0111;
localparam CMD_ACT   = 4'b0011;
localparam CMD_READ  = 4'b0101;
localparam CMD_WRITE = 4'b0100;
localparam CMD_PRE   = 4'b0010;
localparam CMD_REF   = 4'b0001;
localparam CMD_MRS   = 4'b0000;

// Mode register: CAS latency 2, sequential, burst length 1
localparam [12:0] MODE_REG = 13'b000_0_00_010_0_000;

// SDRAM controller states
localparam WAIT       = 4'd0,
           PRECHARGE  = 4'd1,
           REFRESH_1  = 4'd2,
           REFRESH_2  = 4'd3,
           MODE_REG_S = 4'd4,
           IDLE       = 4'd5,
           READ_WRITE = 4'd6,
           WRITE      = 4'd7,
           CAPTURE    = 4'd8;

// Record / play modes
localparam A_IDLE = 2'd0,
           A_REC  = 2'd1,
           A_PLAY = 2'd2;

// Maximum recording length in samples (48828 samples = 1 second, max 33554431)
localparam [24:0] MAX_SAMPLES = 25'd33554431;



wire rst = SW[0];                     
// SDRAM controller
reg [3:0]  state, next_state;
reg [15:0] count;
reg [3:0]  cmd;
reg [12:0] addr;
reg [1:0]  bank;
reg [31:0] wdata;
reg [15:0] rdata;
reg [15:0] refresh_count;
reg        oe;
reg        sdram_ready;
reg        rd_valid;
reg        pend_wr, pend_rd;
reg        is_write;
reg [24:0] wr_addr_l, rd_addr_l;
reg [15:0] wr_data_l, cur_data;
reg [9:0]  cur_col;

// Record / play control
reg [1:0]  mode;
reg [24:0] wr_addr, rd_addr, rec_len;
reg [15:0] play_sample;
reg        rec_prev, play_prev;

// Codec
wire        i2c_done;
wire        sample_ready;
wire [15:0] audio_out_16b;

localparam [19:0] DEBOUNCE_CYCLES = 20'd999999;   // 20 ms at 50 MHz
reg        rec_sw,   play_sw;                     // debounced levels
reg [1:0]  rec_sync, play_sync;                   // synchronisers
reg [19:0] rec_cnt,  play_cnt;                    // stability counters

// Edge detects and sample requests
wire rec_start  = rec_sw  & ~rec_prev;
wire play_start = play_sw & ~play_prev;
wire do_wr = (mode == A_REC)  && sample_ready && (wr_addr != MAX_SAMPLES);
wire do_rd = (mode == A_PLAY) && sample_ready && (rd_addr != rec_len);
wire wr_req = do_wr;
wire rd_req = do_rd;

assign DRAM_CLK  = ~CLOCK_50;
assign DRAM_CKE  = 1'b1;
assign DRAM_DQM  = 4'b0000;
assign {DRAM_CS_N, DRAM_RAS_N, DRAM_CAS_N, DRAM_WE_N} = cmd;
assign DRAM_ADDR = addr;
assign DRAM_BA   = bank;
assign DRAM_DQ   = oe ? wdata : 32'bz;

assign LEDG = {4'b0, (mode == A_PLAY), (mode == A_REC), i2c_done, sdram_ready, 1'b0};
assign LEDR = 18'd0;

always @(posedge CLOCK_50)
begin
    rec_sync  <= {rec_sync[0],  SW[1]};
    play_sync <= {play_sync[0], SW[2]};

    if (~rst)
    begin
        rec_cnt  <= 20'd0;
        play_cnt <= 20'd0;
        rec_sw   <= rec_sync[1];
        play_sw  <= play_sync[1];
    end

    else
    begin
        // SW[1] : record
        if (rec_sync[1] == rec_sw)
        begin
            rec_cnt <= 20'd0;
        end
        else if (rec_cnt == DEBOUNCE_CYCLES)
        begin
            rec_sw  <= rec_sync[1];
            rec_cnt <= 20'd0;
        end
        else
        begin
            rec_cnt <= rec_cnt + 1'b1;
        end

        // SW[2] : play
        if (play_sync[1] == play_sw)
        begin
            play_cnt <= 20'd0;
        end
        else if (play_cnt == DEBOUNCE_CYCLES)
        begin
            play_sw  <= play_sync[1];
            play_cnt <= 20'd0;
        end
        else
        begin
            play_cnt <= play_cnt + 1'b1;
        end
    end
end



AUDIO_CODEC codec (.clk(CLOCK_50),
                   .rst(rst),
                   .ac_recdat(AUD_ADCDAT),
                   .playback_en(mode == A_PLAY),
                   .ac_mclk(AUD_XCK),
                   .ac_bclk(AUD_BCLK),
                   .ac_pblrc(AUD_DACLRCK),
                   .ac_reclrc(AUD_ADCLRCK),
                   .ac_pbdat(AUD_DACDAT),
                   .ac_scl(I2C_SCLK),
                   .ac_sda(I2C_SDAT),
                   .sample_ready(sample_ready),
                   .audio_in_16b(play_sample),
                   .audio_out_16b(audio_out_16b),
                   .i2c_done(i2c_done));


always @(posedge CLOCK_50)
begin
    if (~rst)
    begin
        state         <= WAIT;
        count         <= 16'd10000;      // 200 us power-up delay
        next_state    <= PRECHARGE;
        cmd           <= CMD_NOP;
        addr          <= 13'd0;
        bank          <= 2'd0;
        oe            <= 1'b0;
        wdata         <= 32'd0;
        rdata         <= 16'd0;
        refresh_count <= 16'd0;
        is_write      <= 1'b0;
        sdram_ready   <= 1'b0;
        rd_valid      <= 1'b0;
        pend_wr       <= 1'b0;
        pend_rd       <= 1'b0;
        wr_addr_l     <= 25'd0;
        rd_addr_l     <= 25'd0;
        wr_data_l     <= 16'd0;
        cur_data      <= 16'd0;
        cur_col       <= 10'd0;
    end
    else
    begin
        cmd      <= CMD_NOP;
        rd_valid <= 1'b0;

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
                cmd        <= CMD_PRE;
                addr       <= 13'b0010000000000;   // A10=1 : precharge all banks
                state      <= WAIT;
                count      <= 16'd2;
                next_state <= REFRESH_1;
            end

            REFRESH_1:
            begin
                cmd        <= CMD_REF;
                state      <= WAIT;
                count      <= 16'd4;
                next_state <= REFRESH_2;
            end

            REFRESH_2:
            begin
                cmd        <= CMD_REF;
                state      <= WAIT;
                count      <= 16'd4;
                next_state <= MODE_REG_S;
            end

            MODE_REG_S:
            begin
                cmd         <= CMD_MRS;
                bank        <= 2'b00;
                addr        <= MODE_REG;
                state       <= WAIT;
                count       <= 16'd2;
                sdram_ready <= 1'b1;
                next_state  <= IDLE;
            end

            IDLE:
            begin
                if (refresh_count >= 16'd360)          // 7.2 us
                begin
                    cmd           <= CMD_REF;
                    refresh_count <= 16'd0;
                    state         <= WAIT;
                    count         <= 16'd4;
                    next_state    <= IDLE;
                end

                else if (pend_wr)
                begin
                    cmd        <= CMD_ACT;
                    bank       <= wr_addr_l[24:23];
                    addr       <= wr_addr_l[22:10];
                    cur_col    <= wr_addr_l[9:0];
                    cur_data   <= wr_data_l;
                    is_write   <= 1'b1;
                    pend_wr    <= 1'b0;
                    state      <= WAIT;
                    count      <= 16'd1;
                    next_state <= READ_WRITE;          // tRCD
                end

                else if (pend_rd)
                begin
                    cmd        <= CMD_ACT;
                    bank       <= rd_addr_l[24:23];
                    addr       <= rd_addr_l[22:10];
                    cur_col    <= rd_addr_l[9:0];
                    is_write   <= 1'b0;
                    pend_rd    <= 1'b0;
                    state      <= WAIT;
                    count      <= 16'd1;
                    next_state <= READ_WRITE;          // tRCD
                end
            end

            READ_WRITE:
            begin
                addr <= {2'b00, 1'b1, cur_col};        // A10=1 auto-precharge + column

                if (is_write)
                begin
                    cmd   <= CMD_WRITE;
                    wdata <= {16'd0, cur_data};        // audio sample
                    oe    <= 1'b1;
                    state <= WRITE;
                end

                else
                begin
                    cmd        <= CMD_READ;
                    oe         <= 1'b0;
                    state      <= WAIT;
                    count      <= 16'd3;               // READ -> CAPTURE delay
                    next_state <= CAPTURE;
                end
            end

            WRITE:
            begin
                oe         <= 1'b0;
                state      <= WAIT;
                count      <= 16'd3;
                next_state <= IDLE;                    // tWR + tRP
            end

            CAPTURE:
            begin
                rdata      <= DRAM_DQ[15:0];
                rd_valid   <= 1'b1;
                state      <= WAIT;
                count      <= 16'd3;
                next_state <= IDLE;                    // tRP
            end

            default: state <= WAIT;
        endcase

        // Latch new requests (after the case so a set wins over a clear)
        if (wr_req)
        begin
            pend_wr   <= 1'b1;
            wr_addr_l <= wr_addr;
            wr_data_l <= audio_out_16b;
        end

        if (rd_req)
        begin
            pend_rd   <= 1'b1;
            rd_addr_l <= rd_addr;
        end
    end
end


always @(posedge CLOCK_50)
begin
    rec_prev  <= rec_sw;
    play_prev <= play_sw;

    if (~rst)
    begin
        mode        <= A_IDLE;
        wr_addr     <= 25'd0;
        rd_addr     <= 25'd0;
        rec_len     <= 25'd0;
        play_sample <= 16'd0;
    end

    else
    begin
        if (rd_valid)
        begin
            play_sample <= rdata;
        end

        case (mode)

            A_IDLE:
            begin
                if (rec_start && i2c_done && sdram_ready)
                begin
                    wr_addr <= 25'd0;
                    mode    <= A_REC;
                end

                else if (play_start && i2c_done && sdram_ready && (rec_len != 25'd0))
                begin
                    rd_addr <= 25'd0;
                    mode    <= A_PLAY;
                end
            end

            A_REC:
            begin
                if (do_wr)
                begin
                    wr_addr <= wr_addr + 1'b1;
                end

                if (!rec_sw || wr_addr == MAX_SAMPLES)
                begin
                    rec_len <= wr_addr;
                    mode    <= A_IDLE;
                end
            end

            A_PLAY:
            begin
                if (do_rd)
                begin
                    rd_addr <= rd_addr + 1'b1;
                end

                if (!play_sw || rd_addr == rec_len)
                begin
                    mode <= A_IDLE;
                end
            end

            default: mode <= A_IDLE;
        endcase
    end
end

endmodule

