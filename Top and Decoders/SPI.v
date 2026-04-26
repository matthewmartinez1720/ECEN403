`timescale 1ns/1ps

module spi_decoder #(
    parameter integer CLK_HZ            = 12_000_000,
    parameter integer SAMPLE_HZ         = 12_000_000,
    parameter integer MAX_PAYLOAD_BYTES = 240,
    parameter integer TIMEOUT_CYCLES    = (CLK_HZ/100)
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        enable,
    input  wire        start,

    input  wire        sclk,
    input  wire        cs_n,
    input  wire        mosi,
    input  wire        miso,

    output reg         byte_valid,
    input  wire        byte_ready,
    output reg  [7:0]  byte_data,

    output reg         busy,
    output reg         done
);

    function integer clog2_int;
        input integer value;
        integer i;
        begin
            value = value - 1;
            for (i = 0; value > 0; i = i + 1)
                value = value >> 1;
            clog2_int = (i < 1) ? 1 : i;
        end
    endfunction

    function [7:0] ceil_div8_11;
        input [10:0] v;
        begin
            ceil_div8_11 = (v[2:0] == 3'd0) ? {5'd0, v[10:3]} : ({5'd0, v[10:3]} + 8'd1);
        end
    endfunction

    function [15:0] pack_header;
        input err_flag;
        input trunc_flag;
        input [10:0] bitlen;
        begin
            pack_header = {err_flag, trunc_flag, 3'b000, bitlen};
        end
    endfunction

    // 16-bit wide memory: each word holds {MOSI_byte, MISO_byte}
    localparam integer PAIR_WORDS_MAX = MAX_PAYLOAD_BYTES / 2;
    localparam integer PAIR_W         = clog2_int(PAIR_WORDS_MAX);
    localparam integer IDLE_W         = clog2_int(TIMEOUT_CYCLES + 1);

    localparam [2:0]
        S_IDLE      = 3'd0,
        S_ARMED     = 3'd1,
        S_CAPT      = 3'd2,
        S_EMIT_HI   = 3'd3,
        S_EMIT_LO   = 3'd4,
        S_EMIT_REQ  = 3'd5,
        S_EMIT_SEND = 3'd6;

    // BRAM: single write port single read port
    (* ram_style = "block" *) reg [15:0] txn_mem [0:PAIR_WORDS_MAX-1];

    reg [PAIR_W-1:0]  wr_addr;
    reg [15:0]        wr_word;
    reg               wr_en;

    reg [PAIR_W-1:0]  rd_addr;
    reg [15:0]        rd_word;
    reg               rd_pending;
    reg               emit_miso;

    always @(posedge clk) begin
        if (wr_en)
            txn_mem[wr_addr] <= wr_word;
        rd_word <= txn_mem[rd_addr];
    end

    // Input synchronizers
    reg sclk_ff1, sclk_ff2;
    reg cs_ff1,   cs_ff2;
    reg mosi_ff1, mosi_ff2;
    reg miso_ff1, miso_ff2;

    always @(posedge clk) begin
        if (!rst_n) begin
            sclk_ff1 <= 1'b0; sclk_ff2 <= 1'b0;
            cs_ff1   <= 1'b1; cs_ff2   <= 1'b1;
            mosi_ff1 <= 1'b0; mosi_ff2 <= 1'b0;
            miso_ff1 <= 1'b0; miso_ff2 <= 1'b0;
        end else begin
            sclk_ff1 <= sclk; sclk_ff2 <= sclk_ff1;
            cs_ff1   <= cs_n; cs_ff2   <= cs_ff1;
            mosi_ff1 <= mosi; mosi_ff2 <= mosi_ff1;
            miso_ff1 <= miso; miso_ff2 <= miso_ff1;
        end
    end

    wire sclk_s = sclk_ff2;
    wire cs_s   = cs_ff2;
    wire mosi_s = mosi_ff2;
    wire miso_s = miso_ff2;

    reg sclk_prev, cs_prev;
    wire sclk_rise   = (sclk_prev == 1'b0) && (sclk_s == 1'b1);
    wire cs_assert   = (cs_prev == 1'b1) && (cs_s == 1'b0);
    wire cs_deassert = (cs_prev == 1'b0) && (cs_s == 1'b1);

    reg [2:0] state;

    reg [7:0] sh_mosi, sh_miso;
    reg [2:0] bit_in_byte;
    reg [10:0] bitlen_cur;
    reg [PAIR_W:0] pair_count;   // number of pairs written
    reg trunc_cur;
    reg have_txn;
    reg [IDLE_W-1:0] idle_cnt;

    reg [15:0] hdr_word;
    reg [7:0] out_pairs_left;    // pairs remaining to emit

    wire can_emit = ~byte_valid;

    always @(posedge clk) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            byte_valid       <= 1'b0;
            byte_data        <= 8'h00;
            busy             <= 1'b0;
            done             <= 1'b0;

            sclk_prev        <= 1'b0;
            cs_prev          <= 1'b1;

            sh_mosi          <= 8'h00;
            sh_miso          <= 8'h00;
            bit_in_byte      <= 3'd0;
            bitlen_cur       <= 11'd0;
            pair_count       <= {(PAIR_W+1){1'b0}};
            trunc_cur        <= 1'b0;
            have_txn         <= 1'b0;
            idle_cnt         <= {IDLE_W{1'b0}};

            hdr_word         <= 16'h0000;
            out_pairs_left   <= 8'd0;
            rd_addr          <= {PAIR_W{1'b0}};
            rd_pending       <= 1'b0;
            emit_miso        <= 1'b0;

            wr_en            <= 1'b0;
            wr_addr          <= {PAIR_W{1'b0}};
            wr_word          <= 16'h0000;
        end else begin
            done  <= 1'b0;
            wr_en <= 1'b0;

            if (byte_valid && byte_ready)
                byte_valid <= 1'b0;

            sclk_prev <= sclk_s;
            cs_prev   <= cs_s;

            if (!enable) begin
                state            <= S_IDLE;
                byte_valid       <= 1'b0;
                busy             <= 1'b0;
                done             <= 1'b0;

                sh_mosi          <= 8'h00;
                sh_miso          <= 8'h00;
                bit_in_byte      <= 3'd0;
                bitlen_cur       <= 11'd0;
                pair_count       <= {(PAIR_W+1){1'b0}};
                trunc_cur        <= 1'b0;
                have_txn         <= 1'b0;
                idle_cnt         <= {IDLE_W{1'b0}};

                hdr_word         <= 16'h0000;
                out_pairs_left   <= 8'd0;
                rd_addr          <= {PAIR_W{1'b0}};
                rd_pending       <= 1'b0;
                emit_miso        <= 1'b0;
            end else begin
                case (state)
                    S_IDLE: begin
                        if (start) begin
                            busy        <= 1'b1;
                            have_txn    <= 1'b0;
                            idle_cnt    <= {IDLE_W{1'b0}};
                            pair_count  <= {(PAIR_W+1){1'b0}};
                            sh_mosi     <= 8'h00;
                            sh_miso     <= 8'h00;
                            bit_in_byte <= 3'd0;
                            bitlen_cur  <= 11'd0;
                            trunc_cur   <= 1'b0;

                            if (cs_s == 1'b0) begin
                                hdr_word <= 16'h8000;
                                state    <= S_EMIT_HI;
                            end else begin
                                state    <= S_ARMED;
                            end
                        end
                    end

                    S_ARMED: begin
                        if (cs_assert) begin
                            pair_count  <= {(PAIR_W+1){1'b0}};
                            sh_mosi     <= 8'h00;
                            sh_miso     <= 8'h00;
                            bit_in_byte <= 3'd0;
                            bitlen_cur  <= 11'd0;
                            trunc_cur   <= 1'b0;
                            idle_cnt    <= {IDLE_W{1'b0}};
                            state       <= S_CAPT;
                        end else if (have_txn) begin
                            if (idle_cnt == TIMEOUT_CYCLES - 1) begin
                                busy  <= 1'b0;
                                done  <= 1'b1;
                                state <= S_IDLE;
                            end else begin
                                idle_cnt <= idle_cnt + {{(IDLE_W-1){1'b0}},1'b1};
                            end
                        end
                    end

                    S_CAPT: begin
                        if ((cs_s == 1'b0) && sclk_rise && !trunc_cur) begin
                            sh_mosi <= {sh_mosi[6:0], mosi_s};
                            sh_miso <= {sh_miso[6:0], miso_s};

                            if (bitlen_cur != 11'h7FF)
                                bitlen_cur <= bitlen_cur + 11'd1;

                            if (bit_in_byte == 3'd7) begin
                                if ((pair_count + 1) <= PAIR_WORDS_MAX) begin
                                    // single 16-bit write: {MOSI, MISO}
                                    wr_en   <= 1'b1;
                                    wr_addr <= pair_count[PAIR_W-1:0];
                                    wr_word <= {{sh_mosi[6:0], mosi_s}, {sh_miso[6:0], miso_s}};
                                    pair_count  <= pair_count + 1;
                                    bit_in_byte <= 3'd0;
                                    sh_mosi     <= 8'h00;
                                    sh_miso     <= 8'h00;
                                end else begin
                                    trunc_cur <= 1'b1;
                                end
                            end else begin
                                bit_in_byte <= bit_in_byte + 3'd1;
                            end
                        end

                        if (cs_deassert) begin
                            if ((bit_in_byte != 3'd0) && !trunc_cur) begin
                                if ((pair_count + 1) <= PAIR_WORDS_MAX) begin
                                    wr_en   <= 1'b1;
                                    wr_addr <= pair_count[PAIR_W-1:0];
                                    wr_word <= {(sh_mosi << (4'd8 - {1'b0, bit_in_byte})),
                                                (sh_miso << (4'd8 - {1'b0, bit_in_byte}))};
                                    pair_count <= pair_count + 1;
                                end else begin
                                    trunc_cur <= 1'b1;
                                end
                            end

                            hdr_word       <= pack_header(1'b0, trunc_cur, bitlen_cur);
                            out_pairs_left <= ceil_div8_11(bitlen_cur);  // one pair per SPI byte
                            rd_addr        <= {PAIR_W{1'b0}};
                            rd_pending     <= 1'b0;
                            emit_miso      <= 1'b0;
                            have_txn       <= 1'b1;
                            idle_cnt       <= {IDLE_W{1'b0}};
                            state          <= S_EMIT_HI;
                        end
                    end

                    S_EMIT_HI: begin
                        if (can_emit) begin
                            byte_data  <= hdr_word[15:8];
                            byte_valid <= 1'b1;
                            state      <= S_EMIT_LO;
                        end
                    end

                    S_EMIT_LO: begin
                        if (can_emit) begin
                            byte_data  <= hdr_word[7:0];
                            byte_valid <= 1'b1;

                            if (hdr_word == 16'h8000) begin
                                busy  <= 1'b0;
                                done  <= 1'b1;
                                state <= S_IDLE;
                            end else if (out_pairs_left == 8'd0) begin
                                state <= S_ARMED;
                            end else begin
                                rd_pending <= 1'b0;
                                emit_miso  <= 1'b0;
                                state      <= S_EMIT_REQ;
                            end
                        end
                    end

                    // One cycle for BRAM read latency
                    S_EMIT_REQ: begin
                        rd_addr    <= rd_addr;
                        rd_pending <= 1'b1;
                        state      <= S_EMIT_SEND;
                    end

                    S_EMIT_SEND: begin
                        if (rd_pending && can_emit) begin
                            if (!emit_miso) begin
                                // emit MOSI byte (high byte of 16-bit word)
                                byte_data  <= rd_word[15:8];
                                byte_valid <= 1'b1;
                                emit_miso  <= 1'b1;
                                // stay in EMIT_SEND to emit MISO next cycle
                            end else begin
                                // emit MISO byte (low byte of 16-bit word)
                                byte_data  <= rd_word[7:0];
                                byte_valid <= 1'b1;
                                emit_miso  <= 1'b0;
                                rd_pending <= 1'b0;

                                if (out_pairs_left == 8'd1) begin
                                    out_pairs_left <= 8'd0;
                                    state <= S_ARMED;
                                end else begin
                                    out_pairs_left <= out_pairs_left - 8'd1;
                                    rd_addr <= rd_addr + 1'b1;
                                    state   <= S_EMIT_REQ;
                                end
                            end
                        end
                    end

                    default: begin
                        state <= S_IDLE;
                    end
                endcase
            end
        end
    end

endmodule