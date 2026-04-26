`timescale 1ns / 1ps

module i2c_decoder #(
    parameter integer CAP_BYTES = 256
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        start,
    input  wire        scl,
    input  wire        sda,
    output reg         byte_valid,
    input  wire        byte_ready,
    output reg  [7:0]  byte_data,
    output reg         busy,
    output reg         done
);


    // Synchronizers
    reg scl_ff1, scl_ff2;
    reg sda_ff1, sda_ff2;
    always @(posedge clk) begin
        if (!rst_n) begin
            scl_ff1 <= 1'b1; scl_ff2 <= 1'b1;
            sda_ff1 <= 1'b1; sda_ff2 <= 1'b1;
        end else begin
            scl_ff1 <= scl;  scl_ff2 <= scl_ff1;
            sda_ff1 <= sda;  sda_ff2 <= sda_ff1;
        end
    end
    wire scl_s = scl_ff2;
    wire sda_s = sda_ff2;

    // Edge detect
    reg scl_d, sda_d;
    always @(posedge clk) begin
        if (!rst_n) begin
            scl_d <= 1'b1;
            sda_d <= 1'b1;
        end else begin
            scl_d <= scl_s;
            sda_d <= sda_s;
        end
    end
    wire scl_rise = (scl_d == 1'b0) && (scl_s == 1'b1);
    wire scl_fall = (scl_d == 1'b1) && (scl_s == 1'b0);
    wire sda_fall = (sda_d == 1'b1) && (sda_s == 1'b0);
    wire sda_rise = (sda_d == 1'b0) && (sda_s == 1'b1);

    wire start_cond = (scl_s == 1'b1) && sda_fall;
    wire stop_cond  = (scl_s == 1'b1) && sda_rise;


    // BRAM
    (* ram_style = "block" *) reg [7:0] cap_mem [0:CAP_BYTES-1];

    reg [7:0] mem_wr_data;
    reg [8:0] mem_wr_addr;
    reg       mem_wr_en;

    reg [8:0] mem_rd_addr;
    reg [7:0] mem_rd_data;

    always @(posedge clk) begin
        if (mem_wr_en)
            cap_mem[mem_wr_addr[7:0]] <= mem_wr_data;
        mem_rd_data <= cap_mem[mem_rd_addr[7:0]];
    end


    // Capture state
    reg [7:0]  shreg;
    reg [2:0]  bit_in_byte;
    reg [8:0]  wr_idx;
    reg [15:0] bit_length;
    reg [8:0]  bytes_captured;
    reg [8:0]  out_idx;

    localparam [8:0] CAP_BYTES9 = CAP_BYTES[8:0];

    wire can_emit = (!byte_valid) || (byte_valid && byte_ready);

    reg pending_valid;
    reg pending_bit;

    // FSM
    localparam [2:0]
        S_IDLE       = 3'd0,
        S_WAIT_START = 3'd1,
        S_CAPTURE    = 3'd2,
        S_OUT_LEN_H  = 3'd3,
        S_OUT_LEN_L  = 3'd4,
        S_OUT_REQ    = 3'd5,   // BRAM read-latency wait
        S_OUT_DATA   = 3'd6;

    reg [2:0] state;

    always @(posedge clk) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            busy          <= 1'b0;
            done          <= 1'b0;
            byte_valid    <= 1'b0;
            byte_data     <= 8'h00;

            shreg         <= 8'h00;
            bit_in_byte   <= 3'd0;
            wr_idx        <= 9'd0;
            bit_length    <= 16'd0;
            bytes_captured<= 9'd0;
            out_idx       <= 9'd0;

            pending_valid <= 1'b0;
            pending_bit   <= 1'b0;

            mem_wr_en     <= 1'b0;
            mem_wr_addr   <= 9'd0;
            mem_wr_data   <= 8'h00;
            mem_rd_addr   <= 9'd0;
        end else begin
            done      <= 1'b0;
            mem_wr_en <= 1'b0;

            if (byte_valid && byte_ready)
                byte_valid <= 1'b0;

            if (!enable) begin
                state         <= S_IDLE;
                busy          <= 1'b0;
                byte_valid    <= 1'b0;
                pending_valid <= 1'b0;
            end else begin
                case (state)
                    
                    S_IDLE: begin
                        busy          <= 1'b0;
                        pending_valid <= 1'b0;
                        if (start) begin
                            wr_idx         <= 9'd0;
                            bit_length     <= 16'd0;
                            bytes_captured <= 9'd0;
                            shreg          <= 8'h00;
                            bit_in_byte    <= 3'd0;
                            out_idx        <= 9'd0;
                            byte_valid     <= 1'b0;
                            pending_valid  <= 1'b0;
                            busy           <= 1'b1;
                            state          <= S_WAIT_START;
                        end
                    end


                    S_WAIT_START: begin
                        pending_valid <= 1'b0;
                        if (start_cond) begin
                            shreg       <= 8'h00;
                            bit_in_byte <= 3'd0;
                            state       <= S_CAPTURE;
                        end
                    end


                    S_CAPTURE: begin
                        if (wr_idx >= CAP_BYTES9) begin
                            // Buffer full -> finalize
                            pending_valid  <= 1'b0;
                            bytes_captured <= CAP_BYTES9;
                            bit_length     <= CAP_BYTES9 * 16'd8;
                            out_idx        <= 9'd0;
                            state          <= S_OUT_LEN_H;

                        end else if (stop_cond) begin
                            // Stop condition, store partial byte if any
                            pending_valid <= 1'b0;
                            if ((bit_in_byte != 3'd0) && (wr_idx < CAP_BYTES9)) begin
                                mem_wr_en   <= 1'b1;
                                mem_wr_addr <= wr_idx;
                                mem_wr_data <= shreg;
                                wr_idx      <= wr_idx + 9'd1;
                            end

                            // Compute bytes_captured
                            if (((bit_length + 16'd7) >> 3) > {7'd0, CAP_BYTES9})
                                bytes_captured <= CAP_BYTES9;
                            else
                                bytes_captured <= ((bit_length + 16'd7) >> 3);

                            out_idx <= 9'd0;
                            state   <= S_OUT_LEN_H;

                        end else begin
                            // Normal capture
                            if (start_cond) begin
                                shreg         <= 8'h00;
                                bit_in_byte   <= 3'd0;
                                pending_valid <= 1'b0;
                            end

                            if (scl_rise) begin
                                pending_valid <= 1'b1;
                                pending_bit   <= sda_s;
                            end

                            if (scl_fall && pending_valid) begin
                                pending_valid <= 1'b0;
                                if (bit_length < (CAP_BYTES * 8)) begin
                                    // Commit one bit
                                    if (wr_idx < CAP_BYTES9) begin
                                        shreg      <= {shreg[6:0], pending_bit};
                                        bit_length <= bit_length + 16'd1;

                                        if (bit_in_byte == 3'd7) begin
                                            mem_wr_en   <= 1'b1;
                                            mem_wr_addr <= wr_idx;
                                            mem_wr_data <= {shreg[6:0], pending_bit};
                                            wr_idx      <= wr_idx + 9'd1;
                                            shreg       <= 8'h00;
                                            bit_in_byte <= 3'd0;
                                        end else begin
                                            bit_in_byte <= bit_in_byte + 3'd1;
                                        end
                                    end
                                end
                            end
                        end
                    end


                    S_OUT_LEN_H: begin
                        if (can_emit) begin
                            byte_data  <= bit_length[15:8];
                            byte_valid <= 1'b1;
                            state      <= S_OUT_LEN_L;
                        end
                    end


                    S_OUT_LEN_L: begin
                        if (can_emit) begin
                            byte_data  <= bit_length[7:0];
                            byte_valid <= 1'b1;
                            if (bytes_captured == 9'd0) begin
                                state <= S_IDLE;
                                busy  <= 1'b0;
                                done  <= 1'b1;
                            end else begin
                                // Preload first read address
                                mem_rd_addr <= 9'd0;
                                state       <= S_OUT_REQ;
                            end
                        end
                    end


                    // One-cycle wait for BRAM read latency
                    S_OUT_REQ: begin
                        state <= S_OUT_DATA;
                    end


                    S_OUT_DATA: begin
                        if (can_emit) begin
                            if (out_idx < bytes_captured) begin
                                byte_data   <= mem_rd_data;
                                byte_valid  <= 1'b1;
                                out_idx     <= out_idx + 9'd1;
                                // preload next address
                                mem_rd_addr <= out_idx + 9'd1;
                                // Need one cycle for BRAM read before next emit
                                state       <= S_OUT_REQ;
                            end else begin
                                state <= S_IDLE;
                                busy  <= 1'b0;
                                done  <= 1'b1;
                            end
                        end
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule