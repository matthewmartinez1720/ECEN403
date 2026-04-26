`timescale 1ns / 1ps


module uart_autobaud_samples #(
    parameter integer MIN_BIT_TICKS      = 16,    // 16x oversample @ 1 Mbps
    parameter integer MAX_BIT_TICKS      = 13333, // 16 MHz / 1200 baud
    parameter integer EDGE_COUNT_REQUIRED = 8
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        start,

    input  wire        sample_ce,
    input  wire        sample_in,

    output reg [15:0]  bit_ticks,
    output reg         baud_valid,
    output reg         busy
);

    reg        prev_sample;
    reg [15:0] edge_ticks;
    reg [3:0]  edge_count;
    reg [15:0] min_ticks;

    wire edge_seen = (sample_in != prev_sample);
    wire in_range  = (edge_ticks >= MIN_BIT_TICKS[15:0]) &&
                     (edge_ticks <= MAX_BIT_TICKS[15:0]);

    always @(posedge clk) begin
        if (!rst_n) begin
            prev_sample <= 1'b1;
            edge_ticks  <= 16'd0;
            edge_count  <= 4'd0;
            min_ticks   <= 16'd0;
            bit_ticks   <= 16'd16;
            baud_valid  <= 1'b0;
            busy        <= 1'b0;
        end else begin
            if (!enable) begin
                prev_sample <= 1'b1;
                edge_ticks  <= 16'd0;
                edge_count  <= 4'd0;
                min_ticks   <= 16'd0;
                baud_valid  <= 1'b0;
                busy        <= 1'b0;
            end else begin
                if (start) begin
                    prev_sample <= 1'b1;
                    edge_ticks  <= 16'd0;
                    edge_count  <= 4'd0;
                    min_ticks   <= 16'd0;
                    baud_valid  <= 1'b0;
                    busy        <= 1'b1;
                end else if (busy && sample_ce) begin
                    if (edge_seen) begin
                        if (in_range) begin
                            if (edge_count == 4'd0) begin
                                min_ticks  <= edge_ticks;
                                edge_count <= 4'd1;
                            end else begin
                                if (edge_ticks < min_ticks)
                                    min_ticks <= edge_ticks;
                                if (edge_count < EDGE_COUNT_REQUIRED[3:0])
                                    edge_count <= edge_count + 4'd1;
                            end

                            if ((edge_count + 4'd1) >= EDGE_COUNT_REQUIRED[3:0]) begin
                                bit_ticks  <= (edge_ticks < min_ticks) ? edge_ticks : min_ticks;
                                baud_valid <= 1'b1;
                                busy       <= 1'b0;
                            end
                        end

                        edge_ticks  <= 16'd1;
                        prev_sample <= sample_in;
                    end else begin
                        if (edge_ticks < MAX_BIT_TICKS[15:0])
                            edge_ticks <= edge_ticks + 16'd1;
                    end
                end
            end
        end
    end

endmodule