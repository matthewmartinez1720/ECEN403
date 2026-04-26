`timescale 1ns / 1ps

module uart_tx #(
    parameter integer CLK_HZ = 12_000_000,
    parameter integer BAUD   = 115200
)(
    input  wire      clk,
    input  wire      rst_n,
    input  wire      start,
    input  wire [7:0] data_byte,
    output reg       tx_o,
    output reg       busy
);

    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;

    localparam [2:0] S_IDLE  = 3'd0;
    localparam [2:0] S_START = 3'd1;
    localparam [2:0] S_DATA  = 3'd2;
    localparam [2:0] S_STOP  = 3'd3;

    reg [2:0] state;
    reg [15:0] clk_cnt;
    reg [2:0] bit_idx;
    reg [7:0] shreg;

    always @(posedge clk) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            clk_cnt <= 16'd0;
            bit_idx <= 3'd0;
            shreg   <= 8'd0;
            tx_o    <= 1'b1;
            busy    <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    tx_o    <= 1'b1;
                    busy    <= 1'b0;
                    clk_cnt <= 16'd0;
                    bit_idx <= 3'd0;
                    if (start) begin
                        shreg   <= data_byte;
                        busy    <= 1'b1;
                        state   <= S_START;
                    end
                end

                S_START: begin
                    tx_o <= 1'b0;
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 16'd0;
                        state   <= S_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                S_DATA: begin
                    tx_o <= shreg[bit_idx];
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 16'd0;
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                S_STOP: begin
                    tx_o <= 1'b1;
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 16'd0;
                        state   <= S_IDLE;
                        busy    <= 1'b0;
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
