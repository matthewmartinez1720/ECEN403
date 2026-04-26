`timescale 1ns/1ps

module uart_rx #(
    parameter integer CLK_HZ = 12_000_000,
    parameter integer BAUD   = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx_i,
    output reg        data_valid,
    output reg [7:0]  data_byte
);

    localparam integer CLKS_PER_BIT  = CLK_HZ / BAUD;
    localparam integer HALF_BIT_CLKS = CLKS_PER_BIT / 2;

    localparam [2:0] S_IDLE  = 3'd0;
    localparam [2:0] S_START = 3'd1;
    localparam [2:0] S_DATA  = 3'd2;
    localparam [2:0] S_STOP  = 3'd3;

    reg [2:0] state;
    reg [15:0] clk_cnt;
    reg [2:0] bit_idx;
    reg [7:0] shreg;

    reg rx_ff1, rx_ff2;
    wire rx_s;

    assign rx_s = rx_ff2;

    always @(posedge clk) begin
        if (!rst_n) begin
            rx_ff1 <= 1'b1;
            rx_ff2 <= 1'b1;
        end else begin
            rx_ff1 <= rx_i;
            rx_ff2 <= rx_ff1;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            clk_cnt    <= 16'd0;
            bit_idx    <= 3'd0;
            shreg      <= 8'd0;
            data_valid <= 1'b0;
            data_byte  <= 8'd0;
        end else begin
            data_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    clk_cnt <= 16'd0;
                    bit_idx <= 3'd0;
                    if (rx_s == 1'b0)
                        state <= S_START;
                end

                S_START: begin
                    if (clk_cnt == HALF_BIT_CLKS-1) begin
                        clk_cnt <= 16'd0;
                        if (rx_s == 1'b0)
                            state <= S_DATA;
                        else
                            state <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                S_DATA: begin
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt       <= 16'd0;
                        shreg[bit_idx] <= rx_s;
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
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt    <= 16'd0;
                        state      <= S_IDLE;
                        data_valid <= 1'b1;
                        data_byte  <= shreg;
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
