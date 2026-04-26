`timescale 1ns / 1ps

module uart_rx_buffered_autobaud #(
    parameter integer CLK_HZ         = 12_000_000,
    parameter integer MAX_BAUD       = 250_000,
    parameter integer MIN_BAUD       = 1200,
    parameter integer OVERSAMPLE     = 16,
    parameter integer INVERT_RX      = 0,
    parameter integer RAW_BYTES      = 2048,
    parameter integer IDLE_DONE_BITS = 128,
    parameter integer OUT_BYTES      = 256
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        start,
    input  wire        rx,

    output reg         byte_valid,
    input  wire        byte_ready,
    output reg  [7:0]  byte_data,

    output reg         busy,
    output reg         done,

    output reg         baud_locked,
    output reg [15:0]  locked_bit_ticks,
    output reg         raw_overflow
);

    localparam integer SAMPLE_HZ     = MAX_BAUD * OVERSAMPLE;
    localparam integer MIN_BIT_TICKS = SAMPLE_HZ / MAX_BAUD;
    localparam integer MAX_BIT_TICKS = SAMPLE_HZ / MIN_BAUD;
    localparam integer MAX_OUT_DATA  = OUT_BYTES - 2;

    // Suppress unused parameter warnings
    wire _unused_raw = |RAW_BYTES;

    // states
    localparam [3:0]
        ST_IDLE      = 4'd0,
        ST_HUNT_BAUD = 4'd1,
        ST_HDR1      = 4'd2,
        ST_HDR2      = 4'd3,
        ST_HUNT_EDGE = 4'd4,
        ST_SAMPLE    = 4'd5,
        ST_DONE      = 4'd6;

    reg [3:0] state;

    // RX sync
    reg  rx_ff1, rx_ff2;
    wire rx_in = INVERT_RX ? ~rx : rx;

    // sample clock
    reg  [31:0] sample_accum;
    wire [31:0] sample_accum_sum = sample_accum + SAMPLE_HZ[31:0];
    wire        sample_ce        = (sample_accum_sum >= CLK_HZ[31:0]);

    // falling edge detect on the sample_ce grid
    reg rx_prev_ce;
    wire falling_edge_ce = sample_ce && rx_prev_ce && !rx_ff2;

    // autobaud
    wire [15:0] ab_bit_ticks;
    wire        ab_valid, ab_busy;

    uart_autobaud_samples #(
        .MIN_BIT_TICKS       (MIN_BIT_TICKS),
        .MAX_BIT_TICKS       (MAX_BIT_TICKS),
        .EDGE_COUNT_REQUIRED (8)
    ) u_autobaud (
        .clk       (clk),
        .rst_n     (rst_n),
        .enable    (enable),
        .start     (start),
        .sample_ce (sample_ce),
        .sample_in (rx_ff2),
        .bit_ticks (ab_bit_ticks),
        .baud_valid(ab_valid),
        .busy      (ab_busy)
    );

    // per bit sampling state
    reg [15:0] half_bit_ticks;
    reg [15:0] tick_cnt;      // sample_ce ticks since reference edge/bit
    reg  [3:0] bit_idx;       // 0 = start, 1-8 = data bits, 9 = stop
    reg  [7:0] data_shreg;    // assembled byte (LSB first)
    reg [23:0] idle_tick_cnt; // idle sample_ce ticks while hunting
    reg [11:0] bytes_out;     // decoded data bytes emitted

    // Threshold for the idle timeout computed combinatorially from the locked bit period
    wire [23:0] idle_threshold = IDLE_DONE_BITS[15:0] * locked_bit_ticks;

    // Sequential
    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            rx_ff1         <= 1'b1;
            rx_ff2         <= 1'b1;
            rx_prev_ce     <= 1'b1;
            sample_accum   <= 32'd0;

            byte_valid     <= 1'b0;
            byte_data      <= 8'h00;

            busy           <= 1'b0;
            done           <= 1'b0;

            baud_locked      <= 1'b0;
            locked_bit_ticks <= 16'd0;
            raw_overflow     <= 1'b0;

            half_bit_ticks <= 16'd0;
            tick_cnt       <= 16'd0;
            bit_idx        <= 4'd0;
            data_shreg     <= 8'h00;
            idle_tick_cnt  <= 24'd0;
            bytes_out      <= 12'd0;
        end else begin
            // RX pipeline
            rx_ff1 <= rx_in;
            rx_ff2 <= rx_ff1;

            // defaults
            done <= 1'b0;

            // sample clock
            if (!enable || state == ST_IDLE || state == ST_DONE)
                sample_accum <= 32'd0;
            else if (sample_ce)
                sample_accum <= sample_accum_sum - CLK_HZ[31:0];
            else
                sample_accum <= sample_accum_sum;

            if (sample_ce)
                rx_prev_ce <= rx_ff2;

            // output handshake
            if (byte_valid && byte_ready)
                byte_valid <= 1'b0;

            // disable
            if (!enable) begin
                state       <= ST_IDLE;
                busy        <= 1'b0;
                baud_locked <= 1'b0;
                byte_valid  <= 1'b0;
            end else if (start) begin
                // start pulse
                state            <= ST_HUNT_BAUD;
                busy             <= 1'b1;
                done             <= 1'b0;

                baud_locked      <= 1'b0;
                locked_bit_ticks <= 16'd0;
                half_bit_ticks   <= 16'd0;
                raw_overflow     <= 1'b0;

                tick_cnt         <= 16'd0;
                bit_idx          <= 4'd0;
                data_shreg       <= 8'h00;
                idle_tick_cnt    <= 24'd0;
                bytes_out        <= 12'd0;

                byte_valid       <= 1'b0;
                rx_prev_ce       <= 1'b1;
            end else begin
                case (state)

                ST_IDLE: begin
                    busy <= 1'b0;
                end

                // Wait for autobaud to lock
                ST_HUNT_BAUD: begin
                    if (ab_valid) begin
                        baud_locked      <= 1'b1;
                        locked_bit_ticks <= ab_bit_ticks;
                        half_bit_ticks   <= ab_bit_ticks >> 1;
                        state            <= ST_HDR1;
                    end
                end

                // Emit 2-byte bit_ticks header, MSB first
                ST_HDR1: begin
                    if (!byte_valid) begin
                        byte_data  <= locked_bit_ticks[15:8];
                        byte_valid <= 1'b1;
                        state      <= ST_HDR2;
                    end
                end

                ST_HDR2: begin
                    if (!byte_valid) begin
                        byte_data     <= locked_bit_ticks[7:0];
                        byte_valid    <= 1'b1;
                        state         <= ST_HUNT_EDGE;
                        tick_cnt      <= 16'd0;
                        idle_tick_cnt <= 24'd0;
                    end
                end

                // hunt for the falling edge (start of start bit)
                // Idle timeout: count sample_ce ticks while the line  stays recessive
                ST_HUNT_EDGE: begin
                    if (sample_ce) begin
                        if (falling_edge_ce) begin
                            tick_cnt      <= 16'd1;
                            bit_idx       <= 4'd0;
                            state         <= ST_SAMPLE;
                            idle_tick_cnt <= 24'd0;
                        end else if (rx_ff2) begin
                            if (idle_tick_cnt != {24{1'b1}})
                                idle_tick_cnt <= idle_tick_cnt + 24'd1;

                            if (bytes_out != 12'd0 &&
                                idle_tick_cnt >= idle_threshold) begin
                                state <= ST_DONE;
                                busy  <= 1'b0;
                                done  <= 1'b1;
                            end
                        end
                    end
                end


                // Sample the 10 bits of a UART frame
                ST_SAMPLE: begin
                    if (sample_ce) begin
                        if (bit_idx == 4'd0) begin
                            // Start bit centre
                            if (tick_cnt + 16'd1 >= half_bit_ticks) begin
                                bit_idx  <= 4'd1;
                                tick_cnt <= 16'd0;
                            end else begin
                                tick_cnt <= tick_cnt + 16'd1;
                            end
                        end else begin
                            // data/stop bit centres
                            if (tick_cnt + 16'd1 >= locked_bit_ticks) begin
                                if (bit_idx <= 4'd8) begin
                                    // data bit: LSB first
                                    data_shreg <= {rx_ff2, data_shreg[7:1]};
                                    bit_idx    <= bit_idx + 4'd1;
                                    tick_cnt   <= 16'd0;
                                end else begin
                                    // stop bit - emit assembled byte
                                    if (!byte_valid) begin
                                        byte_data  <= data_shreg;
                                        byte_valid <= 1'b1;
                                    end
                                    bytes_out     <= bytes_out + 12'd1;
                                    idle_tick_cnt <= 24'd0;

                                    if ((bytes_out + 12'd1) >=
                                            MAX_OUT_DATA[11:0]) begin
                                        state <= ST_DONE;
                                        busy  <= 1'b0;
                                        done  <= 1'b1;
                                    end else begin
                                        state <= ST_HUNT_EDGE;
                                    end
                                end
                            end else begin
                                tick_cnt <= tick_cnt + 16'd1;
                            end
                        end
                    end
                end

                ST_DONE: begin
                    busy <= 1'b0;
                end

                default: state <= ST_IDLE;
                endcase
            end
        end
    end

endmodule