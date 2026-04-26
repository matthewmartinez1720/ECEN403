`timescale 1ns / 1ps

module can_decoder #(
    parameter integer CLK_HZ          = 100_000_000,
    parameter integer MAX_FRAME_BYTES = 32,           // worst case output bytes
    parameter integer IDLE_TIMEOUT    = 100_000_000   // 1 s @ 100 MHz
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        enable,
    input  wire        start,

    input  wire        can_rx,

    output reg         byte_valid,
    input  wire        byte_ready,
    output reg  [7:0]  byte_data,

    output reg         busy,
    output reg         done,
    output reg         crc_error
);

    // input synchroniser (2-FF + majority-vote glitch filter)
    reg rx_ff1, rx_ff2, rx_ff3;
    always @(posedge clk) begin
        if (!rst_n) begin
            rx_ff1 <= 1'b1; rx_ff2 <= 1'b1; rx_ff3 <= 1'b1;
        end else begin
            rx_ff1 <= can_rx; rx_ff2 <= rx_ff1; rx_ff3 <= rx_ff2;
        end
    end
    wire rx_s = (rx_ff1 & rx_ff2) | (rx_ff2 & rx_ff3) | (rx_ff1 & rx_ff3);


    // Edge detect
    reg rx_prev;
    always @(posedge clk) begin
        if (!rst_n) rx_prev <= 1'b1;
        else        rx_prev <= rx_s;
    end
    wire rx_falling = ( rx_prev) & (~rx_s);
    wire rx_rising  = (~rx_prev) & ( rx_s);
    wire rx_edge    = rx_falling | rx_rising;

    // bit time measuerment 
    localparam BT_W = 16;

    reg  [BT_W-1:0] edge_cnt;
    reg  [BT_W-1:0] min_pulse;
    reg  [BT_W-1:0] bit_time;
    reg              baud_locked;
    reg  [4:0]       edge_tally;

    localparam [BT_W-1:0] INIT_MIN   = {BT_W{1'b1}};
    localparam [4:0]      LOCK_EDGES = 5'd12;

    // bit sampling
    reg [BT_W-1:0] sample_cnt;
    reg             sample_tick;
    reg             sampled_bit;

    wire [BT_W-1:0] sample_point = bit_time - (bit_time >> 2); // 75 %


    // Bit-destuffing
    reg [2:0] consec_same;
    reg       prev_bit;
    reg       destuffed_valid;
    reg       destuffed_bit;
    reg       stuff_error;


    // CRC-15
    reg [14:0] crc_reg;
    wire       crc_nxt_bit = destuffed_bit ^ crc_reg[14];


    // state machine
    localparam [3:0]
        S_OFF         = 4'd0,
        S_WAIT_IDLE   = 4'd1,
        S_WAIT_SOF    = 4'd2,
        S_BAUD_HUNT   = 4'd3,
        S_ARBITRATION = 4'd4,
        S_CONTROL     = 4'd5,
        S_DATA        = 4'd6,
        S_CRC         = 4'd7,
        S_ACK_EOF     = 4'd8,
        S_EMIT        = 4'd9,
        S_DONE        = 4'd10;

    reg [3:0] state;


    // parsed frame fields
    reg        is_extended;
    reg        is_rtr;
    reg [28:0] frame_id;
    reg  [3:0] dlc;
    reg [63:0] payload;
    reg [14:0] rx_crc;

    reg  [6:0] field_cnt;
    reg  [6:0] data_bits_left;
    reg [27:0] idle_timer;


    // emission state (no out_buf array)
    reg [4:0] out_idx;

    // Total output bytes computed combinatorially from frame fields
    wire [4:0] out_len = (is_extended ? 5'd7 : 5'd5) + {1'b0, dlc};


    // Combinatorial byte-select function
    function [7:0] emit_byte_mux;
        input [4:0]  idx;
        input        ext;
        input        rtr;
        input [28:0] fid;
        input  [3:0] d;
        input [63:0] pld;
        input [14:0] crc;
        reg [4:0] data_off;
        reg [3:0] di;
        begin
            data_off = ext ? 5'd5 : 5'd3;
            // payload byte index (valid when idx in data range)
            di = idx[3:0] - data_off[3:0];

            if (idx == 5'd0) begin
                emit_byte_mux = {ext, rtr, 2'b00, d};
            end else if (!ext) begin
                // standard frame
                if (idx == 5'd1)
                    emit_byte_mux = {5'd0, fid[10:8]};
                else if (idx == 5'd2)
                    emit_byte_mux = fid[7:0];
                else if (idx >= 5'd3 && idx < (5'd3 + {1'b0, d})) begin
                    case (di[2:0])
                        3'd0: emit_byte_mux = pld[63:56];
                        3'd1: emit_byte_mux = pld[55:48];
                        3'd2: emit_byte_mux = pld[47:40];
                        3'd3: emit_byte_mux = pld[39:32];
                        3'd4: emit_byte_mux = pld[31:24];
                        3'd5: emit_byte_mux = pld[23:16];
                        3'd6: emit_byte_mux = pld[15: 8];
                        default: emit_byte_mux = pld[7:0];
                    endcase
                end else if (idx == (5'd3 + {1'b0, d}))
                    emit_byte_mux = {1'b0, crc[14:8]};
                else
                    emit_byte_mux = crc[7:0];
            end else begin
                // extended frame
                if (idx == 5'd1)
                    emit_byte_mux = {3'd0, fid[28:24]};
                else if (idx == 5'd2)
                    emit_byte_mux = fid[23:16];
                else if (idx == 5'd3)
                    emit_byte_mux = fid[15:8];
                else if (idx == 5'd4)
                    emit_byte_mux = fid[7:0];
                else if (idx >= 5'd5 && idx < (5'd5 + {1'b0, d})) begin
                    case (di[2:0])
                        3'd0: emit_byte_mux = pld[63:56];
                        3'd1: emit_byte_mux = pld[55:48];
                        3'd2: emit_byte_mux = pld[47:40];
                        3'd3: emit_byte_mux = pld[39:32];
                        3'd4: emit_byte_mux = pld[31:24];
                        3'd5: emit_byte_mux = pld[23:16];
                        3'd6: emit_byte_mux = pld[15: 8];
                        default: emit_byte_mux = pld[7:0];
                    endcase
                end else if (idx == (5'd5 + {1'b0, d}))
                    emit_byte_mux = {1'b0, crc[14:8]};
                else
                    emit_byte_mux = crc[7:0];
            end
        end
    endfunction

    // sequential logic
    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= S_OFF;
            busy           <= 1'b0;
            done           <= 1'b0;
            byte_valid     <= 1'b0;
            byte_data      <= 8'd0;
            crc_error      <= 1'b0;
            baud_locked    <= 1'b0;
            edge_cnt       <= 0;
            min_pulse      <= INIT_MIN;
            bit_time       <= 0;
            edge_tally     <= 0;
            sample_cnt     <= 0;
            sample_tick    <= 1'b0;
            sampled_bit    <= 1'b1;
            consec_same    <= 0;
            prev_bit       <= 1'b1;
            destuffed_valid <= 1'b0;
            destuffed_bit  <= 1'b0;
            stuff_error    <= 1'b0;
            crc_reg        <= 15'd0;
            out_idx        <= 0;
            field_cnt      <= 0;
            data_bits_left <= 0;
            is_extended    <= 1'b0;
            is_rtr         <= 1'b0;
            frame_id       <= 29'd0;
            dlc            <= 4'd0;
            payload        <= 64'd0;
            rx_crc         <= 15'd0;
            idle_timer     <= 0;
        end else begin

            // defaults
            sample_tick     <= 1'b0;
            destuffed_valid <= 1'b0;


            // edge counter (free running while active)
            if (state != S_OFF && state != S_DONE && state != S_EMIT) begin
                if (rx_edge) begin
                    if (edge_cnt < min_pulse && edge_cnt > (CLK_HZ[BT_W-1:0] / 2_000_000))
                        min_pulse <= edge_cnt;
                    edge_cnt <= 1;
                    if (edge_tally < LOCK_EDGES)
                        edge_tally <= edge_tally + 1'b1;
                end else begin
                    if (edge_cnt != {BT_W{1'b1}})
                        edge_cnt <= edge_cnt + 1'b1;
                end
            end

            // Bit time sample counter
            if (baud_locked && state >= S_ARBITRATION && state <= S_ACK_EOF) begin
                if (rx_edge) begin
                    if (sample_cnt < sample_point)
                        sample_cnt <= 1;
                end else begin
                    sample_cnt <= sample_cnt + 1'b1;
                end

                if (sample_cnt == sample_point) begin
                    sample_tick <= 1'b1;
                    sampled_bit <= rx_s;
                end

                if (sample_cnt >= bit_time)
                    sample_cnt <= 1;
            end


            // bit destuffing
            if (sample_tick && state >= S_ARBITRATION && state <= S_CRC) begin
                if (consec_same == 3'd5) begin
                    if (sampled_bit == prev_bit)
                        stuff_error <= 1'b1;
                    consec_same <= 3'd1;
                    prev_bit    <= sampled_bit;
                    // stuff bit: do not forward
                end else begin
                    destuffed_valid <= 1'b1;
                    destuffed_bit   <= sampled_bit;
                    consec_same     <= (sampled_bit == prev_bit) ? consec_same + 1'b1 : 3'd1;
                    prev_bit        <= sampled_bit;
                end
            end

            // CRC-15 accumulation
            if (destuffed_valid && state >= S_ARBITRATION && state <= S_DATA) begin
                crc_reg <= crc_nxt_bit
                         ? {crc_reg[13:0], 1'b0} ^ 15'h4599
                         : {crc_reg[13:0], 1'b0};
            end

            // idle timer
            if (rx_s == 1'b0 || rx_edge)
                idle_timer <= 0;
            else if (idle_timer != {28{1'b1}})
                idle_timer <= idle_timer + 1'b1;

            // state machine
            case (state)

            // disabled
            S_OFF: begin
                busy <= 1'b0;
                done <= 1'b0;
                if (enable && start) begin
                    state       <= S_WAIT_IDLE;
                    busy        <= 1'b1;
                    done        <= 1'b0;
                    crc_error   <= 1'b0;
                    stuff_error <= 1'b0;
                    baud_locked <= 1'b0;
                    min_pulse   <= INIT_MIN;
                    edge_tally  <= 0;
                    edge_cnt    <= 0;
                    idle_timer  <= 0;
                end
            end

            // Wwait for bus idle (~200 µs of recessive)
            S_WAIT_IDLE: begin
                if (idle_timer >= (CLK_HZ[27:0] / 5000))
                    state <= S_WAIT_SOF;
                if (idle_timer >= IDLE_TIMEOUT[27:0])
                    state <= S_DONE;
            end

            // wait for SOF (falling edge = dominant) 
            S_WAIT_SOF: begin
                if (rx_falling) begin
                    state         <= S_BAUD_HUNT;
                    edge_cnt      <= 1;
                    min_pulse     <= INIT_MIN;
                    edge_tally    <= 0;
                    crc_reg       <= 15'd0;
                    consec_same   <= 3'd1;
                    prev_bit      <= 1'b0;
                    field_cnt     <= 0;
                    is_extended   <= 1'b0;
                    is_rtr        <= 1'b0;
                    frame_id      <= 29'd0;
                    dlc           <= 4'd0;
                    payload       <= 64'd0;
                end
                if (idle_timer >= IDLE_TIMEOUT[27:0])
                    state <= S_DONE;
            end

            // collecting edges to measure baud
            S_BAUD_HUNT: begin
                if (edge_tally >= LOCK_EDGES) begin
                    bit_time    <= min_pulse;
                    baud_locked <= 1'b1;
                    sample_cnt  <= 1;
                    // restart cleanly from next SOF with locked baud rate
                    state       <= S_WAIT_SOF;
                end
                if (idle_timer >= IDLE_TIMEOUT[27:0])
                    state <= S_DONE;
            end

            // arbitration field
            // STD: 11 ID[10:0] + RTR + IDE(0)
            // EXT: 11 ID_A + SRR + IDE(1) + 18 ID_B + RTR
            S_ARBITRATION: begin
                if (stuff_error) state <= S_DONE;
                if (destuffed_valid) begin
                    field_cnt <= field_cnt + 1'b1;

                    if (!is_extended) begin
                        case (field_cnt)
                            7'd0:  frame_id[28] <= destuffed_bit;
                            7'd1:  frame_id[27] <= destuffed_bit;
                            7'd2:  frame_id[26] <= destuffed_bit;
                            7'd3:  frame_id[25] <= destuffed_bit;
                            7'd4:  frame_id[24] <= destuffed_bit;
                            7'd5:  frame_id[23] <= destuffed_bit;
                            7'd6:  frame_id[22] <= destuffed_bit;
                            7'd7:  frame_id[21] <= destuffed_bit;
                            7'd8:  frame_id[20] <= destuffed_bit;
                            7'd9:  frame_id[19] <= destuffed_bit;
                            7'd10: frame_id[18] <= destuffed_bit;
                            7'd11: is_rtr       <= destuffed_bit;
                            7'd12: begin
                                if (destuffed_bit) begin
                                    // IDE=1 -> extended frame
                                    is_extended <= 1'b1;
                                    field_cnt   <= 7'd13;
                                end else begin
                                    // IDE=0  -> standard frame; align ID to [10:0]
                                    frame_id[10:0]  <= frame_id[28:18];
                                    frame_id[28:11] <= 18'd0;
                                    state     <= S_CONTROL;
                                    field_cnt <= 0;
                                end
                            end
                            default: ;
                        endcase
                    end else begin
                        // extended: collect 18 bit ID_B
                        if (field_cnt >= 7'd13 && field_cnt <= 7'd30)
                            frame_id[30 - field_cnt] <= destuffed_bit;
                        if (field_cnt == 7'd31)
                            is_rtr <= destuffed_bit;
                        if (field_cnt == 7'd32) begin
                            state     <= S_CONTROL;
                            field_cnt <= 7'd1; // r0 comes next
                        end
                    end
                end
            end

            // control field: r0 + DLC[3:0]
            S_CONTROL: begin
                if (stuff_error) state <= S_DONE;
                if (destuffed_valid) begin
                    field_cnt <= field_cnt + 1'b1;
                    case (field_cnt)
                        7'd0: ; // r0 - discard
                        7'd1: dlc[3] <= destuffed_bit;
                        7'd2: dlc[2] <= destuffed_bit;
                        7'd3: dlc[1] <= destuffed_bit;
                        7'd4: begin
                            dlc[0] <= destuffed_bit;
                            field_cnt <= 0;
                            if ({dlc[3:1], destuffed_bit} > 4'd8)
                                data_bits_left <= 7'd64;
                            else
                                data_bits_left <= {3'd0, {dlc[3:1], destuffed_bit}} << 3;
                            state <= (({dlc[3:1], destuffed_bit} == 4'd0) || is_rtr)
                                     ? S_CRC : S_DATA;
                        end
                        default: ;
                    endcase
                end
            end

            // data field
            S_DATA: begin
                if (stuff_error) state <= S_DONE;
                if (destuffed_valid) begin
                    payload        <= {payload[62:0], destuffed_bit};
                    data_bits_left <= data_bits_left - 1'b1;
                    if (data_bits_left == 7'd1) begin
                        field_cnt <= 0;
                        state     <= S_CRC;
                    end
                end
            end

            // CRC field (15 bits -> no further CRC accumulation)
            S_CRC: begin
                if (stuff_error) state <= S_DONE;
                if (destuffed_valid) begin
                    rx_crc    <= {rx_crc[13:0], destuffed_bit};
                    field_cnt <= field_cnt + 1'b1;
                    if (field_cnt == 7'd14) begin
                        state     <= S_ACK_EOF;
                        field_cnt <= 0;
                    end
                end
            end

            // ACK slot + ACK delimiter + EOF
            S_ACK_EOF: begin
                if (sample_tick) begin
                    field_cnt <= field_cnt + 1'b1;
                    if (sampled_bit == 1'b1) begin
                        // recessive
                        if (field_cnt >= 7'd8) begin
                            if (crc_reg != rx_crc)
                                crc_error <= 1'b1;

                            out_idx    <= 5'd0;
                            byte_valid <= 1'b0;
                            state      <= S_EMIT;
                        end
                    end else begin
                        if (field_cnt <= 7'd2)
                            field_cnt <= field_cnt;
                    end
                end
                if (idle_timer >= (bit_time << 5))
                    state <= S_EMIT;
            end

            // emit frame bytes directly from registers
            S_EMIT: begin
                if (!byte_valid) begin
                    if (out_idx < out_len) begin
                        byte_data  <= emit_byte_mux(out_idx,
                                                    is_extended, is_rtr,
                                                    frame_id, dlc,
                                                    payload, rx_crc);
                        byte_valid <= 1'b1;
                        out_idx    <= out_idx + 5'd1;
                    end else begin
                        state <= S_DONE;
                    end
                end else if (byte_ready) begin
                    byte_valid <= 1'b0;
                end
            end

            // done
            S_DONE: begin
                byte_valid <= 1'b0;
                done       <= 1'b1;
                busy       <= 1'b0;
                if (!enable)
                    state <= S_OFF;
            end

            default: state <= S_OFF;

            endcase

            if (!enable && state != S_OFF) begin
                state      <= S_OFF;
                busy       <= 1'b0;
                done       <= 1'b0;
                byte_valid <= 1'b0;
            end
        end
    end

endmodule