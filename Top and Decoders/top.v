`timescale 1ns/1ps


module protocol_demo_top #(
    parameter integer CLK_HZ                = 12_000_000,
    parameter integer UART_BAUD             = 115200,
    parameter integer DEMO_BUF_BYTES        = 255,
    parameter integer SPI_MAX_PAYLOAD_BYTES = 240,
    parameter integer SPI_SAMPLE_HZ         = 12_000_000,
    parameter integer SPI_TIMEOUT_CYCLES    = CLK_HZ,
    parameter integer I2C_CAP_BYTES         = 256,
    parameter integer UART_PROBE_MAX_BAUD = 250_000,
    parameter integer UART_PROBE_RAW_BYTES = 2048,
    parameter integer UART_PROBE_OUT_BYTES = 256,
    parameter integer CAN_MAX_FRAME_BYTES  = 32,
    parameter integer CAN_IDLE_TIMEOUT     = CLK_HZ
)(
    input  wire       clk_12mhz,
    input  wire       rst_btn,

    // UART for laptop
    input  wire       uart_rx_i,
    output wire       uart_tx_o,

    // SPI probe
    input  wire       spi_sclk_i,
    input  wire       spi_cs_n_i,
    input  wire       spi_mosi_i,
    input  wire       spi_miso_i,

    // I2C probe
    input  wire       i2c_scl_i,
    input  wire       i2c_sda_i,

    // UART probe
    input  wire  probe_uart_rx_i,

    // CAN probe
    input  wire  probe_can_rx_i,

    output wire [3:0] led_o,
    output wire       rgb_r_o,
    output wire       rgb_g_o,
    output wire       rgb_b_o
);

    wire rst_n = ~rst_btn;


    // UART 
    wire       uart_rx_valid;
    wire [7:0] uart_rx_byte;

    reg        uart_tx_start;
    reg  [7:0] uart_tx_byte;
    wire       uart_tx_busy;

    uart_rx #(
        .CLK_HZ(CLK_HZ),
        .BAUD  (UART_BAUD)
    ) u_uart_rx (
        .clk       (clk_12mhz),
        .rst_n     (rst_n),
        .rx_i      (uart_rx_i),
        .data_valid(uart_rx_valid),
        .data_byte (uart_rx_byte)
    );

    uart_tx #(
        .CLK_HZ(CLK_HZ),
        .BAUD  (UART_BAUD)
    ) u_uart_tx (
        .clk      (clk_12mhz),
        .rst_n    (rst_n),
        .start    (uart_tx_start),
        .data_byte(uart_tx_byte),
        .tx_o     (uart_tx_o),
        .busy     (uart_tx_busy)
    );

    // shared capture RAM
    (* ram_style = "block" *) reg [7:0] capture_mem [0:DEMO_BUF_BYTES-1];

    reg [7:0] capture_count;
    reg       capture_done;
    reg       capture_overflow;

    reg [7:0] cap_rd_addr;
    reg [7:0] cap_rd_data;

    reg [7:0] cap_wr_addr;
    reg [7:0] cap_wr_data;
    reg       cap_wr_en;

    always @(posedge clk_12mhz) begin
        if (cap_wr_en)
            capture_mem[cap_wr_addr] <= cap_wr_data;
        cap_rd_data <= capture_mem[cap_rd_addr];
    end

    wire cap_full = (capture_count == DEMO_BUF_BYTES[7:0]);

   // Protocol selection vars
   reg [2:0] proto_sel;

    wire en_spi      = (proto_sel == 3'd1);
    wire en_i2c      = (proto_sel == 3'd2);
    wire en_uart_ttl = (proto_sel == 3'd3);
    wire en_can      = (proto_sel == 3'd4);
    
    
    // SPI decoder instantiation 
    reg  start_spi;
    wire spi_byte_valid;
    wire [7:0] spi_byte_data;
    wire spi_busy;
    wire spi_done;

    spi_decoder #(
        .CLK_HZ           (CLK_HZ),
        .SAMPLE_HZ        (SPI_SAMPLE_HZ),
        .MAX_PAYLOAD_BYTES(SPI_MAX_PAYLOAD_BYTES),
        .TIMEOUT_CYCLES   (SPI_TIMEOUT_CYCLES)
    ) u_spi_decoder (
        .clk       (clk_12mhz),
        .rst_n     (rst_n),
        .enable    (en_spi),
        .start     (start_spi),
        .sclk      (spi_sclk_i),
        .cs_n      (spi_cs_n_i),
        .mosi      (spi_mosi_i),
        .miso      (spi_miso_i),
        .byte_valid(spi_byte_valid),
        .byte_ready(1'b1),
        .byte_data (spi_byte_data),
        .busy      (spi_busy),
        .done      (spi_done)
    );

    // I2C decoder instantiation
    reg  start_i2c;
    wire i2c_byte_valid;
    wire [7:0] i2c_byte_data;
    wire i2c_busy;
    wire i2c_done;

    i2c_decoder #(
        .CAP_BYTES(I2C_CAP_BYTES)
    ) u_i2c_decoder (
        .clk       (clk_12mhz),
        .rst_n     (rst_n),
        .enable    (en_i2c),
        .start     (start_i2c),
        .scl       (i2c_scl_i),
        .sda       (i2c_sda_i),
        .byte_valid(i2c_byte_valid),
        .byte_ready(1'b1),
        .byte_data (i2c_byte_data),
        .busy      (i2c_busy),
        .done      (i2c_done)
    );


    // UART decoder instantiation
    reg  start_uart;
    wire uart_byte_valid;
    wire [7:0] uart_byte_data;
    wire uart_busy, uart_done;
    
    uart_rx_buffered_autobaud #(
        .CLK_HZ    (CLK_HZ),
        .MAX_BAUD  (UART_PROBE_MAX_BAUD),
        .MIN_BAUD  (1200),
        .OVERSAMPLE(16),
        .RAW_BYTES (UART_PROBE_RAW_BYTES),
        .OUT_BYTES (UART_PROBE_OUT_BYTES)
    ) u_uart_probe (
        .clk(clk_12mhz), .rst_n(rst_n),
        .enable(en_uart_ttl), .start(start_uart),
        .rx(probe_uart_rx_i),
        .byte_valid(uart_byte_valid),
        .byte_ready(1'b1),
        .byte_data(uart_byte_data),
        .busy(uart_busy), .done(uart_done),
        .baud_locked(), .locked_bit_ticks(), .raw_overflow()
    );
    
    
    // CAN decoder instantiation
    reg  start_can;
    wire can_byte_valid;
    wire [7:0] can_byte_data;
    wire can_busy, can_done;
    
    can_decoder #(
        .CLK_HZ         (CLK_HZ),
        .MAX_FRAME_BYTES(CAN_MAX_FRAME_BYTES),
        .IDLE_TIMEOUT   (CAN_IDLE_TIMEOUT)
    ) u_can_decoder (
        .clk(clk_12mhz), .rst_n(rst_n),
        .enable(en_can), .start(start_can),
        .can_rx(probe_can_rx_i),
        .byte_valid(can_byte_valid),
        .byte_ready(1'b1),
        .byte_data(can_byte_data),
        .busy(can_busy), .done(can_done),
        .crc_error()
    );
    
    // Mux selected decoder into shared signals
    wire       dec_byte_valid = en_spi      ? spi_byte_valid  :
                            en_i2c      ? i2c_byte_valid  :
                            en_uart_ttl ? uart_byte_valid :
                            en_can      ? can_byte_valid  : 1'b0;
    wire [7:0] dec_byte_data  = en_spi      ? spi_byte_data   :
                            en_i2c      ? i2c_byte_data   :
                            en_uart_ttl ? uart_byte_data  :
                            en_can      ? can_byte_data   : 8'h00;
    wire       dec_busy       = en_spi      ? spi_busy  :
                            en_i2c      ? i2c_busy  :
                            en_uart_ttl ? uart_busy :
                            en_can      ? can_busy  : 1'b0;
    wire       dec_done       = en_spi      ? spi_done  :
                            en_i2c      ? i2c_done  :
                            en_uart_ttl ? uart_done :
                            en_can      ? can_done  : 1'b0;

    // TX state machine
    localparam [1:0]
        TX_NONE   = 2'd0,
        TX_STATUS = 2'd1,
        TX_READ   = 2'd2;

    localparam [1:0]
        TXS_IDLE      = 2'd0,
        TXS_PULSE     = 2'd1,
        TXS_WAIT_HIGH = 2'd2,
        TXS_WAIT_LOW  = 2'd3;

    reg [1:0] tx_mode;
    reg [1:0] tx_state;
    reg [7:0] tx_index;

    // Delayed start: proto_sel needs one cycle to propagate to en_* before decoder sees start pulse
    reg start_spi_pending;
    reg start_i2c_pending;
    reg start_uart_pending;
    reg start_can_pending;

    always @(posedge clk_12mhz) begin
        if (!rst_n) begin
            start_spi        <= 1'b0;
            start_i2c        <= 1'b0;
            start_uart       <= 1'b0;
            start_can        <= 1'b0;
            
            start_spi_pending  <= 1'b0;
            start_i2c_pending  <= 1'b0;
            start_uart_pending <= 1'b0;
            start_can_pending  <= 1'b0;
            proto_sel          <= 3'd0;

            capture_count    <= 8'd0;
            capture_done     <= 1'b0;
            capture_overflow <= 1'b0;

            cap_rd_addr      <= 8'd0;
            cap_wr_addr      <= 8'd0;
            cap_wr_data      <= 8'h00;
            cap_wr_en        <= 1'b0;

            uart_tx_start    <= 1'b0;
            uart_tx_byte     <= 8'h00;

            tx_mode          <= TX_NONE;
            tx_state         <= TXS_IDLE;
            tx_index         <= 8'd0;
        end else begin
            start_spi      <= 1'b0;
            start_i2c      <= 1'b0;
            start_uart     <= 1'b0;
            start_can      <= 1'b0;
            cap_wr_en      <= 1'b0;
            uart_tx_start  <= 1'b0;

            // Fire actual start pulse one cycle after proto_sel is set
            if (start_spi_pending) begin
                start_spi         <= 1'b1;
                start_spi_pending <= 1'b0;
            end
            if (start_i2c_pending) begin
                start_i2c         <= 1'b1;
                start_i2c_pending <= 1'b0;
            end
            if (start_uart_pending) begin
                start_uart         <= 1'b1;
                start_uart_pending <= 1'b0;
            end
            if (start_can_pending) begin
                start_can         <= 1'b1;
                start_can_pending <= 1'b0;
            end 
            
            // Capture incoming decoder bytes
            if (dec_byte_valid) begin
                if (!cap_full) begin
                    cap_wr_en      <= 1'b1;
                    cap_wr_addr    <= capture_count;
                    cap_wr_data    <= dec_byte_data;
                    capture_count  <= capture_count + 8'd1;
                end else begin
                    capture_overflow <= 1'b1;
                end
            end

            if (dec_done)
                capture_done <= 1'b1;

            // UART command dispatch
            if (uart_rx_valid && (tx_mode == TX_NONE) && (tx_state == TXS_IDLE)) begin
                case (uart_rx_byte)
                    8'h63: begin // clear
                        if (!dec_busy) begin
                            capture_count    <= 8'd0;
                            capture_done     <= 1'b0;
                            capture_overflow <= 1'b0;
                        end
                    end

                    8'h31: begin // start SPI capture
                        if (!dec_busy) begin
                            proto_sel         <= 3'd1;
                            capture_count     <= 8'd0;
                            capture_done      <= 1'b0;
                            capture_overflow  <= 1'b0;
                            start_spi_pending <= 1'b1;
                        end
                    end

                    8'h32: begin // start I2C capture
                        if (!dec_busy) begin
                            proto_sel         <= 3'd2;
                            capture_count     <= 8'd0;
                            capture_done      <= 1'b0;
                            capture_overflow  <= 1'b0;
                            start_i2c_pending <= 1'b1;
                        end
                    end
                    8'h33: begin // start UART capture
                        if (!dec_busy) begin
                            proto_sel          <= 3'd3;
                            capture_count      <= 8'd0;
                            capture_done       <= 1'b0;
                            capture_overflow   <= 1'b0;
                            start_uart_pending <= 1'b1;
                        end
                    end
                    
                    8'h34: begin // start CAN capture
                        if (!dec_busy) begin
                            proto_sel         <= 3'd4;
                            capture_count     <= 8'd0;
                            capture_done      <= 1'b0;
                            capture_overflow  <= 1'b0;
                            start_can_pending <= 1'b1;
                        end
                    end 
                    8'h3F: begin // '?'
                        tx_mode  <= TX_STATUS;
                        tx_state <= TXS_IDLE;
                        tx_index <= 8'd0;
                    end

                    8'h72: begin // 'r'
                        tx_mode     <= TX_READ;
                        tx_state    <= TXS_IDLE;
                        tx_index    <= 8'd0;
                        cap_rd_addr <= 8'd0;
                    end

                    default: begin
                    end
                endcase
            end

            // TX state machine
            case (tx_mode)
                TX_NONE: begin
                    tx_state <= TXS_IDLE;
                end

                TX_STATUS: begin
                    case (tx_state)
                        TXS_IDLE: begin
                            case (tx_index)
                                8'd0: uart_tx_byte <= 8'hA5;
                                8'd1: uart_tx_byte <= {7'd0, dec_busy};
                                8'd2: uart_tx_byte <= {7'd0, capture_done};
                                8'd3: uart_tx_byte <= capture_count;
                                default: uart_tx_byte <= 8'h00;
                            endcase
                            tx_state <= TXS_PULSE;
                        end

                        TXS_PULSE: begin
                            uart_tx_start <= 1'b1;
                            tx_state <= TXS_WAIT_HIGH;
                        end

                        TXS_WAIT_HIGH: begin
                            if (uart_tx_busy)
                                tx_state <= TXS_WAIT_LOW;
                        end

                        TXS_WAIT_LOW: begin
                            if (!uart_tx_busy) begin
                                if (tx_index == 8'd3) begin
                                    tx_mode  <= TX_NONE;
                                    tx_state <= TXS_IDLE;
                                    tx_index <= 8'd0;
                                end else begin
                                    tx_index <= tx_index + 8'd1;
                                    tx_state <= TXS_IDLE;
                                end
                            end
                        end

                        default: tx_state <= TXS_IDLE;
                    endcase
                end

                TX_READ: begin
                    case (tx_state)
                        TXS_IDLE: begin
                            if (tx_index >= capture_count) begin
                                tx_mode  <= TX_NONE;
                                tx_state <= TXS_IDLE;
                                tx_index <= 8'd0;
                            end else begin
                                cap_rd_addr <= tx_index;
                                tx_state    <= TXS_PULSE;
                            end
                        end

                        TXS_PULSE: begin
                            uart_tx_byte  <= cap_rd_data;
                            uart_tx_start <= 1'b1;
                            tx_state      <= TXS_WAIT_HIGH;
                        end

                        TXS_WAIT_HIGH: begin
                            if (uart_tx_busy)
                                tx_state <= TXS_WAIT_LOW;
                        end

                        TXS_WAIT_LOW: begin
                            if (!uart_tx_busy) begin
                                if ((tx_index + 8'd1) >= capture_count) begin
                                    tx_mode  <= TX_NONE;
                                    tx_state <= TXS_IDLE;
                                    tx_index <= 8'd0;
                                end else begin
                                    tx_index    <= tx_index + 8'd1;
                                    cap_rd_addr <= tx_index + 8'd1;  // preload next addr
                                    tx_state    <= TXS_IDLE;
                                end
                            end
                        end

                        default: tx_state <= TXS_IDLE;
                    endcase
                end

                default: begin
                    tx_mode  <= TX_NONE;
                    tx_state <= TXS_IDLE;
                    tx_index <= 8'd0;
                end
            endcase
        end
    end

    assign led_o[0] = 1'b1;
    assign led_o[1] = dec_busy;
    assign led_o[2] = capture_done;
    assign led_o[3] = capture_overflow;

    assign rgb_r_o = ~capture_overflow;
    assign rgb_g_o = ~capture_done;
    assign rgb_b_o = ~dec_busy;

endmodule
