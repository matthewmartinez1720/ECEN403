`timescale 1ns / 1ps

module spi_led_echo_slave (
    input  wire       clk,       // 12 MHz onboard clock
    input  wire       rst,       // BTN0

    input  wire       spi_sclk,
    input  wire       spi_mosi,
    input  wire       spi_cs_n,
    output wire       spi_miso,

    output reg [3:0]  led
);

    // Synchronizers for asynchronous external SPI signals
    reg [1:0] sclk_sync;
    reg [1:0] mosi_sync;
    reg [1:0] cs_sync;

    always @(posedge clk) begin
        if (rst) begin
            sclk_sync <= 2'b00;
            mosi_sync <= 2'b00;
            cs_sync   <= 2'b11;
        end else begin
            sclk_sync <= {sclk_sync[0], spi_sclk};
            mosi_sync <= {mosi_sync[0], spi_mosi};
            cs_sync   <= {cs_sync[0],   spi_cs_n};
        end
    end

    wire sclk_rise = (sclk_sync == 2'b01);
    wire sclk_fall = (sclk_sync == 2'b10);
    wire cs_active = (cs_sync[1] == 1'b0);
    wire cs_fall   = (cs_sync == 2'b10); // 1 -> 0, start of transaction
    wire cs_rise   = (cs_sync == 2'b01); // 0 -> 1, end of transaction

    wire mosi_sample = mosi_sync[1];


    // SPI data path
    reg [15:0] rx_shift;
    reg [15:0] tx_shift;
    reg [15:0] last_cmd;
    reg [4:0]  bit_cnt;
    reg        miso_reg;

    assign spi_miso = miso_reg;

    always @(posedge clk) begin
        if (rst) begin
            rx_shift <= 16'h0000;
            tx_shift <= 16'h0000;
            last_cmd <= 16'h0000;
            bit_cnt  <= 5'd0;
            miso_reg <= 1'b0;
            led      <= 4'b0010;
        end else begin
            // Start of SPI transaction: preload response
            if (cs_fall) begin
                tx_shift <= last_cmd;
                miso_reg <= last_cmd[15];
                bit_cnt  <= 5'd0;
            end

            // While CS is active do SPI Mode 0 behavior
            if (cs_active) begin
                // Sample MOSI on rising edge
                if (sclk_rise) begin
                    rx_shift <= {rx_shift[14:0], mosi_sample};

                    if (bit_cnt == 5'd15) begin
                        last_cmd <= {rx_shift[14:0], mosi_sample};

                        case ({rx_shift[14:0], mosi_sample})
                            16'h0000: led <= 4'b0001;
                            16'h0001: led <= 4'b0010;
                            16'h0002: led <= 4'b0100;
                            16'h0003: led <= 4'b1000;
                            default:  led <= led;
                        endcase

                        bit_cnt <= 5'd0;
                    end else begin
                        bit_cnt <= bit_cnt + 5'd1;
                    end
                end

                // Shift next MISO bit out on falling edge
                if (sclk_fall) begin
                    tx_shift <= {tx_shift[14:0], 1'b0};
                    miso_reg <= tx_shift[14];
                end
            end

            // End of transaction cleanup
            if (cs_rise) begin
                bit_cnt <= 5'd0;
            end
        end
    end

endmodule
