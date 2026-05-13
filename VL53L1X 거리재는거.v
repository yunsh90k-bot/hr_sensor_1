`timescale 1ns / 1ps

module ultrasonic_uart_test (
    input        clk,
    input  [1:0] btn,
    output reg [3:0] led,
    output       uart_tx,

    output       ultra_trig,
    input        ultra_echo,

    output       tof_xsdn,
    input        tof_int,
    inout        tof_sda,
    inout        tof_scl
);

parameter TRIG_PULSE  = 24'd120;
parameter MEASURE_GAP = 24'd720000;
parameter CLK_PER_BIT = 13'd104;

assign tof_xsdn = 1'b1;

wire tof_sda_low;
wire tof_scl_low;
wire tof_found;

assign tof_sda = tof_sda_low ? 1'b0 : 1'bz;
assign tof_scl = tof_scl_low ? 1'b0 : 1'bz;

vl53l1x_ack_probe U_TOF (
    .clk(clk),
    .rst(btn[0]),
    .sda_in(tof_sda),
    .sda_drive_low(tof_sda_low),
    .scl_drive_low(tof_scl_low),
    .found(tof_found)
);

reg [23:0] trig_cnt = 0;
reg trig_reg = 0;

assign ultra_trig = trig_reg;

always @(posedge clk) begin
    if (btn[0]) begin
        trig_cnt <= 0;
        trig_reg <= 0;
    end else begin
        if (trig_cnt >= MEASURE_GAP - 1)
            trig_cnt <= 0;
        else
            trig_cnt <= trig_cnt + 1;

        trig_reg <= (trig_cnt < TRIG_PULSE);
    end
end

wire [23:0] echo_time;
wire [15:0] dist_cm;
wire        sample_valid;

echo_measure U_ECHO (
    .clk(clk),
    .rst(btn[0]),
    .trig(trig_reg),
    .echo_async(ultra_echo),
    .echo_time(echo_time),
    .dist_cm(dist_cm),
    .sample_valid(sample_valid)
);

wire uart_start;
wire [7:0] uart_data;
wire uart_busy;

distance_uart_sender U_SEND (
    .clk(clk),
    .rst(btn[0]),
    .sample_valid(sample_valid),
    .dist_cm(dist_cm),
    .tof_found(tof_found),
    .tof_int(tof_int),
    .uart_busy(uart_busy),
    .uart_start(uart_start),
    .uart_data(uart_data)
);

uart_tx_module #(
    .CLK_PER_BIT(CLK_PER_BIT)
) U_UART (
    .clk(clk),
    .rst(btn[0]),
    .start(uart_start),
    .data(uart_data),
    .tx(uart_tx),
    .busy(uart_busy)
);

always @(posedge clk) begin
    if (dist_cm == 0)
        led[2:0] <= 3'b000;
    else if (dist_cm < 16'd10)
        led[2:0] <= 3'b001;
    else if (dist_cm < 16'd20)
        led[2:0] <= 3'b011;
    else if (dist_cm < 16'd30)
        led[2:0] <= 3'b111;
    else
        led[2:0] <= 3'b100;

    led[3] <= tof_found;
end

endmodule

module echo_measure (
    input        clk,
    input        rst,
    input        trig,
    input        echo_async,
    output reg [23:0] echo_time = 0,
    output reg [15:0] dist_cm = 0,
    output reg        sample_valid = 0
);

parameter ECHO_TIMEOUT = 24'd300000;

reg [2:0] echo_sync = 0;
reg [23:0] echo_cnt = 0;
reg measuring = 0;

wire echo_now  = echo_sync[2];
wire echo_rise = (echo_sync[2:1] == 2'b01);

always @(posedge clk) begin
    echo_sync <= {echo_sync[1:0], echo_async};
    sample_valid <= 0;

    if (rst || trig) begin
        echo_cnt <= 0;
        measuring <= 0;
    end else if (echo_rise) begin
        echo_cnt <= 0;
        measuring <= 1;
    end else if (measuring) begin
        if (echo_now && echo_cnt < ECHO_TIMEOUT) begin
            echo_cnt <= echo_cnt + 1;
        end else begin
            echo_time <= echo_cnt;
            dist_cm <= echo_cnt / 16'd706;
            sample_valid <= 1;
            echo_cnt <= 0;
            measuring <= 0;
        end
    end
end

endmodule

module distance_uart_sender (
    input        clk,
    input        rst,
    input        sample_valid,
    input [15:0] dist_cm,
    input        tof_found,
    input        tof_int,
    input        uart_busy,
    output reg   uart_start = 0,
    output reg [7:0] uart_data = 0
);

reg [4:0] state = 0;
reg [3:0] thousands = 0;
reg [3:0] hundreds  = 0;
reg [3:0] tens      = 0;
reg [3:0] ones      = 0;
reg tof_found_latched = 0;
reg tof_int_latched = 0;

always @(posedge clk) begin
    uart_start <= 0;

    if (rst) begin
        state <= 0;
    end else begin
        case (state)
            0: begin
                if (sample_valid) begin
                    thousands <= (dist_cm / 1000) % 10;
                    hundreds  <= (dist_cm / 100)  % 10;
                    tens      <= (dist_cm / 10)   % 10;
                    ones      <=  dist_cm         % 10;
                    tof_found_latched <= tof_found;
                    tof_int_latched <= tof_int;
                    state <= 1;
                end
            end

            1:  if (!uart_busy) begin uart_data <= "D"; uart_start <= 1; state <= 2; end
            2:  if (!uart_busy) begin uart_data <= ":"; uart_start <= 1; state <= 3; end
            3:  if (!uart_busy) begin uart_data <= 8'd48 + thousands; uart_start <= 1; state <= 4; end
            4:  if (!uart_busy) begin uart_data <= 8'd48 + hundreds;  uart_start <= 1; state <= 5; end
            5:  if (!uart_busy) begin uart_data <= 8'd48 + tens;      uart_start <= 1; state <= 6; end
            6:  if (!uart_busy) begin uart_data <= 8'd48 + ones;      uart_start <= 1; state <= 7; end
            7:  if (!uart_busy) begin uart_data <= "c"; uart_start <= 1; state <= 8; end
            8:  if (!uart_busy) begin uart_data <= "m"; uart_start <= 1; state <= 9; end
            9:  if (!uart_busy) begin uart_data <= " "; uart_start <= 1; state <= 10; end
            10: if (!uart_busy) begin uart_data <= "V"; uart_start <= 1; state <= 11; end
            11: if (!uart_busy) begin uart_data <= "L"; uart_start <= 1; state <= 12; end
            12: if (!uart_busy) begin uart_data <= ":"; uart_start <= 1; state <= 13; end
            13: if (!uart_busy) begin uart_data <= tof_found_latched ? "1" : "0"; uart_start <= 1; state <= 14; end
            14: if (!uart_busy) begin uart_data <= " "; uart_start <= 1; state <= 15; end
            15: if (!uart_busy) begin uart_data <= "I"; uart_start <= 1; state <= 16; end
            16: if (!uart_busy) begin uart_data <= ":"; uart_start <= 1; state <= 17; end
            17: if (!uart_busy) begin uart_data <= tof_int_latched ? "1" : "0"; uart_start <= 1; state <= 18; end
            18: if (!uart_busy) begin uart_data <= 8'h0A; uart_start <= 1; state <= 0; end

            default: state <= 0;
        endcase
    end
end

endmodule

module vl53l1x_ack_probe (
    input clk,
    input rst,
    input sda_in,
    output reg sda_drive_low = 0,
    output reg scl_drive_low = 0,
    output reg found = 0
);

parameter I2C_HALF_CYCLES = 16'd120;
parameter STARTUP_WAIT    = 32'd240000;
parameter RETRY_WAIT      = 32'd12000000;
parameter ADDR_WRITE      = 8'h52;

localparam IDLE       = 4'd0;
localparam START_A    = 4'd1;
localparam START_B    = 4'd2;
localparam BIT_SETUP  = 4'd3;
localparam BIT_HIGH   = 4'd4;
localparam BIT_LOW    = 4'd5;
localparam ACK_SETUP  = 4'd6;
localparam ACK_HIGH   = 4'd7;
localparam ACK_SAMPLE = 4'd8;
localparam STOP_A     = 4'd9;
localparam STOP_B     = 4'd10;
localparam STOP_C     = 4'd11;
localparam CHECK      = 4'd12;

reg [3:0] state = IDLE;
reg [31:0] wait_cnt = STARTUP_WAIT;
reg [3:0] bit_idx = 4'd7;
reg ack_seen = 0;

always @(posedge clk) begin
    if (rst) begin
        sda_drive_low <= 0;
        scl_drive_low <= 0;
        found <= 0;
        ack_seen <= 0;
        bit_idx <= 4'd7;
        wait_cnt <= STARTUP_WAIT;
        state <= IDLE;
    end else if (wait_cnt != 0) begin
        wait_cnt <= wait_cnt - 1;
    end else begin
        case (state)
            IDLE: begin
                sda_drive_low <= 0;
                scl_drive_low <= 0;
                ack_seen <= 0;
                bit_idx <= 4'd7;

                if (!found) begin
                    wait_cnt <= I2C_HALF_CYCLES;
                    state <= START_A;
                end
            end

            START_A: begin
                sda_drive_low <= 0;
                scl_drive_low <= 0;
                wait_cnt <= I2C_HALF_CYCLES;
                state <= START_B;
            end

            START_B: begin
                sda_drive_low <= 1;
                scl_drive_low <= 0;
                wait_cnt <= I2C_HALF_CYCLES;
                state <= BIT_SETUP;
            end

            BIT_SETUP: begin
                scl_drive_low <= 1;
                sda_drive_low <= (ADDR_WRITE[bit_idx] == 1'b0);
                wait_cnt <= I2C_HALF_CYCLES;
                state <= BIT_HIGH;
            end

            BIT_HIGH: begin
                scl_drive_low <= 0;
                wait_cnt <= I2C_HALF_CYCLES;
                state <= BIT_LOW;
            end

            BIT_LOW: begin
                scl_drive_low <= 1;
                wait_cnt <= I2C_HALF_CYCLES;

                if (bit_idx == 0)
                    state <= ACK_SETUP;
                else begin
                    bit_idx <= bit_idx - 1;
                    state <= BIT_SETUP;
                end
            end

            ACK_SETUP: begin
                sda_drive_low <= 0;
                scl_drive_low <= 1;
                wait_cnt <= I2C_HALF_CYCLES;
                state <= ACK_HIGH;
            end

            ACK_HIGH: begin
                scl_drive_low <= 0;
                wait_cnt <= I2C_HALF_CYCLES;
                state <= ACK_SAMPLE;
            end

            ACK_SAMPLE: begin
                ack_seen <= (sda_in == 1'b0);
                scl_drive_low <= 1;
                wait_cnt <= I2C_HALF_CYCLES;
                state <= STOP_A;
            end

            STOP_A: begin
                sda_drive_low <= 1;
                scl_drive_low <= 1;
                wait_cnt <= I2C_HALF_CYCLES;
                state <= STOP_B;
            end

            STOP_B: begin
                scl_drive_low <= 0;
                wait_cnt <= I2C_HALF_CYCLES;
                state <= STOP_C;
            end

            STOP_C: begin
                sda_drive_low <= 0;
                wait_cnt <= I2C_HALF_CYCLES;
                state <= CHECK;
            end

            CHECK: begin
                if (ack_seen)
                    found <= 1;
                else
                    wait_cnt <= RETRY_WAIT;

                state <= IDLE;
            end

            default: state <= IDLE;
        endcase
    end
end

endmodule

module uart_tx_module #(
    parameter CLK_PER_BIT = 104
)(
    input       clk,
    input       rst,
    input       start,
    input [7:0] data,
    output reg  tx = 1,
    output reg  busy = 0
);

reg [12:0] clk_cnt = 0;
reg [3:0] bit_idx = 0;
reg [9:0] tx_shift = 10'b1111111111;

always @(posedge clk) begin
    if (rst) begin
        tx <= 1;
        busy <= 0;
        clk_cnt <= 0;
        bit_idx <= 0;
        tx_shift <= 10'b1111111111;
    end else if (!busy) begin
        tx <= 1;
        if (start) begin
            busy <= 1;
            clk_cnt <= 0;
            bit_idx <= 0;
            tx_shift <= {1'b1, data, 1'b0};
            tx <= 0;
        end
    end else begin
        if (clk_cnt >= CLK_PER_BIT - 1) begin
            clk_cnt <= 0;

            if (bit_idx >= 9) begin
                busy <= 0;
                bit_idx <= 0;
                tx <= 1;
            end else begin
                bit_idx <= bit_idx + 1;
                tx <= tx_shift[bit_idx + 1];
            end
        end else begin
            clk_cnt <= clk_cnt + 1;
        end
    end
end

endmodule
