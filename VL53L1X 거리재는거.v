`timescale 1ns / 1ps

module ultrasonic_uart_test (
    input        clk,
    input  [1:0] btn,
    output reg [3:0] led,
    output       uart_tx,

    output       ultra_trig,
    input        ultra_echo
);

parameter TRIG_PULSE    = 24'd120;     // 10 us at 12 MHz
parameter MEASURE_GAP   = 24'd720000;  // 60 ms at 12 MHz
parameter ECHO_TIMEOUT  = 24'd300000;  // 25 ms
parameter CLK_PER_BIT   = 13'd104;     // 115200 baud at 12 MHz

parameter CLOSE_THR_CM = 16'd10;
parameter MID_THR_CM   = 16'd20;
parameter FAR_THR_CM   = 16'd30;

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
wire tx_active;

distance_uart_sender U_SEND (
    .clk(clk),
    .rst(btn[0]),
    .sample_valid(sample_valid),
    .dist_cm(dist_cm),
    .uart_busy(uart_busy),
    .uart_start(uart_start),
    .uart_data(uart_data),
    .active(tx_active)
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
    if (dist_cm == 0) begin
        led[2:0] <= 3'b000;
    end else if (dist_cm < CLOSE_THR_CM) begin
        led[2:0] <= 3'b001;
    end else if (dist_cm < MID_THR_CM) begin
        led[2:0] <= 3'b011;
    end else if (dist_cm < FAR_THR_CM) begin
        led[2:0] <= 3'b111;
    end else begin
        led[2:0] <= 3'b100;
    end

    led[3] <= tx_active;
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
wire echo_fall = (echo_sync[2:1] == 2'b10);

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
    input        uart_busy,
    output reg   uart_start = 0,
    output reg [7:0] uart_data = 0,
    output reg   active = 0
);

reg [3:0] state = 0;
reg [15:0] latched = 0;

reg [3:0] thousands = 0;
reg [3:0] hundreds  = 0;
reg [3:0] tens      = 0;
reg [3:0] ones      = 0;

always @(posedge clk) begin
    uart_start <= 0;

    if (rst) begin
        state <= 0;
        active <= 0;
    end else begin
        case (state)
            0: begin
                active <= 0;
                if (sample_valid) begin
                    latched <= dist_cm;
                    thousands <= (dist_cm / 1000) % 10;
                    hundreds  <= (dist_cm / 100)  % 10;
                    tens      <= (dist_cm / 10)   % 10;
                    ones      <=  dist_cm         % 10;
                    state <= 1;
                    active <= 1;
                end
            end

            1: if (!uart_busy) begin uart_data <= "D"; uart_start <= 1; state <= 2; end
            2: if (!uart_busy) begin uart_data <= ":"; uart_start <= 1; state <= 3; end
            3: if (!uart_busy) begin uart_data <= 8'd48 + thousands; uart_start <= 1; state <= 4; end
            4: if (!uart_busy) begin uart_data <= 8'd48 + hundreds;  uart_start <= 1; state <= 5; end
            5: if (!uart_busy) begin uart_data <= 8'd48 + tens;      uart_start <= 1; state <= 6; end
            6: if (!uart_busy) begin uart_data <= 8'd48 + ones;      uart_start <= 1; state <= 7; end
            7: if (!uart_busy) begin uart_data <= "c"; uart_start <= 1; state <= 8; end
            8: if (!uart_busy) begin uart_data <= "m"; uart_start <= 1; state <= 9; end
            9: if (!uart_busy) begin uart_data <= 8'h0A; uart_start <= 1; state <= 0; end

            default: state <= 0;
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
