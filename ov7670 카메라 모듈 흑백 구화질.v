`timescale 1ns / 1ps

module ultrasonic_test (
    input  clk,
    input  [1:0] btn,
    inout  [7:0] ja,
    output reg [3:0] led,

    output pio1, output pio2, output pio3,
    output pio4, output pio5, output pio6,
    output uart_tx,

    input  [7:0] cam_d,
    input        cam_pclk,
    output       cam_xclk,
    input        cam_href,
    input        cam_vsync,
    output       cam_sioc,
    inout        cam_siod,
    output       cam_reset,
    output       cam_pwdn
);

parameter TRIG_PULSE = 24'd120;
parameter SENSOR_GAP = 24'd70_000;

parameter CLOSE_THR = 24'd7_060;
parameter MID_THR   = 24'd14_120;
parameter FAR_THR   = 24'd21_180;

// 12 MHz / 115200 ~= 104
parameter CLK_PER_BIT = 13'd104;

reg trig0 = 0;
wire echo0 = ja[1];

assign ja[0] = trig0;
assign ja[2] = 1'bz;
assign ja[3] = 1'bz;
assign ja[4] = 1'bz;
assign ja[5] = 1'bz;
assign ja[6] = 1'bz;
assign ja[7] = 1'bz;

wire buzzer0;
wire servo0;

assign pio1 = buzzer0;
assign pio2 = 1'b0;
assign pio3 = 1'b0;

assign pio4 = servo0;
assign pio5 = 1'b0;
assign pio6 = 1'b0;

// =====================================================
// OV7670 basic bring-up
// =====================================================
assign cam_xclk  = clk;    // 12 MHz XCLK to OV7670
assign cam_reset = 1'b1;   // 1 = run
assign cam_pwdn  = 1'b0;   // 0 = run

assign cam_sioc = 1'b1;
assign cam_siod = 1'bz;

wire [7:0] angle0;

// =====================================================
// Sensor0 Trigger Only
// =====================================================
reg [23:0] seq_cnt  = 0;

always @(posedge clk) begin
    trig0 <= 0;

    if (seq_cnt < TRIG_PULSE)
        trig0 <= 1;

    if (seq_cnt >= SENSOR_GAP - 1)
        seq_cnt <= 0;
    else
        seq_cnt <= seq_cnt + 1;
end

// =====================================================
// Echo Measure
// =====================================================
wire [23:0] echo_time0;
wire [15:0] dist0;

echo_measure U_ECHO0(
    .clk(clk),
    .trig(trig0),
    .echo(echo0),
    .echo_time(echo_time0),
    .dist_cm(dist0)
);

// =====================================================
// Buzzer / Servo
// =====================================================
buzzer_ctrl U_BUZ0(
    .clk(clk),
    .echo_time(echo_time0),
    .buzzer(buzzer0)
);

servo_sweep_angle U_SERVO0(
    .clk(clk),
    .servo_pwm(servo0),
    .angle(angle0)
);

// =====================================================
// Camera UART Stream
// Sends:
// AA 55 50 3C + 4800 bytes
// 50 hex = 80, 3C hex = 60
// =====================================================
wire uart_start;
wire [7:0] uart_data;
wire uart_busy;
wire cam_activity;

camera_frame_sender U_CAM_TX(
    .clk(clk),
    .cam_d(cam_d),
    .cam_pclk(cam_pclk),
    .cam_href(cam_href),
    .cam_vsync(cam_vsync),
    .uart_busy(uart_busy),
    .uart_start(uart_start),
    .uart_data(uart_data),
    .activity(cam_activity)
);

uart_tx_module #(.CLK_PER_BIT(CLK_PER_BIT)) U_UART(
    .clk(clk),
    .start(uart_start),
    .data(uart_data),
    .tx(uart_tx),
    .busy(uart_busy)
);

// =====================================================
// LED
// led[2:0] = ultrasonic distance
// led[3]   = camera activity
// =====================================================
always @(posedge clk) begin
    if      (echo_time0 == 0)        led[2:0] <= 3'b000;
    else if (echo_time0 < CLOSE_THR) led[2:0] <= 3'b001;
    else if (echo_time0 < MID_THR)   led[2:0] <= 3'b011;
    else if (echo_time0 < FAR_THR)   led[2:0] <= 3'b111;
    else                             led[2:0] <= 3'b111;

    led[3] <= cam_activity;
end

endmodule

// =====================================================
// Camera Frame Sender
// Captures rough reduced grayscale/raw frame.
// =====================================================
module camera_frame_sender(
    input clk,

    input [7:0] cam_d,
    input       cam_pclk,
    input       cam_href,
    input       cam_vsync,

    input       uart_busy,
    output reg  uart_start = 0,
    output reg [7:0] uart_data = 0,

    output reg activity = 0
);

parameter OUT_W = 80;
parameter OUT_H = 60;
parameter FRAME_SIZE = 4800;

// Rough downsample from camera byte stream.
parameter X_SKIP = 16;
parameter Y_SKIP = 8;

reg [7:0] frame_mem [0:FRAME_SIZE-1];

reg [2:0] pclk_sync = 0;
reg [2:0] href_sync = 0;
reg [2:0] vsync_sync = 0;
reg [7:0] d_sync = 0;

wire pclk_rise  = (pclk_sync[2:1] == 2'b01);
wire href_now   = href_sync[2];
wire href_rise  = (href_sync[2:1] == 2'b01);
wire href_fall  = (href_sync[2:1] == 2'b10);
wire vsync_rise = (vsync_sync[2:1] == 2'b01);

reg [12:0] wr_addr = 0;
reg [12:0] rd_addr = 0;

reg [7:0] x_skip_cnt = 0;
reg [3:0] y_skip_cnt = 0;
reg [6:0] out_x = 0;
reg [6:0] out_y = 0;

reg capture_line = 0;
reg [23:0] activity_timeout = 0;

reg [3:0] state = 0;
reg [2:0] header_idx = 0;

always @(posedge clk) begin
    pclk_sync  <= {pclk_sync[1:0], cam_pclk};
    href_sync  <= {href_sync[1:0], cam_href};
    vsync_sync <= {vsync_sync[1:0], cam_vsync};
    d_sync <= cam_d;

    if ((pclk_sync[2] ^ pclk_sync[1]) ||
        (href_sync[2] ^ href_sync[1]) ||
        (vsync_sync[2] ^ vsync_sync[1])) begin
        activity_timeout <= 24'd12_000_000;
    end
    else if (activity_timeout != 0) begin
        activity_timeout <= activity_timeout - 1;
    end

    activity <= (activity_timeout != 0);
end

always @(posedge clk) begin
    uart_start <= 0;

    case (state)
        // Wait for frame start
        0: begin
            if (vsync_rise) begin
                wr_addr <= 0;
                out_y <= 0;
                y_skip_cnt <= 0;
                capture_line <= 0;
                state <= 1;
            end
        end

        // Capture reduced frame
        1: begin
            if (href_rise) begin
                x_skip_cnt <= 0;
                out_x <= 0;

                if (y_skip_cnt == 0 && out_y < OUT_H)
                    capture_line <= 1;
                else
                    capture_line <= 0;
            end

            if (href_now && capture_line && wr_addr < FRAME_SIZE) begin
    if (x_skip_cnt >= X_SKIP - 1) begin
        x_skip_cnt <= 0;

        if (out_x < OUT_W) begin
            frame_mem[wr_addr] <= {1'b1, cam_vsync, cam_pclk, d_sync[4:0]};
            wr_addr <= wr_addr + 1;
            out_x <= out_x + 1;
        end
    end
    else begin
        x_skip_cnt <= x_skip_cnt + 1;
    end
end


            if (href_fall) begin
                if (capture_line && out_y < OUT_H)
                    out_y <= out_y + 1;

                capture_line <= 0;

                if (y_skip_cnt >= Y_SKIP - 1)
                    y_skip_cnt <= 0;
                else
                    y_skip_cnt <= y_skip_cnt + 1;

                if (out_y >= OUT_H - 1 && capture_line) begin
                    rd_addr <= 0;
                    header_idx <= 0;
                    state <= 2;
                end
            end
        end

        // Header: AA 55 80 60
        2: begin
            if (!uart_busy && !uart_start) begin
                uart_start <= 1;

                case (header_idx)
                    0: uart_data <= 8'hAA;
                    1: uart_data <= 8'h55;
                    2: uart_data <= 8'd80;
                    3: uart_data <= 8'd60;
                    default: uart_data <= 8'h00;
                endcase

                if (header_idx == 3)
                    state <= 3;
                else
                    header_idx <= header_idx + 1;
            end
        end

        // Pixel bytes
        3: begin
            if (!uart_busy && !uart_start) begin
                uart_start <= 1;
                uart_data <= frame_mem[rd_addr];

                if (rd_addr >= FRAME_SIZE - 1)
                    state <= 0;
                else
                    rd_addr <= rd_addr + 1;
            end
        end

        default: state <= 0;
    endcase
end

endmodule

// =====================================================
// Echo Measure Module
// =====================================================
module echo_measure(
    input clk,
    input trig,
    input echo,
    output reg [23:0] echo_time = 0,
    output reg [15:0] dist_cm = 0
);

reg [23:0] echo_cnt = 0;
reg measuring = 0;

always @(posedge clk) begin
    if (trig) begin
        echo_cnt  <= 0;
        measuring <= 0;
    end
    else if (echo) begin
        if (echo_cnt < 24'd1_000_000)
            echo_cnt <= echo_cnt + 1;
        measuring <= 1;
    end
    else if (measuring) begin
        echo_time <= echo_cnt;
        dist_cm   <= echo_cnt / 706;
        echo_cnt  <= 0;
        measuring <= 0;
    end
end

endmodule

// =====================================================
// Buzzer Module
// =====================================================
module buzzer_ctrl(
    input clk,
    input [23:0] echo_time,
    output reg buzzer = 0
);

parameter CLOSE_THR = 24'd7_060;
parameter MID_THR   = 24'd14_120;
parameter BEEP_FAST = 23'd600;
parameter BEEP_SLOW = 23'd6_000;

reg [22:0] beep_cnt = 0;
reg [22:0] beep_thr = BEEP_FAST;
reg beep_en = 0;

always @(posedge clk) begin
    if (echo_time > 0 && echo_time < CLOSE_THR) begin
        beep_en <= 1;
        beep_thr <= BEEP_FAST;
    end
    else if (echo_time > 0 && echo_time < MID_THR) begin
        beep_en <= 1;
        beep_thr <= BEEP_SLOW;
    end
    else begin
        beep_en <= 0;
    end

    if (!beep_en) begin
        buzzer <= 0;
        beep_cnt <= 0;
    end
    else if (beep_cnt >= beep_thr) begin
        beep_cnt <= 0;
        buzzer <= ~buzzer;
    end
    else begin
        beep_cnt <= beep_cnt + 1;
    end
end

endmodule

// =====================================================
// Servo Sweep + Angle Module
// =====================================================
module servo_sweep_angle(
    input clk,
    output reg servo_pwm = 0,
    output reg [7:0] angle = 0
);

parameter SERVO_PERIOD = 18'd240_000;
parameter SERVO_LEFT   = 18'd9_000;
parameter SERVO_RIGHT  = 18'd27_000;

parameter STEP_FAST = 24'd1_200;
parameter STEP_MID  = 24'd4_000;
parameter STEP_SLOW = 24'd13_000;

parameter SLOW_ZONE = 18'd2_000;
parameter MID_ZONE  = 18'd5_000;

reg [17:0] servo_cnt   = 0;
reg [17:0] servo_width = SERVO_LEFT;
reg [23:0] step_cnt    = 0;
reg [23:0] step_delay  = STEP_FAST;
reg        direction   = 0;

always @(posedge clk) begin
    if (servo_cnt >= SERVO_PERIOD - 1)
        servo_cnt <= 0;
    else
        servo_cnt <= servo_cnt + 1;

    servo_pwm <= (servo_cnt < servo_width);
end

always @(posedge clk) begin
    if (direction == 0) begin
        if      (servo_width >= SERVO_RIGHT - SLOW_ZONE) step_delay <= STEP_SLOW;
        else if (servo_width >= SERVO_RIGHT - MID_ZONE)  step_delay <= STEP_MID;
        else                                             step_delay <= STEP_FAST;
    end
    else begin
        if      (servo_width <= SERVO_LEFT + SLOW_ZONE) step_delay <= STEP_SLOW;
        else if (servo_width <= SERVO_LEFT + MID_ZONE)  step_delay <= STEP_MID;
        else                                            step_delay <= STEP_FAST;
    end

    if (step_cnt >= step_delay) begin
        step_cnt <= 0;

        if (direction == 0) begin
            if (servo_width < SERVO_RIGHT)
                servo_width <= servo_width + 1;
            else
                direction <= 1;
        end
        else begin
            if (servo_width > SERVO_LEFT)
                servo_width <= servo_width - 1;
            else
                direction <= 0;
        end

        angle <= ((servo_width - SERVO_LEFT) * 120) / (SERVO_RIGHT - SERVO_LEFT);
    end
    else begin
        step_cnt <= step_cnt + 1;
    end
end

endmodule

// =====================================================
// UART TX Module
// =====================================================
module uart_tx_module #(
    parameter CLK_PER_BIT = 1250
)(
    input clk,
    input start,
    input [7:0] data,
    output reg tx = 1,
    output reg busy = 0
);

reg [12:0] clk_cnt = 0;
reg [3:0]  bit_idx = 0;
reg [9:0]  tx_data = 10'b1111111111;

always @(posedge clk) begin
    if (!busy) begin
        tx <= 1;

        if (start) begin
            busy <= 1;
            clk_cnt <= 0;
            bit_idx <= 0;
            tx_data <= {1'b1, data, 1'b0};
        end
    end
    else begin
        if (clk_cnt >= CLK_PER_BIT - 1) begin
            clk_cnt <= 0;
            tx <= tx_data[bit_idx];

            if (bit_idx >= 9) begin
                busy <= 0;
                bit_idx <= 0;
            end
            else begin
                bit_idx <= bit_idx + 1;
            end
        end
        else begin
            clk_cnt <= clk_cnt + 1;
        end
    end
end

endmodule
