/*
 * Copyright (c) 2026 kpettit-kii
 * SPDX-License-Identifier: Apache-2.0
 *
 * 8x8 LED driver with a 64 x 8-bit framebuffer and two output modes:
 *
 *   Matrix mode (default): row-scanned 8x8 LED matrix, 8-bit PWM per pixel.
 *   WS2812 mode:           64-LED serial stream on uo_out[0], pixel byte is
 *                          packed 3-2-3 RGB (R = [7:5], G = [4:3], B = [2:0]),
 *                          placed in the high bits of each 8-bit colour.
 *
 *   ui_in[0]   SPI SCLK   (mode 0, sampled on rising edge)
 *   ui_in[1]   SPI MOSI   (MSB first)
 *   ui_in[2]   SPI CS_n   (active low, frames a transfer)
 *   ui_in[3]   BLANK      (1 = all LEDs off / all-zero pixels)
 *   ui_in[5:4] SPEED      (matrix mode: PWM tick every 1 / 8 / 64 / 512 clocks)
 *   ui_in[6]   COL_INV    (invert column outputs, or the WS2812 data line)
 *   ui_in[7]   ROW_INV    (invert row outputs)
 *
 *   uo_out[7:0]  matrix: column drive, active high.  WS2812: [0] = DOUT, rest 0
 *   uio_out[7:0] matrix: row drive, active low, one at a time.  WS2812: idle
 *
 * SPI transfer: first byte = address, then any number of data bytes with
 * auto-increment.  Address bit 7 = 0 selects framebuffer pixels 0..63
 * (row*8+col, wraps at 64).  Address bit 7 = 1 selects control registers:
 *   reg 0  CTRL   bit 0: 1 = WS2812 mode, 0 = matrix mode
 *                 bit 1: 1 = send colours in RGB order, 0 = GRB (WS2812)
 *   reg 1  WS_T0H clocks of high time for a WS2812 '0' bit (default 4,
 *                 i.e. 400 ns at 10 MHz). A '1' bit is high for 2*T0H and
 *                 the bit period is 3*T0H. Reset gap = 256 bit periods.
 */

`default_nettype none

module tt_um_kpettit_led_matrix (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // ------------------------------------------------------------------
  // Pin map
  // ------------------------------------------------------------------
  wire       spi_sclk_pin = ui_in[0];
  wire       spi_mosi_pin = ui_in[1];
  wire       spi_csn_pin  = ui_in[2];
  wire       blank        = ui_in[3];
  wire [1:0] speed        = ui_in[5:4];
  wire       col_invert   = ui_in[6];
  wire       row_invert   = ui_in[7];

  // ------------------------------------------------------------------
  // SPI input synchronizers + SCLK rising-edge detect
  // ------------------------------------------------------------------
  reg [1:0] sclk_sync, mosi_sync, csn_sync;
  reg       sclk_prev;

  always @(posedge clk) begin
    if (!rst_n) begin
      sclk_sync <= 2'b00;
      mosi_sync <= 2'b00;
      csn_sync  <= 2'b11;
      sclk_prev <= 1'b0;
    end else begin
      sclk_sync <= {sclk_sync[0], spi_sclk_pin};
      mosi_sync <= {mosi_sync[0], spi_mosi_pin};
      csn_sync  <= {csn_sync[0], spi_csn_pin};
      sclk_prev <= sclk_sync[1];
    end
  end

  wire sclk      = sclk_sync[1];
  wire mosi      = mosi_sync[1];
  wire csn       = csn_sync[1];
  wire sclk_rise = sclk & ~sclk_prev;

  // ------------------------------------------------------------------
  // SPI receiver: byte 0 = address, bytes 1..n = data (auto-increment)
  // ------------------------------------------------------------------
  reg [6:0] shreg;     // 7 bits already received; 8th bit arrives with byte_done
  reg [2:0] bitcnt;
  reg       got_addr;
  reg       reg_space; // 0 = framebuffer, 1 = control registers
  reg [5:0] wr_addr;

  wire       byte_done = sclk_rise & (bitcnt == 3'd7);
  wire [7:0] rx_byte   = {shreg, mosi};
  wire       data_wr   = byte_done & got_addr;
  wire       pix_wr    = data_wr & ~reg_space;
  wire       reg_wr    = data_wr &  reg_space;

  always @(posedge clk) begin
    if (!rst_n) begin
      shreg     <= 7'd0;
      bitcnt    <= 3'd0;
      got_addr  <= 1'b0;
      reg_space <= 1'b0;
      wr_addr   <= 6'd0;
    end else if (csn) begin
      bitcnt   <= 3'd0;
      got_addr <= 1'b0;
    end else if (sclk_rise) begin
      shreg  <= {shreg[5:0], mosi};
      bitcnt <= bitcnt + 3'd1;
      if (byte_done) begin
        if (!got_addr) begin
          got_addr  <= 1'b1;
          wr_addr   <= rx_byte[5:0];
          reg_space <= rx_byte[7];
        end else begin
          wr_addr   <= wr_addr + 6'd1;
        end
      end
    end
  end

  // ------------------------------------------------------------------
  // Control registers
  // ------------------------------------------------------------------
  reg [7:0] ctrl;
  reg [7:0] ws_t0h;

  always @(posedge clk) begin
    if (!rst_n) begin
      ctrl   <= 8'h00;
      ws_t0h <= 8'd4;
    end else if (reg_wr) begin
      case (wr_addr)
        6'd0:    ctrl   <= rx_byte;
        6'd1:    ws_t0h <= rx_byte;
        default: ;
      endcase
    end
  end

  wire ws_mode = ctrl[0];
  wire ws_rgb  = ctrl[1];

  // ------------------------------------------------------------------
  // Framebuffer: 64 pixels x 8 bits
  // ------------------------------------------------------------------
  reg [7:0] fb [0:63];
  integer i;

  always @(posedge clk) begin
    if (!rst_n) begin
      for (i = 0; i < 64; i = i + 1) fb[i] <= 8'd0;
    end else if (pix_wr) begin
      fb[wr_addr] <= rx_byte;
    end
  end

  // ------------------------------------------------------------------
  // Matrix scan timing: prescaler -> 8-bit PWM counter -> row counter
  // ------------------------------------------------------------------
  reg [8:0] presc;
  reg [7:0] pwm_cnt;
  reg [2:0] row;

  wire tick = (speed == 2'd0) ? 1'b1 :
              (speed == 2'd1) ? (presc[2:0] == 3'd0) :
              (speed == 2'd2) ? (presc[5:0] == 6'd0) :
                                (presc[8:0] == 9'd0);

  always @(posedge clk) begin
    if (!rst_n) begin
      presc   <= 9'd0;
      pwm_cnt <= 8'd0;
      row     <= 3'd0;
    end else begin
      presc <= presc + 9'd1;
      if (tick) begin
        pwm_cnt <= pwm_cnt + 8'd1;
        if (pwm_cnt == 8'hFF) row <= row + 3'd1;
      end
    end
  end

  // ------------------------------------------------------------------
  // Shared framebuffer read port: one row of 8 pixels
  //   matrix mode: the scanned row, compared against the PWM counter
  //   WS2812 mode: the row holding the LED currently being sent
  // ------------------------------------------------------------------
  reg [5:0] ws_led;
  wire [2:0] rd_row = ws_mode ? ws_led[5:3] : row;

  reg [63:0] rd_pix;    // pixel c of the selected row at [c*8 +: 8]
  reg [7:0]  col_next;
  integer c;

  always @* begin
    for (c = 0; c < 8; c = c + 1) begin
      rd_pix[c*8 +: 8] = fb[{rd_row, c[2:0]}];
      col_next[c]      = (rd_pix[c*8 +: 8] > pwm_cnt);
    end
  end

  // ------------------------------------------------------------------
  // WS2812 serializer
  // ------------------------------------------------------------------
  wire [7:0]  ws_pix  = blank ? 8'h00 : rd_pix[ws_led[2:0]*8 +: 8];
  wire [7:0]  ws_r    = {ws_pix[7:5], 5'b00000};
  wire [7:0]  ws_g    = {ws_pix[4:3], 6'b000000};
  wire [7:0]  ws_b    = {ws_pix[2:0], 5'b00000};
  wire [23:0] ws_word = ws_rgb ? {ws_r, ws_g, ws_b} : {ws_g, ws_r, ws_b};

  wire [9:0] ws_period = {ws_t0h, 2'b00} - {2'b00, ws_t0h};  // 3 * T0H
  wire [8:0] ws_t1h    = {ws_t0h, 1'b0};                      // 2 * T0H

  reg [9:0]  ws_phase;   // clock within the current bit period
  reg [4:0]  ws_bit;     // 23 .. 0, MSB first
  reg [7:0]  ws_gapcnt;  // bit periods spent in the reset gap
  reg        ws_gap;     // 1 = sending the reset gap (line low)
  reg [23:0] ws_sr;      // current LED word, MSB is the bit being sent

  wire       ws_phase_last = (ws_phase >= ws_period - 10'd1);
  wire [9:0] ws_high       = ws_sr[23] ? {1'b0, ws_t1h} : {2'b00, ws_t0h};
  wire       ws_dout       = ~ws_gap & (ws_phase < ws_high);

  always @(posedge clk) begin
    if (!rst_n || !ws_mode) begin
      ws_phase  <= 10'd0;
      ws_bit    <= 5'd23;
      ws_led    <= 6'd0;
      ws_gapcnt <= 8'd0;
      ws_gap    <= 1'b1;
      ws_sr     <= 24'd0;
    end else begin
      ws_phase <= ws_phase_last ? 10'd0 : ws_phase + 10'd1;

      // Latch the LED's word on the first clock of its first bit. The line
      // is high during phase 0 for both bit values, so the stale MSB in
      // that one clock does not affect the waveform.
      if (!ws_gap && ws_phase == 10'd0 && ws_bit == 5'd23)
        ws_sr <= ws_word;

      if (ws_phase_last) begin
        if (ws_gap) begin
          ws_gapcnt <= ws_gapcnt + 8'd1;
          if (ws_gapcnt == 8'hFF) ws_gap <= 1'b0;
        end else if (ws_bit != 5'd0) begin
          ws_bit <= ws_bit - 5'd1;
          ws_sr  <= {ws_sr[22:0], 1'b0};
        end else begin
          ws_bit <= 5'd23;
          ws_led <= ws_led + 6'd1;
          if (ws_led == 6'd63) begin
            ws_gap    <= 1'b1;
            ws_gapcnt <= 8'd0;
          end
        end
      end
    end
  end

  // ------------------------------------------------------------------
  // Registered outputs
  // ------------------------------------------------------------------
  reg [7:0] col_q;
  reg [7:0] row_q;

  always @(posedge clk) begin
    if (!rst_n) begin
      col_q <= 8'h00;
      row_q <= 8'hFF;
    end else if (ws_mode) begin
      col_q <= {7'b0000000, ws_dout ^ col_invert};
      row_q <= {8{~row_invert}};                       // all rows deselected
    end else begin
      col_q <= (blank ? 8'h00 : col_next) ^ {8{col_invert}};
      row_q <= ~(8'b0000_0001 << row) ^ {8{row_invert}};
    end
  end

  assign uo_out  = col_q;
  assign uio_out = row_q;
  assign uio_oe  = 8'hFF;

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, uio_in, 1'b0};

endmodule
