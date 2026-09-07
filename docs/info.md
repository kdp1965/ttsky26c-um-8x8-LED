<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

The design holds a 64-pixel framebuffer (8 rows x 8 columns, 8 bits per pixel) in flip-flops and drives it out
in one of two modes selected by a control register:

* **Matrix mode** (default after reset): scans a plain 8x8 LED matrix one row at a time with 8-bit PWM per pixel.
* **WS2812 mode**: streams the 64 pixels as a serial WS2812 / NeoPixel frame on `uo[0]`, treating each pixel
  byte as packed 3-2-3 RGB.

### Loading pixels and registers over SPI

Data is written over SPI (mode 0, MSB first) on `SPI_SCLK`, `SPI_MOSI` and `SPI_CS_N`. While `SPI_CS_N` is low,
the first byte received is an address and every following byte is data written at that address, after which the
address auto-increments.

* Address bit 7 = 0: the low 6 bits select a **framebuffer pixel** 0..63 (`row * 8 + col`). The address wraps
  from 63 back to 0. A full frame is one transfer of 65 bytes: `0x00` followed by 64 pixels.
* Address bit 7 = 1: the low 6 bits select a **control register**:

| Register | Address byte | Reset | Function |
|---|---|---|---|
| `CTRL`   | `0x80` | `0x00` | bit 0: 1 = WS2812 mode, 0 = matrix mode. bit 1: colour order, 0 = GRB (WS2812 native), 1 = RGB. |
| `WS_T0H` | `0x81` | `4`    | Clocks of high time for a WS2812 "0" bit. A "1" bit is high for 2 x `WS_T0H`; the bit period is 3 x `WS_T0H`. |

For example the transfer `0x80, 0x01` switches to WS2812 mode, and `0x81, 0x08` sets the bit timing for a
20 MHz clock. The SPI inputs are synchronised to the core clock, so keep SCLK below about one eighth of the core
clock.

### Matrix mode

A free-running 8-bit PWM counter advances on every *tick*. A column output is driven high while its pixel value
is greater than the PWM counter, so a value of 0 is always off and 255 is on for 255/256 of the row period. When
the counter wraps, the active row advances. One full frame takes 8 x 256 = 2048 ticks. The `SPEED[1:0]` inputs
set the tick rate to every 1, 8, 64 or 512 clocks, so the refresh rate can be kept in a sensible range for any
clock between about 1 MHz and 50 MHz. At 10 MHz and `SPEED = 1` the frame rate is roughly 610 Hz.

`COL[7:0]` on the dedicated outputs are active-high column drives (LED anodes). `ROW0_N..ROW7_N` on the
bidirectional pins (configured as outputs) are active-low row selects, exactly one low at a time (LED cathodes).
`COL_INV` and `ROW_INV` invert the respective outputs for matrices or driver boards with the opposite polarity.
`BLANK` forces all columns off while the scan keeps running. Both output banks are registered and update on the
same clock edge, so there is no ghosting between rows.

### WS2812 mode

Each pixel byte is unpacked as R = bits [7:5], G = bits [4:3], B = bits [2:0], and those bits become the high bits
of the three 8-bit colour values (the low bits are zero, so full scale is 224 / 192 / 224). The 64 LEDs are sent
in framebuffer order (pixel 0 first) as 24-bit words, MSB first. WS2812 / WS2812B parts expect the colours in
**GRB** order, which is the default; set `CTRL` bit 1 for RGB-ordered parts such as some WS2811 strips.

The stream repeats continuously: 64 LEDs, then a reset gap of 256 bit periods with the line low (about 300 us
at the default timing), then the next frame. With the default `WS_T0H = 4` at 10 MHz the waveform is
400 ns / 800 ns high for a 0 / 1 bit with a 1.2 us bit period, which is inside the WS2812B tolerance. For other
clocks set `WS_T0H` to 0.4 us worth of clocks (8 at 20 MHz, 10 at 25 MHz, 20 at 50 MHz).

In this mode `uo[0]` is the data output and `uo[7:1]` are held low. The row pins are held in their deselected
state. `BLANK` sends all-zero pixels, and `COL_INV` inverts the data line, which lets a single inverting
transistor act as the 3.3 V to 5 V level shifter.

The framebuffer clears to all-off on reset, and switching modes does not disturb it.

## How to test

1. Apply a clock (10 MHz is the reference; anything from 1 MHz to 50 MHz works) and pulse `rst_n` low.
2. Tie `BLANK`, `COL_INV` and `ROW_INV` low. For a matrix at 10 MHz set `SPEED` to `01`.
3. Send an SPI transfer of 65 bytes: `0x00`, then 64 brightness values in row-major order. From the demo
   board's RP2040 you can bit-bang the three input pins or use a hardware SPI peripheral in mode 0 at 1 MHz.
4. The pattern should appear on the matrix immediately. Send shorter transfers with a non-zero start address to
   update individual pixels.
5. For a WS2812 panel or strip, connect its DIN to `uo[0]` and send `0x80, 0x01`. Each pixel byte is now
   3-2-3 RGB, e.g. `0xE0` is red, `0x18` is green, `0x07` is blue, `0xFF` is white. Send `0x80, 0x00` to return
   to matrix mode.
6. Raise `BLANK` to blank the display, or toggle `COL_INV` / `ROW_INV` if the LEDs light with inverted
   polarity.

Without LEDs you can watch the outputs on a logic analyser: in matrix mode `ROW*_N` walks a single low through
the eight row pins and each `COL*` pin shows a PWM waveform whose duty matches the pixel value; in WS2812 mode
`uo[0]` shows 64 x 24 pulses followed by a long low gap.

The cocotb test in `test/test.py` bit-bangs SPI writes and samples the outputs for a full frame to check every
pixel's PWM duty, the partial-write / address-wrap behaviour, `BLANK`, the inversion bits and the `SPEED`
prescaler. It then enables WS2812 mode and decodes the serial waveform bit by bit to check the colour packing,
GRB / RGB ordering, programmable timing, inverted output and blanking.

## External hardware

* An 8x8 LED matrix (e.g. a 1088AS / 788BS style common-cathode-row module, or a 788AS common-anode type using the
  invert bits), with series resistors on the eight column lines. The Tiny Tapeout pads can only source or sink a
  few milliamps, so for a bright display add a transistor or driver IC on the row cathodes and a high-side switch
  per column; the polarity bits accommodate either.
* Or a WS2812 / WS2812B / NeoPixel 8x8 panel or 64-LED strip on `uo[0]` with its own 5 V supply. Most WS2812
  parts accept a 3.3 V data input directly; if not, use a level shifter (a single inverting transistor works with
  `COL_INV` set).
* A microcontroller or the demo board's RP2040 to provide the SPI stream.
