# SPDX-FileCopyrightText: © 2026 kpettit-kii
# SPDX-License-Identifier: Apache-2.0
#
# cocotb test for the 8x8 LED matrix driver.
#
# The design is driven over bit-banged SPI mode 0 on ui_in[2:0], and the
# outputs are sampled once per clock for a whole scan frame to measure the
# PWM duty of every pixel. Because the column and row output registers update
# together, any window of exactly one frame yields an on-count per pixel that
# equals its 8-bit brightness value.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge

# ui_in configuration bits (SPI pins live in ui_in[2:0])
BLANK   = 1 << 3
SPEED0  = 0 << 4
SPEED1  = 1 << 4
SPEED2  = 2 << 4
COL_INV = 1 << 6
ROW_INV = 1 << 7

FRAME_CYCLES = 8 * 256  # one full scan at SPEED0

# SPI address byte with bit 7 set addresses the control registers
REG_BASE = 0x80
REG_CTRL = 0
REG_T0H  = 1
CTRL_WS  = 1 << 0   # 1 = WS2812 serial mode
CTRL_RGB = 1 << 1   # 1 = RGB colour order, 0 = GRB (WS2812 native)


def drive(dut, cfg, sclk=0, mosi=0, csn=1):
    dut.ui_in.value = (cfg & 0xF8) | (csn << 2) | (mosi << 1) | sclk


async def spi_write(dut, cfg, start_addr, pixels, half=4):
    """Send start address then pixel bytes, MSB first, SPI mode 0."""
    payload = [start_addr & 0xFF] + list(pixels)
    drive(dut, cfg, csn=1)
    await ClockCycles(dut.clk, half)
    drive(dut, cfg, csn=0)
    await ClockCycles(dut.clk, half)
    for b in payload:
        for i in range(7, -1, -1):
            bit = (b >> i) & 1
            drive(dut, cfg, sclk=0, mosi=bit, csn=0)
            await ClockCycles(dut.clk, half)
            drive(dut, cfg, sclk=1, mosi=bit, csn=0)
            await ClockCycles(dut.clk, half)
    drive(dut, cfg, sclk=0, csn=0)
    await ClockCycles(dut.clk, half)
    drive(dut, cfg, csn=1)
    await ClockCycles(dut.clk, half)


def active_row(rows, row_inv=False):
    """Return the single active row index from the 8-bit row output."""
    if row_inv:
        rows ^= 0xFF
    active = [r for r in range(8) if not (rows >> r) & 1]
    assert len(active) == 1, f"row outputs not one-hot: {rows:08b}"
    return active[0]


async def measure_frame(dut, cycles=FRAME_CYCLES, col_inv=False, row_inv=False):
    """Count, per pixel, how many clocks its column is on while its row is active."""
    counts = [[0] * 8 for _ in range(8)]
    for _ in range(cycles):
        await FallingEdge(dut.clk)
        r = active_row(int(dut.uio_out.value), row_inv)
        cols = int(dut.uo_out.value)
        if col_inv:
            cols ^= 0xFF
        for c in range(8):
            if (cols >> c) & 1:
                counts[r][c] += 1
    return counts


async def row_dwell(dut):
    """Number of clocks the active row stays constant."""
    await FallingEdge(dut.clk)
    start = active_row(int(dut.uio_out.value))
    while active_row(int(dut.uio_out.value)) == start:
        await FallingEdge(dut.clk)
    prev = active_row(int(dut.uio_out.value))
    n = 0
    while active_row(int(dut.uio_out.value)) == prev:
        n += 1
        await FallingEdge(dut.clk)
    return n


def expected_from(image):
    return [[image[r * 8 + c] for c in range(8)] for r in range(8)]


async def reset(dut):
    dut.ena.value = 1
    dut.uio_in.value = 0
    drive(dut, SPEED0, csn=1)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


def ws_word(pix, rgb_order=False):
    """Expected 24-bit WS2812 word for a packed 3-2-3 RGB pixel byte."""
    r = ((pix >> 5) & 7) << 5
    g = ((pix >> 3) & 3) << 6
    b = (pix & 7) << 5
    return (r << 16 | g << 8 | b) if rgb_order else (g << 16 | r << 8 | b)


async def capture_ws2812(dut, t0h, cfg, n_leds=64):
    """Sample uo_out[0] every clock and decode one WS2812 frame.

    Must be called right after WS2812 mode is enabled, so the stream starts
    with its reset gap. Returns the list of 24-bit words, one per LED.
    """
    period = 3 * t0h
    gap = 256 * period
    inv = bool(cfg & COL_INV)
    rows_idle = 0x00 if cfg & ROW_INV else 0xFF
    total = gap + n_leds * 24 * period + gap

    runs = []  # run-length encoded line level: [level, length]
    for _ in range(total):
        await FallingEdge(dut.clk)
        uo = int(dut.uo_out.value)
        assert uo & 0xFE == 0, f"uo_out[7:1] not idle in WS2812 mode: {uo:#04x}"
        assert int(dut.uio_out.value) == rows_idle, "rows not deselected in WS2812 mode"
        lv = (uo & 1) ^ inv
        if runs and runs[-1][0] == lv:
            runs[-1][1] += 1
        else:
            runs.append([lv, 1])

    # skip to the end of the leading reset gap
    i = 0
    while i < len(runs) and not (runs[i][0] == 0 and runs[i][1] >= gap // 2):
        i += 1
    assert i < len(runs), "no leading reset gap found"
    i += 1

    bits = []
    while i + 1 < len(runs):
        high, low = runs[i], runs[i + 1]
        assert high[0] == 1
        if high[1] == 2 * t0h:
            bits.append(1)
        elif high[1] == t0h:
            bits.append(0)
        else:
            raise AssertionError(f"bad high time {high[1]} clocks at bit {len(bits)}")
        if low[1] >= gap // 2:
            break  # trailing reset gap: frame complete
        assert high[1] + low[1] == period, \
            f"bad bit period {high[1] + low[1]} clocks at bit {len(bits)}"
        i += 2

    assert len(bits) == 24 * n_leds, f"decoded {len(bits)} bits, expected {24 * n_leds}"
    return [int("".join(map(str, bits[k * 24:(k + 1) * 24])), 2) for k in range(n_leds)]


@cocotb.test()
async def test_reset_state(dut):
    """After reset the framebuffer is dark, rows scan, uio pins are outputs."""
    cocotb.start_soon(Clock(dut.clk, 100, unit="ns").start())
    await reset(dut)

    assert int(dut.uio_oe.value) == 0xFF

    counts = await measure_frame(dut)
    assert counts == [[0] * 8 for _ in range(8)], "framebuffer not dark after reset"

    dwell = await row_dwell(dut)
    assert dwell == 256, f"row dwell {dwell} != 256 at SPEED0"


@cocotb.test()
async def test_full_frame_pwm(dut):
    """Write all 64 pixels and check every PWM duty cycle."""
    cocotb.start_soon(Clock(dut.clk, 100, unit="ns").start())
    await reset(dut)

    # a mix of off, full, and graded brightness values
    image = [(r * 37 + c * 11) & 0xFF for r in range(8) for c in range(8)]
    image[0] = 0
    image[7] = 255
    image[63] = 128

    await spi_write(dut, SPEED0, 0, image)
    await ClockCycles(dut.clk, 4)

    counts = await measure_frame(dut)
    assert counts == expected_from(image), f"PWM counts mismatch:\n{counts}"


@cocotb.test()
async def test_partial_write_and_wrap(dut):
    """A transfer starting mid-buffer only touches those pixels; address wraps 63 -> 0."""
    cocotb.start_soon(Clock(dut.clk, 100, unit="ns").start())
    await reset(dut)

    image = [0] * 64
    await spi_write(dut, SPEED0, 21, [10, 20, 30])
    image[21:24] = [10, 20, 30]

    # start at 62, write 4 bytes: addresses 62, 63, 0, 1
    await spi_write(dut, SPEED0, 62, [1, 2, 3, 4])
    image[62], image[63], image[0], image[1] = 1, 2, 3, 4

    await ClockCycles(dut.clk, 4)
    counts = await measure_frame(dut)
    assert counts == expected_from(image), f"partial write mismatch:\n{counts}"


@cocotb.test()
async def test_blank_and_invert(dut):
    """BLANK forces columns off; COL_INV / ROW_INV flip output polarity."""
    cocotb.start_soon(Clock(dut.clk, 100, unit="ns").start())
    await reset(dut)

    image = [200] * 64
    await spi_write(dut, SPEED0, 0, image)

    drive(dut, SPEED0 | BLANK)
    await ClockCycles(dut.clk, 4)
    counts = await measure_frame(dut)
    assert counts == [[0] * 8 for _ in range(8)], "BLANK did not turn columns off"

    drive(dut, SPEED0 | COL_INV | ROW_INV)
    await ClockCycles(dut.clk, 4)
    counts = await measure_frame(dut, col_inv=True, row_inv=True)
    assert counts == expected_from(image), "inverted outputs mismatch"


@cocotb.test()
async def test_speed_select(dut):
    """SPEED bits scale the PWM tick by 1 / 8 / 64."""
    cocotb.start_soon(Clock(dut.clk, 100, unit="ns").start())
    await reset(dut)

    for cfg, expect in ((SPEED0, 256), (SPEED1, 256 * 8), (SPEED2, 256 * 64)):
        drive(dut, cfg)
        await ClockCycles(dut.clk, 4)
        dwell = await row_dwell(dut)
        assert dwell == expect, f"row dwell {dwell} != {expect} for cfg {cfg:#04x}"


@cocotb.test()
async def test_ws2812_frame(dut):
    """WS2812 mode streams 64 LEDs in GRB order with the default 400 ns timing."""
    cocotb.start_soon(Clock(dut.clk, 100, unit="ns").start())
    await reset(dut)

    image = [(r * 41 + c * 13) & 0xFF for r in range(8) for c in range(8)]
    image[0] = 0xFF
    image[1] = 0x00
    image[2] = 0b111_00_000  # red only
    image[3] = 0b000_11_000  # green only
    image[4] = 0b000_00_111  # blue only
    await spi_write(dut, SPEED0, 0, image)

    await spi_write(dut, SPEED0, REG_BASE | REG_CTRL, [CTRL_WS])
    words = await capture_ws2812(dut, t0h=4, cfg=SPEED0)
    assert words == [ws_word(p) for p in image], f"GRB frame mismatch: {[hex(w) for w in words]}"


@cocotb.test()
async def test_ws2812_rgb_timing_invert_blank(dut):
    """RGB order bit, programmable T0H, inverted data line, and BLANK in WS2812 mode."""
    cocotb.start_soon(Clock(dut.clk, 100, unit="ns").start())
    await reset(dut)

    image = [(r * 29 + c * 7 + 3) & 0xFF for r in range(8) for c in range(8)]
    await spi_write(dut, SPEED0, 0, image)

    # T0H = 6 clocks -> 18-clock bit period; then enable WS2812 mode in RGB order
    await spi_write(dut, SPEED0, REG_BASE | REG_T0H, [6])
    cfg = COL_INV | ROW_INV
    await spi_write(dut, cfg, REG_BASE | REG_CTRL, [CTRL_WS | CTRL_RGB])
    words = await capture_ws2812(dut, t0h=6, cfg=cfg)
    assert words == [ws_word(p, rgb_order=True) for p in image], "RGB frame mismatch"

    # BLANK sends all-zero pixels; restart the stream by toggling the mode bit
    await spi_write(dut, cfg, REG_BASE | REG_CTRL, [0])
    cfg |= BLANK
    await spi_write(dut, cfg, REG_BASE | REG_CTRL, [CTRL_WS | CTRL_RGB])
    words = await capture_ws2812(dut, t0h=6, cfg=cfg)
    assert words == [0] * 64, "BLANK did not zero the WS2812 stream"

    # back to matrix mode: the framebuffer is intact and rows scan again
    drive(dut, SPEED0)
    await spi_write(dut, SPEED0, REG_BASE | REG_CTRL, [0])
    await ClockCycles(dut.clk, 4)
    counts = await measure_frame(dut)
    assert counts == expected_from(image), "framebuffer changed after WS2812 mode"
