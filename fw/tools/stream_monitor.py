#!/usr/bin/env python3
"""
stream_monitor.py — live plot of Nano Sentinel frames.

Whole-file summary:
  Opens the serial port safely (no auto-reset pulse), then continuously reads
  framed replies from the device:
    • FT_STATUS (0x04, payload 3 bytes): shows sampling ON/OFF + rate_ms banner.
    • FT_DATA   (0x01, payload 3 bytes): interprets as 16-bit sample + seq
  A background reader thread parses frames; the main thread renders a live plot.

Usage:
  python fw/tools/stream_monitor.py /dev/cu.usbserial-1130
"""

import sys, time, threading, collections, argparse
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Frame types (match firmware's frame.h)
FT_DATA   = 0x01
FT_STATUS = 0x04

# ----------------------------- Framing Utils ---------------------------------

def crc16_ccitt_false(data: bytes) -> int:
    """
    CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, no reflect, no xorout).

    Whole-function summary:
      Matches the firmware CRC implementation so host and device agree.

    Line-by-line:
      - XOR each byte into the CRC high byte.
      - Shift left 8 times; XOR with 0x1021 when high bit set.
      - Keep CRC masked to 16 bits.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if (crc & 0x8000) != 0:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_first_frame(buf: bytes):
    """
    Parse the first complete frame found in 'buf'; return (start_idx, total_len, ftype, payload) or (None, None, None, None).

    Whole-function summary:
      Finds sync (AA 55), verifies header + payload + CRC are present and valid,
      and returns the frame along with indices so the caller can drop consumed bytes.

    Line-by-line:
      - Find 0xAA55, ensure >=6 bytes to read header (VER,TYPE,LEN).
      - Ensure full frame present; compute CRC over VER..PAYLOAD; compare.
      - Return tuple; otherwise indicate “no full/valid frame yet”.
    """
    i = buf.find(b"\xAA\x55")
    if i < 0 or len(buf) - i < 6:
        return (None, None, None, None)
    ver  = buf[i+2]
    ftyp = buf[i+3]
    ln   = (buf[i+4] << 8) | buf[i+5]
    total = 2 + 4 + ln + 2
    if len(buf) - i < total:
        return (None, None, None, None)
    body = buf[i+2 : i+2+4+ln]                         # VER,TYPE,LEN, PAYLOAD
    rx_crc = (buf[i+2+4+ln] << 8) | buf[i+2+4+ln+1]
    if crc16_ccitt_false(body) != rx_crc:
        # Bad CRC: drop the sync byte and keep searching next loop
        return (i, 2, None, None)
    payload = buf[i+6 : i+6+ln]
    return (i, total, ftyp, payload)

# ----------------------------- Serial Helpers --------------------------------

def open_serial_safely(port: str, baud: int) -> serial.Serial:
    """
    Open serial in a way that avoids auto-reset and allows TX (DTR=1, RTS=0).

    Whole-function summary:
      Build a Serial() with flow control disabled; set DTR asserted and RTS
      deasserted *before* opening so adapters pass TX without a falling-edge
      reset. Swallow any boot banner and return the handle.
    """
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.25
    ser.write_timeout = 0.5
    ser.xonxoff = False
    ser.rtscts  = False
    ser.dsrdtr  = False
    ser.dtr = True
    ser.rts = False
    ser.open()
    # Swallow brief boot banner
    t0 = time.time()
    junk = b""
    while time.time() - t0 < 2.0:
        chunk = ser.read(256)
        if not chunk:
            break
        junk += chunk
        if b"[BOOT] READY" in junk or b"SELF-TEST PASS" in junk:
            break
    ser.reset_input_buffer()
    return ser

# ----------------------------- Live Reader + Plot -----------------------------

def start_reader(ser: serial.Serial, out_deque: collections.deque, status_dict: dict):
    """
    Spawn a thread that reads bytes, parses frames, and populates sample deque.

    Whole-function summary:
      Reads chunks from serial, appends to a buffer, extracts valid frames,
      and when it sees:
        * FT_DATA (0x01) with 3-byte payload [hi,lo,seq], pushes sample to deque
        * FT_STATUS (0x04) with 3-byte payload, updates status_dict
      Drops bad-CRC frames by skipping their sync and keeps streaming.

    Line-by-line:
      - Loop: read -> append -> parse; on valid frames, act and remove them.
      - Use out_deque (maxlen-limited) so the plot stays responsive.
    """
    buf = b""
    while True:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            while True:
                i, total, ftyp, payload = parse_first_frame(buf)
                if i is None:
                    break
                if ftyp is None:
                    # bad CRC: drop first 2 bytes and continue
                    buf = buf[i+2:]
                    continue
                # Consume this valid frame
                frame = buf[i:i+total]
                buf = buf[i+total:]

                if ftyp == FT_DATA and len(payload) == 3:
                    sample = (payload[0] << 8) | payload[1]
                    seq    = payload[2]
                    ts     = time.time()
                    out_deque.append((ts, sample, seq))
                elif ftyp == FT_STATUS and len(payload) == 3:
                    status_dict["on"]   = (payload[0] == 1)
                    status_dict["rate"] = (payload[1] << 8) | payload[2]


def main():
    """
    Parse CLI args, start reader thread, and render a live plot.

    Whole-function summary:
      Opens serial, spawns a reader thread filling a deque of (ts,sample,seq),
      and uses Matplotlib animation to draw the last N seconds of samples.
      The plot title shows current sampling ON/OFF and rate_ms from STATUS.
    """
    ap = argparse.ArgumentParser(description="Live stream monitor for Nano Sentinel")
    ap.add_argument("port", help="e.g., /dev/cu.usbserial-1130")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=20.0, help="seconds of data to display")
    args = ap.parse_args()

    ser = open_serial_safely(args.port, args.baud)

    samples = collections.deque(maxlen=2048)
    status  = {"on": False, "rate": 0}

    t = threading.Thread(target=start_reader, args=(ser, samples, status), daemon=True)
    t.start()

    fig, ax = plt.subplots()
    ax.set_xlabel("time (s)")
    ax.set_ylabel("sample")
    line, = ax.plot([], [], lw=1)

    def animate(_frame_idx):
        """Update plot with latest samples."""
        if not samples:
            return line,
        t_now = time.time()
        xs = [s[0] - t_now for s in samples if (t_now - s[0]) <= args.window]
        ys = [s[1] for s in samples if (t_now - s[0]) <= args.window]
        line.set_data(xs, ys)
        ax.set_xlim(-args.window, 0)
        if ys:
            ypad = max(1, int((max(ys) - min(ys)) * 0.1))
            ymin = min(ys) - ypad
            ymax = max(ys) + ypad
            if ymin == ymax:
                ymin -= 1; ymax += 1
            ax.set_ylim(ymin, ymax)
        ax.set_title(f"Nano Sentinel — sampling={'ON' if status['on'] else 'OFF'}  rate_ms={status['rate']}")
        return line,

    FuncAnimation(fig, animate, interval=200, blit=True)
    plt.show()

if __name__ == "__main__":
    main()
