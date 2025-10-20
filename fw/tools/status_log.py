#!/usr/bin/env python3
"""
status_log.py — poll GET_STATUS and log to CSV (with optional verbose debug).

Whole-file summary:
  Opens serial safely (avoid auto-reset, allow TX), waits for the firmware
  boot banner to finish, then polls GET_STATUS (opcode 0x05) at a fixed
  interval and appends rows to CSV: timestamp_iso, sampling_on, rate_ms.
  With --verbose, it prints TX frames and any raw RX bytes it gets so you
  can see exactly why a row did or did not log.
"""

import sys, time, argparse
import serial

# ----------------------------- Framing Utilities -----------------------------

def crc16_ccitt_false(data: bytes) -> int:
    """
    CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, no reflect, no xorout).

    Whole-function summary:
      Matches the firmware's CRC (crc16_ccitt_false). Feed the same bytes in
      the same order to get identical results.

    Line-by-line:
      - XOR each byte into the high byte of the running CRC.
      - Shift left 8 times; XOR with 0x1021 whenever the high bit is set.
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


def build_frame(ftype: int, payload: bytes) -> bytes:
    """
    Build a frame: [AA 55][VER=0x01][TYPE][LEN_HI LEN_LO][PAYLOAD][CRC_HI CRC_LO].

    Whole-function summary:
      Constructs the wire format the firmware expects, with CRC16 over
      VER,TYPE,LEN and PAYLOAD (sync bytes excluded).
    """
    ver = 0x01
    ln  = len(payload)
    header = bytes([ver, ftype, (ln >> 8) & 0xFF, ln & 0xFF])
    crc = crc16_ccitt_false(header + payload)
    return bytes([0xAA, 0x55]) + header + payload + bytes([(crc >> 8) & 0xFF, crc & 0xFF])


def parse_first_frame(buf: bytes):
    """
    Parse the first complete frame found in 'buf'; return (type:int, payload:bytes) or (None, None).

    Whole-function summary:
      Looks for sync (AA 55), verifies enough bytes for header+payload+CRC,
      checks CRC, and returns (TYPE, PAYLOAD) if valid. Otherwise (None, None).
    """
    i = buf.find(b"\xAA\x55")
    if i < 0 or len(buf) - i < 6:
        return (None, None)
    ftype = buf[i+3]
    ln = (buf[i+4] << 8) | buf[i+5]
    total = 2 + 4 + ln + 2
    if len(buf) - i < total:
        return (None, None)
    body = buf[i+2 : i+2+4+ln]                        # VER,TYPE,LEN, PAYLOAD
    rx_crc = (buf[i+2+4+ln] << 8) | buf[i+2+4+ln+1]
    if crc16_ccitt_false(body) != rx_crc:
        return (None, None)
    payload = buf[i+6 : i+6+ln]
    return (ftype, payload)

# ----------------------------- Serial Helpers --------------------------------

def wait_for_ready(ser, timeout=3.0):
    """
    Swallow the firmware boot banner until we see 'READY' or 'SELF-TEST PASS'.

    Whole-function summary:
      Reads for up to 'timeout' seconds looking for known end-of-boot markers.
      Returns True if a marker is seen, else False. Either way, leaves the
      port ready for command traffic.
    """
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            text = buf.decode(errors="ignore")
            if "SELF-TEST PASS" in text or "[BOOT] READY" in text:
                return True
    return False


def open_serial_safely(port: str, baud: int, verbose: bool = False) -> serial.Serial:
    """
    Open serial avoiding auto-reset, with TX allowed.

    Whole-function summary:
      Creates a Serial with flow control off, sets DTR=True and RTS=False
      *before* open (no falling-edge reset), waits for boot banner to finish,
      flushes remaining bytes, and returns the handle.
    """
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.5
    ser.write_timeout = 0.5
    ser.xonxoff = False
    ser.rtscts = False
    ser.dsrdtr = False
    ser.dtr = True         # assert DTR from the start so TX isn't gated
    ser.rts = False
    ser.open()
    if verbose:
        print(f"[open] {port} @ {baud} (DTR=1 RTS=0)")

    # Eat boot banner if device resets on open
    _ = wait_for_ready(ser, timeout=3.0)
    ser.reset_input_buffer()
    return ser


def send_cmd(ser: serial.Serial, payload: bytes, expect_timeout: float = 2.5, verbose: bool = False):
    """
    Send one command as FT_DATA and read back one reply frame.

    Whole-function summary:
      Frames the payload (TYPE=0x01), writes it, then reads until a
      CRC-valid reply is parsed or timeout. If verbose, prints TX bytes
      and any raw RX seen on timeout to aid debugging.

    Line-by-line:
      - Build the frame with build_frame(0x01, payload) to match firmware.
      - Clear stale RX, write frame, and flush immediately.
      - Read chunks into a buffer and attempt to parse the first valid frame.
      - On success, return (type, payload, raw_bytes_buffer).
      - On timeout, return (None, None, raw_bytes_buffer) and optionally print it.
    """
    frame = build_frame(0x01, payload)  # 0x01 = FT_DATA per firmware
    if verbose:
        print("TX:", frame.hex(" "))
    ser.reset_input_buffer()
    ser.write(frame); ser.flush()

    import time
    deadline = time.time() + expect_timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            t, p = parse_first_frame(buf)
            if t is not None:
                if verbose:
                    print(f"(parsed) type=0x{t:02X} len={len(p)}")
                return (t, p, buf)
    if verbose and buf:
        print("RX (raw):", buf.hex(" "))
    return (None, None, buf)

# ----------------------------- Monitor Logic ---------------------------------

def monitor_status(port: str, period_s: float, duration_s: float | None, out_csv: str, verbose: bool):
    """
    Poll GET_STATUS at a fixed period and write CSV (timestamp_iso,sampling_on,rate_ms).

    Whole-function summary:
      Opens serial safely, then for 'duration_s' seconds (or forever if None)
      sends GET_STATUS each 'period_s'. If a reply parses and its payload is
      exactly 3 bytes, it decodes [on, rate_hi, rate_lo] and logs a row —
      regardless of numeric frame TYPE — because some builds may vary TYPE IDs.

    Line-by-line:
      - Open serial with DTR asserted (TX allowed) and swallow boot banner.
      - In a loop: send opcode 0x05 (GET_STATUS), parse reply.
      - If payload length == 3, decode it and append CSV; else print a hint.
      - Sleep the remaining time to maintain the period.
    """
    ser = open_serial_safely(port, 115200, verbose=verbose)
    t_end = None if duration_s is None else (time.time() + duration_s)

    with open(out_csv, "w", encoding="utf-8") as f:
        f.write("timestamp_iso,sampling_on,rate_ms\n")
        while t_end is None or time.time() < t_end:
            # OP_GET_STATUS = 0x05
            t, p, raw = send_cmd(ser, bytes([0x05]), expect_timeout=2.5, verbose=verbose)
            if p is not None and len(p) == 3:
                on = 1 if p[0] == 1 else 0
                rate = (p[1] << 8) | p[2]
                iso = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())
                f.write(f"{iso},{on},{rate}\n"); f.flush()
                print(f"{iso}  STATUS  sampling={'ON' if on else 'OFF'}  rate_ms={rate}")
            else:
                # Helpful hints so you see what's going on each tick
                if t is None and not raw:
                    print("(timeout)", end="", flush=True)
                elif t is None and raw:
                    print("(no-parse)", end="", flush=True)
                else:
                    # Parsed but payload length unexpected: show type/len
                    print(f"(type=0x{t:02X}, len={len(p) if p else 0})", end="", flush=True)
            time.sleep(period_s)

# ----------------------------- CLI Entry -------------------------------------

def main():
    """
    Parse CLI args and start the monitor.
    """
    ap = argparse.ArgumentParser(description="Log Nano Sentinel GET_STATUS to CSV")
    ap.add_argument("port", help="e.g., /dev/cu.usbserial-1130")
    ap.add_argument("--period", type=float, default=1.0, help="seconds between polls")
    ap.add_argument("--duration", type=float, default=10.0, help="seconds to run (default 10)")
    ap.add_argument("--out", type=str, default="status.csv", help="CSV output path")
    ap.add_argument("--verbose", action="store_true", help="print TX and raw RX for debugging")
    args = ap.parse_args()
    monitor_status(args.port, args.period, args.duration, args.out, args.verbose)

if __name__ == "__main__":
    main()
