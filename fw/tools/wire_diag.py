#!/usr/bin/env python3
"""
wire_diag.py — minimal on-wire probe for Nano Sentinel.

Whole-file summary:
  Opens the serial port with DTR=1, RTS=0 (to avoid auto-reset pulse),
  shows any boot banner, then sends a precomputed PING frame (FT_DATA with
  payload [OP_PING]) and prints raw bytes seen + parsed result. This bypasses
  the rest of the CLI stack so we can tell exactly what's happening on the wire.
"""

import sys, time, serial

# ----------------------------- CRC + framing ---------------------------------

def crc16_ccitt_false(data: bytes) -> int:
    """
    CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, no reflect, no xorout).

    Whole-function summary:
      Matches firmware's CRC so host/device agree.

    Line-by-line:
      - XOR byte into CRC high byte.
      - Shift left 8 times; when high bit set, XOR poly 0x1021.
      - Keep to 16 bits.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if (crc & 0x8000):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def build_frame(ftype: int, payload: bytes) -> bytes:
    """
    Build [AA 55][VER=0x01][TYPE][LEN_HI LEN_LO][PAYLOAD][CRC_HI CRC_LO].

    Whole-function summary:
      Exactly the on-wire format the firmware expects.
    """
    ver = 0x01
    ln  = len(payload)
    header = bytes([ver, ftype, (ln >> 8) & 0xFF, ln & 0xFF])
    crc = crc16_ccitt_false(header + payload)
    return b"\xAA\x55" + header + payload + bytes([crc >> 8, crc & 0xFF])

def parse_first_frame(buf: bytes):
    """
    Return (type:int, payload:bytes) if a valid frame exists, else (None, None).

    Whole-function summary:
      Finds 0xAA55, checks length, verifies CRC; returns first complete frame.
    """
    i = buf.find(b"\xAA\x55")
    if i < 0 or len(buf) - i < 6:
        return (None, None)
    ftype = buf[i+3]
    ln = (buf[i+4] << 8) | buf[i+5]
    total = 2 + 4 + ln + 2
    if len(buf) - i < total:
        return (None, None)
    body = buf[i+2 : i+2+4+ln]
    rx_crc = (buf[i+2+4+ln] << 8) | buf[i+2+4+ln+1]
    if crc16_ccitt_false(body) != rx_crc:
        return (None, None)
    payload = buf[i+6 : i+6+ln]
    return (ftype, payload)

# ----------------------------- Probe routine ---------------------------------

def main():
    """
    Open port, show boot text, send PING frame, print raw + parsed reply.

    Whole-function summary:
      Uses pySerial with timeouts; asserts DTR before opening; swallows any
      boot banner; sends PING; reads for 4s; dumps RX (raw hex) and parsed type.
    """
    if len(sys.argv) < 2:
        print("Usage: python fw/tools/wire_diag.py /dev/cu.usbserial-XXXX")
        sys.exit(2)
    port = sys.argv[1]
    baud = 115200

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.6
    ser.write_timeout = 0.6
    ser.xonxoff = False
    ser.rtscts  = False
    ser.dsrdtr  = False
    ser.dtr = True   # assert DTR up-front (no falling-edge reset)
    ser.rts = False
    ser.open()

    # 1) Swallow short boot banner (if any)
    t0 = time.time()
    banner = b""
    while time.time() - t0 < 2.0:
        chunk = ser.read(256)
        if not chunk: break
        banner += chunk
    if banner:
        try:
            print("[boot]", banner.decode(errors="ignore").strip())
        except Exception:
            print("[boot hex]", banner.hex(" "))

    ser.reset_input_buffer()

    # 2) Build + send a PING frame: TYPE=FT_DATA(0x01), payload [OP_PING=0x01]
    payload = bytes([0x01])                # OP_PING
    frame   = build_frame(0x01, payload)   # FT_DATA = 0x01
    print("TX:", frame.hex(" "))
    ser.write(frame); ser.flush()

    # 3) Read for up to 4 seconds and parse first valid frame
    deadline = time.time() + 4.0
    buf = b""
    parsed_once = False
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            t, p = parse_first_frame(buf)
            if t is not None:
                print(f"(parsed) type=0x{t:02X} len={len(p)} payload={p!r}")
                parsed_once = True
                break

    if not parsed_once:
        if buf:
            print("RX (raw):", buf.hex(" "))
        else:
            print("(timeout) no bytes received after TX")

if __name__ == "__main__":
    main()