#!/usr/bin/env python3
"""
sniff.py — passive frame sniffer (no TX).

Whole-function summary:
  Opens the serial port with DTR/RTS LOW (to avoid resets), then reads bytes
  and prints any valid framed packets it can parse. Helpful to verify the
  device is sending heartbeats and the host can parse them.
"""

import sys, serial, time

def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, no reflect, no xorout)."""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if (crc & 0x8000): crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:               crc = (crc << 1) & 0xFFFF
    return crc

def parse_first_frame(buf: bytes):
    """Return (start, total, type, payload) or (None,None,None,None) if incomplete/invalid."""
    i = buf.find(b"\xAA\x55")
    if i < 0 or len(buf) - i < 6: return (None, None, None, None)
    ftyp = buf[i+3]
    ln   = (buf[i+4] << 8) | buf[i+5]
    total = 2 + 4 + ln + 2
    if len(buf) - i < total: return (None, None, None, None)
    body = buf[i+2 : i+2+4+ln]
    rx_crc = (buf[i+2+4+ln] << 8) | buf[i+2+4+ln+1]
    if crc16_ccitt_false(body) != rx_crc:
        return (i, 2, None, None)  # signal: drop 2 bytes and re-sync
    payload = buf[i+6 : i+6+ln]
    return (i, total, ftyp, payload)

def main():
    if len(sys.argv) < 2:
        print("Usage: python fw/tools/sniff.py /dev/cu.usbserial-1130"); sys.exit(2)
    port = sys.argv[1]

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.5
    ser.write_timeout = 0.5
    ser.xonxoff = False
    ser.rtscts  = False
    ser.dsrdtr  = False
    ser.dtr = False     # keep LOW to avoid reset
    ser.rts = False
    ser.open()

    print("[sniff] listening… (Ctrl-C to stop)")
    buf = b""
    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                buf += chunk
                while True:
                    i, tot, typ, pay = parse_first_frame(buf)
                    if i is None:
                        break
                    if typ is None:           # bad CRC: drop sync and continue
                        buf = buf[i+2:]; continue
                    # consume this valid frame
                    frame = buf[i:i+tot]
                    buf = buf[i+tot:]
                    print(f"FT=0x{typ:02X} len={len(pay)} pay={pay.hex(' ')}")
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()