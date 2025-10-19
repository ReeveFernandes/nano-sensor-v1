#!/usr/bin/env python3
import sys, struct, argparse, time
import serial

# ---------------- CRC + framing helpers ----------------

def crc16_ccitt_false(data: bytes) -> int:
    """
    Compute CRC-16/CCITT-FALSE for a bytestring.

    Whole-function summary:
      Implements CCITT-FALSE (poly 0x1021, init 0xFFFF, no final XOR).
      Used to protect frame header + payload.

    Line-by-line:
      - Start crc at 0xFFFF.
      - XOR each byte into the high byte, then process 8 bit steps.
      - On each bit: left shift; if MSB set, XOR with 0x1021.
      - Return 16-bit CRC as int.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def build_frame(ftype: int, payload: bytes) -> bytes:
    """
    Build one wire frame: [AA 55][VER=1][TYPE][LEN_HI LEN_LO][PAYLOAD][CRC_HI CRC_LO].

    Whole-function summary:
      Packs sync, version, type, length; computes CRC over VER..PAYLOAD and appends it.

    Line-by-line:
      - Assemble header with sync (AA 55), version (1), type, and big-endian length.
      - Compute CRC over header[2:] + payload (VER through end of payload).
      - Append big-endian CRC to complete the frame.
    """
    ver = 0x01
    hdr = bytes([0xAA, 0x55, ver, ftype]) + struct.pack(">H", len(payload))
    crc = crc16_ccitt_false(hdr[2:] + payload)
    return hdr + payload + struct.pack(">H", crc)

def parse_first_frame(blob: bytes):
    """
    Parse the first complete frame in 'blob'; return (type, payload) or None.

    Whole-function summary:
      Finds sync, checks header, validates CRC when enough bytes are present,
      and returns the type and payload bytes for the first complete frame.

    Line-by-line:
      - Search for sync AA 55.
      - Ensure we have ≥6 bytes to read version, type, length.
      - Compute total frame size = 6 + payload + 2.
      - If enough bytes exist, validate CRC and return (type, payload).
      - Otherwise return None to indicate incomplete data.
    """
    i = blob.find(b"\xAA\x55")
    if i < 0 or len(blob) < i + 6:
        return None
    ver, ftype = blob[i+2], blob[i+3]
    length = struct.unpack(">H", blob[i+4:i+6])[0]
    total = 6 + length + 2
    if len(blob) < i + total:
        return None
    frame = blob[i:i+total]
    crc_rx = struct.unpack(">H", frame[-2:])[0]
    crc_calc = crc16_ccitt_false(frame[2:-2])
    if crc_rx != crc_calc:
        return None
    payload = frame[6:-2]
    return ftype, payload

# ---------------- Command helpers (opcodes) ----------------
OP_PING       = 0x01
OP_START      = 0x02
OP_STOP       = 0x03
OP_SET_RATE   = 0x04
OP_GET_STATUS = 0x05

FT_DATA   = 0x01
FT_ACK    = 0x02
FT_NACK   = 0x03
FT_STATUS = 0x04

def wait_for_ready(ser, timeout=3.0):
    """
    Wait for the firmware boot banner to finish before sending commands.

    Whole-function summary:
      Reads bytes for up to 'timeout' seconds and watches for markers the
      firmware prints at boot ("SELF-TEST PASS" and "[BOOT] READY"). As soon
      as one is seen, the function returns True. If neither appears before
      the deadline, it returns False (we'll still try to send afterward).

    Line-by-line:
      - Set an absolute deadline based on time.time().
      - Repeatedly read small chunks from the serial port and append to a buffer.
      - Convert the buffer to text (errors ignored) and search for markers.
      - If a marker is found, return True immediately.
      - If we hit the deadline, return False (caller may still proceed).
    """
    import time
    deadline = time.time() + timeout                 # When to stop waiting
    buf = b""                                        # Accumulate banner bytes
    while time.time() < deadline:
        chunk = ser.read(256)                        # Non-blocking-ish read
        if chunk:
            buf += chunk
            text = buf.decode(errors="ignore")       # Convert to text safely
            if "SELF-TEST PASS" in text or "[BOOT] READY" in text:
                return True                          # Boot finished
    return False                                     # Timed out waiting

def send_cmd(ser, payload: bytes, expect_timeout=2.5):
    """
    Send one command as FT_DATA and read back one reply frame.

    Whole-function summary:
      Frames the payload (TYPE=0x01), writes it, and reads until a valid reply
      parses or timeout. Prints TX bytes and, on failure, dumps raw RX bytes.

    Line-by-line:
      - Build header + CRC16 (CCITT-FALSE) exactly like firmware.
      - Clear any stale RX, write frame, flush immediately.
      - Read chunks and attempt to parse a complete reply frame.
      - If nothing parses before deadline, print whatever we did read.
    """
    frame = build_frame(0x01, payload)       # 0x01 = FT_DATA (matches firmware)
    print("TX:", frame.hex(" "))
    ser.reset_input_buffer()                  # drop old bytes
    ser.write(frame); ser.flush()             # send now

    import time
    deadline = time.time() + expect_timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            parsed = parse_first_frame(buf)
            if parsed:
                return parsed
    if buf:
        print("RX (raw):", buf.hex(" "))      # helpful when only banner/debug prints arrive
    return None, None

def main():
    """
    Open serial and send one of: --ping/--start/--stop/--set-rate MS/--get-status.

    Whole-function summary:
      Opens the serial port with DTR asserted from the start (to satisfy USB-
      serial chips that gate TX on DTR), without pulsing a reset. We:
        - Build Serial() (do-not-open).
        - Pre-set DTR=True and RTS=False BEFORE open (no falling-edge pulse).
        - Open, wait for boot banner ("READY"/"PASS"), flush input.
        - Send framed command and print reply (retry once).

    Steps:
      - Disable all flow control.
      - Set dtr=True, rts=False BEFORE ser.open() to avoid a reset edge.
      - Wait for firmware banner via wait_for_ready(), then flush.
      - Send command with send_cmd(); retry once if needed.
    """
    import argparse, sys, time, serial
    ap = argparse.ArgumentParser(description="Nano Sentinel host CLI")
    ap.add_argument("port", help="e.g., /dev/cu.usbserial-1120")
    ap.add_argument("--baud", type=int, default=115200)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--ping", action="store_true")
    g.add_argument("--start", action="store_true")
    g.add_argument("--stop", action="store_true")
    g.add_argument("--set-rate", type=int, metavar="MS")
    g.add_argument("--get-status", action="store_true")
    args = ap.parse_args()

    # Build serial WITHOUT opening yet (so we can set control lines safely)
    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.timeout = 0.5
    ser.write_timeout = 0.5
    ser.xonxoff = False
    ser.rtscts  = False
    ser.dsrdtr  = False

    # Key: assert DTR from the start so TX isn't gated; keep RTS false.
    ser.dtr = True
    ser.rts = False
    try:
      ser.exclusive = True
    except Exception:
      pass
    ser.open()                              # open with DTR already True

    # Eat the boot banner so we don't race the bootloader
    _ = wait_for_ready(ser, timeout=3.0)
    ser.reset_input_buffer()

    # Build payload
    if args.ping:
        payload = bytes([OP_PING])
    elif args.start:
        payload = bytes([OP_START])
    elif args.stop:
        payload = bytes([OP_STOP])
    elif args.set_rate is not None:
        ms = args.set_rate
        if not (10 <= ms <= 10000):
            print("SET_RATE out of range (10..10000)"); sys.exit(2)
        payload = bytes([OP_SET_RATE, (ms >> 8) & 0xFF, ms & 0xFF])
    else:
        payload = bytes([OP_GET_STATUS])

    # Send (retry once if nothing comes back)
    rtype, rpay = send_cmd(ser, payload, expect_timeout=2.5)
    if rtype is None:
        time.sleep(0.6)
        rtype, rpay = send_cmd(ser, payload, expect_timeout=2.5)

    if rtype is None:
        print("No reply (timeout). Make sure firmware with proto.c is flashed and baud=115200.")
        sys.exit(3)

    if rtype == FT_ACK:
        print(f"ACK: {rpay!r}")
    elif rtype == FT_STATUS and len(rpay) == 3:
        on = "ON" if rpay[0] == 1 else "OFF"
        rate = (rpay[1] << 8) | rpay[2]
        print(f"STATUS: sampling={on}, rate_ms={rate}")
    elif rtype == FT_NACK and len(rpay) >= 1:
        print(f"NACK: reason={rpay[0]}")
    else:
        print(f"Reply type=0x{rtype:02X}, payload={rpay!r}")

if __name__ == "__main__":
    main()