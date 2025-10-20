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

def drain_boot_banner(ser, timeout=3.0):
    """
    Drain any boot banner if the board just reset.

    Whole-function summary:
      Reads for up to 'timeout' seconds looking for typical boot text
      (e.g., 'SELF-TEST PASS' or '[BOOT] READY'). Returns True if a banner
      was seen (i.e., we *did* catch a reset), False otherwise.

    Line-by-line:
      - Read chunks into a buffer; decode as best-effort text.
      - If known markers appear, return True and leave the port clean.
    """
    import time
    deadline = time.time() + timeout
    buf = b""
    saw = False
    while time.time() < deadline:
        chunk = ser.read(256)
        if not chunk:
            # small pause to give USB a breath
            time.sleep(0.02)
            continue
        buf += chunk
        text = buf.decode(errors="ignore")
        if "SELF-TEST PASS" in text or "[BOOT] READY" in text or "[BOOT]" in text:
            saw = True
            # keep draining a moment more to get to READY
    ser.reset_input_buffer()
    return saw

def wait_for_ready(ser, timeout=5.0):
    """
    Wait until firmware finishes its boot banner.

    Whole-function summary:
      Reads for up to 'timeout' seconds looking for "SELF-TEST PASS" or
      "[BOOT] READY". Returns True if seen, else False. Either way, leaves
      the port ready for command traffic.

    Line-by-line:
      - Read small chunks and accumulate in a buffer.
      - Decode to text ignoring unicode errors; search for markers.
      - If nothing seen by the deadline, just return False and continue.
    """
    import time
    deadline = time.time() + timeout         # Absolute time limit
    buf = b""                                # Accumulate boot text here
    while time.time() < deadline:
        chunk = ser.read(256)                # Read any available bytes
        if chunk:
            buf += chunk                     # Append to rolling buffer
            text = buf.decode(errors="ignore")  # Safe decode for ASCII-ish logs
            if "SELF-TEST PASS" in text or "[BOOT] READY" in text:
                return True                  # Boot complete
    return False                              # No banner seen (ok to proceed)


def send_cmd(ser, payload: bytes, expect_timeout: float = 4.0, expected_types=None):
    """
    Send one command as FT_DATA and wait for a reply of a specific TYPE.

    Whole-function summary:
      Builds the wire frame for the command (TYPE=0x01), transmits it, then
      reads frames until one of the expected TYPES is seen (e.g., ACK for PING).
      Any unrelated frames (like the 1 Hz FT_STATUS heartbeat) are parsed and
      discarded so they don't cause false timeouts.

    Parameters:
      ser            : open pySerial Serial instance
      payload        : the command payload (e.g., b'\\x01' for OP_PING)
      expect_timeout : max seconds to wait for the expected reply
      expected_types : set of integer TYPEs allowed for the reply.
                       If None, accepts any non-STATUS reply.

    Returns:
      (rtype:int, rpay:bytes) on success, or (None, None) on timeout.
    """
    import time

    # --- small, self-contained helpers (no external dependencies) ---

    def _crc16_ccitt_false(data: bytes) -> int:
        """Compute CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF)."""
        crc = 0xFFFF
        for b in data:
            crc ^= (b << 8)
            for _ in range(8):
                if (crc & 0x8000):
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def _build_frame(ftype: int, pay: bytes) -> bytes:
        """[AA 55][VER=0x01][TYPE][LEN_HI LEN_LO][PAY][CRC_HI CRC_LO]."""
        ver = 0x01
        ln  = len(pay)
        hdr = bytes([ver, ftype, (ln >> 8) & 0xFF, ln & 0xFF])
        crc = _crc16_ccitt_false(hdr + pay)
        return b"\xAA\x55" + hdr + pay + bytes([crc >> 8, crc & 0xFF])

    def _pop_first_frame(buf: bytearray):
        """
        Try to parse and pop the first complete/valid frame from 'buf'.

        Whole-function summary:
          Finds sync 0xAA55, verifies header/payload/CRC, and if valid returns
          (rtype:int, rpay:bytes, raw:bytes) while removing that frame from 'buf'.
          If CRC is bad, it discards the sync bytes and keeps searching.

        Returns:
          (rtype:int, rpay:bytes, raw:bytes|None) on success, or (None, None, None)
          if no full frame yet (caller should keep waiting).
        """
        # find sync
        i = buf.find(b"\xAA\x55")
        if i < 0:
            # no sync in buffer; drop everything
            buf.clear()
            return (None, None, None)
        # drop bytes before sync
        if i > 0:
            del buf[:i]
        # need at least header
        if len(buf) < 6:
            return (None, None, None)
        ver  = buf[2]
        rtyp = buf[3]
        ln   = (buf[4] << 8) | buf[5]
        total = 2 + 4 + ln + 2
        if len(buf) < total:
            return (None, None, None)
        # verify CRC
        body = bytes(buf[2: 2+4+ln])  # VER,TYPE,LEN, PAYLOAD
        rxcrc = (buf[2+4+ln] << 8) | buf[2+4+ln+1]
        if _crc16_ccitt_false(body) != rxcrc:
            # bad CRC: drop sync and continue later
            del buf[:2]
            return (None, None, None)
        # extract payload and remove frame from buffer
        rpay = bytes(buf[6: 6+ln])
        raw = bytes(buf[:total])
        del buf[:total]
        return (rtyp, rpay, raw)

    # --- transmit the command and wait for the expected reply ---

    # Build the command frame (FT_DATA = 0x01) and show TX bytes for visibility
    frame = _build_frame(0x01, payload)
    print("TX:", frame.hex(" "))

    # Clear any stale input (e.g., old heartbeat) before sending
    ser.reset_input_buffer()
    ser.write(frame); ser.flush()

    deadline = time.time() + expect_timeout
    rx = bytearray()
    last_frame_bytes = None
    last_status_payload = None

    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            rx += chunk
            while True:
                rtyp, rpay, raw = _pop_first_frame(rx)
                if rtyp is None:
                    break  # need more bytes
                if raw:
                    last_frame_bytes = raw
                if expected_types is None:
                    # Default path: ignore unsolicited status heartbeats while waiting for ACK/NACK.
                    if rtyp == FT_STATUS and len(rpay) == 3:
                        last_status_payload = rpay
                        continue
                    return (rtyp, rpay)
                else:
                    if rtyp in expected_types:
                        return (rtyp, rpay)
                    # Some older firmware revisions sent STATUS payloads with a different frame type.
                    if (FT_STATUS in expected_types) and len(rpay) == 3:
                        return (FT_STATUS, rpay)
                    if len(rpay) == 3:
                        last_status_payload = rpay

    # Timeout: show whatever we captured to aid debugging
    if rx:
        print("RX (raw):", rx.hex(" "))
    elif last_frame_bytes:
        typ = last_frame_bytes[3]
        payload = last_frame_bytes[6:-2]
        print(f"Last frame seen type=0x{typ:02X} len={len(payload)} payload={payload.hex(' ')}")
    elif last_status_payload is not None:
        print("Last STATUS payload (len=3) seen:", last_status_payload.hex(" "))
    return (None, None)

def main():
    """
    Open serial with DTR asserted (keeps TX path open), drain boot banner, then send.
    Requires a specific reply TYPE for each command to avoid false matches.
    """
    import argparse, sys, time, serial

    ap = argparse.ArgumentParser(description="Nano Sentinel CLI (quiet open)")
    ap.add_argument("port", help="e.g., /dev/cu.usbserial-1130 or /dev/tty.usbserial-1130")
    ap.add_argument("--baud", type=int, default=115200)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--ping", action="store_true")
    g.add_argument("--start", action="store_true")
    g.add_argument("--stop", action="store_true")
    g.add_argument("--set-rate", type=int, metavar="MS")
    g.add_argument("--get-status", action="store_true")
    args = ap.parse_args()

    # Open with DTR asserted (many USB-UARTs require this for TX), RTS kept low.
    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.timeout = 0.9
    ser.write_timeout = 0.9
    ser.xonxoff = False
    ser.rtscts  = False
    ser.dsrdtr  = False
    ser.dtr = True   # assert DTR so adapters don’t gate TX; will pulse reset once
    ser.rts = False
    try:
        ser.exclusive = True
    except Exception:
        pass
    ser.open()

    # Let the virtual COM port settle and drain any boot banner if we caused a reset.
    time.sleep(0.2)
    _ = drain_boot_banner(ser, timeout=3.0)

    # Payload + expected reply types
    expected = None
    if args.ping:
        payload  = bytes([0x01])   # OP_PING
        expected = {FT_ACK}
    elif args.start:
        payload  = bytes([0x02])   # OP_START
        expected = {FT_ACK, FT_NACK}
    elif args.stop:
        payload  = bytes([0x03])   # OP_STOP
        expected = {FT_ACK, FT_NACK}
    elif args.set_rate is not None:
        ms = args.set_rate
        if not (10 <= ms <= 10000):
            print("SET_RATE out of range (10..10000)"); sys.exit(2)
        payload  = bytes([0x04, (ms >> 8) & 0xFF, ms & 0xFF])
        expected = {FT_ACK, FT_NACK}
    else:
        payload  = bytes([0x05])   # OP_GET_STATUS
        expected = {FT_STATUS}     # FT_STATUS

    rtype, rpay = send_cmd(ser, payload, expect_timeout=6.0, expected_types=expected)

    if rtype is None:
        print("No reply (timeout)."); sys.exit(3)

    if rtype == FT_ACK:  # FT_ACK
        print(f"ACK: {rpay!r}")
    elif rtype == FT_STATUS and len(rpay) == 3:  # FT_STATUS
        on = "ON" if rpay[0] == 1 else "OFF"
        rate = (rpay[1] << 8) | rpay[2]
        print(f"STATUS: sampling={on}, rate_ms={rate}")
    elif rtype == FT_NACK and len(rpay) >= 1:  # FT_NACK
        print(f"NACK: reason={rpay[0]}")
    else:
        print(f"Reply type=0x{rtype:02X}, payload={rpay!r}")

if __name__ == "__main__":
    main()
