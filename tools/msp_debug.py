#!/usr/bin/env python3
# Simple MSPv2-native debug reader - prints MSP_DEBUGMSG payloads
# Usage: python3 msp_debug_reader.py /dev/ttyACM0 115200

import sys, serial, struct, time

MSP_DEBUGMSG = 253
POLY = 0xD5

def crc8_dvb_s2_update(crc, b):
    crc ^= b
    for _ in range(8):
        if crc & 0x80:
            crc = ((crc << 1) ^ POLY) & 0xFF
        else:
            crc = (crc << 1) & 0xFF
    return crc

def read_exact(ser, n):
    buf = b''
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

def main():
    if len(sys.argv) < 2:
        print("Usage: {} <device> [baud]".format(sys.argv[0])); sys.exit(1)
    dev = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    ser = serial.Serial(dev, baud, timeout=1)
    
    print("Opened", dev, "at", baud)
    
    try:
        while True:
            b = ser.read(1)
            if not b:
                continue
            # Wait for MSP start
            if b != b'$':
                continue
            next2 = read_exact(ser, 2)
            if not next2:
                continue
            # MSPv2 native: 'X<' or 'X>' ; MSPv1 / v2-over-v1: 'M<' or 'M>'
            if next2 in (b'X<', b'X>'):
                # MSPv2 native: flags(1) cmd(2 le) size(2 le)
                hdr = read_exact(ser, 5)
                if not hdr:
                    continue
                flags = hdr[0]
                cmd = hdr[1] | (hdr[2] << 8)
                size = hdr[3] | (hdr[4] << 8)
                if size < 0 or size > 65536:
                    _ = read_exact(ser, size + 1) if size else None
                    continue
                payload = read_exact(ser, size) if size else b''
                if payload is None:
                    continue
                crc_byte = read_exact(ser, 1)
                if not crc_byte:
                    continue
                # compute crc8 DVB-S2 over header+payload
                crc = 0
                for ch in hdr:
                    crc = crc8_dvb_s2_update(crc, ch)
                for ch in payload:
                    crc = crc8_dvb_s2_update(crc, ch)
                if crc != crc_byte[0]:
                    continue
                if cmd == MSP_DEBUGMSG:
                    try:
                        s = payload.decode('utf-8', errors='replace')
                    except Exception:
                        s = repr(payload)
                    s = s.rstrip('\x00').rstrip('\r\n')
                    print(s)
            elif next2 in (b'M<', b'M>'):
                # MSPv1 simple header: size(1) cmd(1)
                hdr = read_exact(ser, 2)
                if not hdr:
                    continue
                size = hdr[0]
                cmd = hdr[1]
                payload = read_exact(ser, size) if size else b''
                if payload is None:
                    continue
                crc_byte = read_exact(ser, 1)
                if not crc_byte:
                    continue
                # v1 checksum: XOR of size, cmd and payload
                chk = 0
                chk ^= hdr[0]
                chk ^= hdr[1]
                for ch in payload:
                    chk ^= ch
                if chk != crc_byte[0]:
                    continue
                if cmd == MSP_DEBUGMSG:
                    try:
                        s = payload.decode('utf-8', errors='replace')
                    except Exception:
                        s = repr(payload)
                    s = s.rstrip('\x00').rstrip('\r\n')
                    print(s)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == "__main__":
    main()