#!/usr/bin/env python3
# Simple MSPv2-native debug reader - prints MSP_DEBUGMSG payloads
# Usage: python3 msp_debug_reader.py /dev/ttyACM0 115200

import argparse
import sys
import threading
import time
from serial import Serial

MSP_DEBUGMSG = 253
POLY = 0xD5

class PacketStore:
    def __init__(self):
        self._cond = threading.Condition()
        self._seq = 0
        self._packets = []

    def add(self, packet):
        with self._cond:
            self._seq += 1
            packet['seq'] = self._seq
            self._packets.append(packet)
            self._cond.notify_all()

    def last_seq(self):
        with self._cond:
            return self._seq

    def wait_response(self, protocol, cmd, after_seq, timeout_sec):
        deadline = time.time() + timeout_sec
        with self._cond:
            while True:
                for packet in self._packets:
                    if packet['seq'] <= after_seq:
                        continue
                    if packet['protocol'] != protocol:
                        continue
                    if packet['cmd'] != cmd:
                        continue
                    if packet['direction'] not in ('>', '!'):
                        continue
                    return packet

                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                self._cond.wait(timeout=remaining)

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

def parse_hex_payload(text):
    cleaned = text.replace(' ', '').replace(',', '').replace('_', '')
    if not cleaned:
        return b''
    if len(cleaned) % 2 != 0:
        raise ValueError("hex payload must have an even number of digits")
    return bytes.fromhex(cleaned)

def build_msp_v2_frame(cmd, payload=b'', flags=0):
    if cmd < 0 or cmd > 0xFFFF:
        raise ValueError("v2 command must be in range 0..65535")
    size = len(payload)
    if size > 0xFFFF:
        raise ValueError("v2 payload too large")

    hdr = bytes((flags & 0xFF, cmd & 0xFF, (cmd >> 8) & 0xFF, size & 0xFF, (size >> 8) & 0xFF))
    crc = 0
    for ch in hdr:
        crc = crc8_dvb_s2_update(crc, ch)
    for ch in payload:
        crc = crc8_dvb_s2_update(crc, ch)

    return b'$X<' + hdr + payload + bytes((crc,))

def build_msp_v1_frame(cmd, payload=b''):
    if cmd < 0 or cmd > 0xFF:
        raise ValueError("v1 command must be in range 0..255")
    size = len(payload)
    if size > 0xFF:
        raise ValueError("v1 payload too large")

    hdr = bytes((size, cmd))
    chk = 0
    for ch in hdr:
        chk ^= ch
    for ch in payload:
        chk ^= ch

    return b'$M<' + hdr + payload + bytes((chk,))

def process_msp_packet(cmd, payload):
    if cmd == MSP_DEBUGMSG:
        try:
            s = payload.decode('utf-8', errors='replace')
        except Exception:
            s = repr(payload)
        s = s.rstrip('\x00').rstrip('\r\n')
        print(s)

def format_payload_hex(payload):
    if not payload:
        return ""
    return payload.hex()

def print_msp_packet(packet):
    payload_hex = format_payload_hex(packet['payload'])
    line = "RX {}{} cmd={} len={}".format(
        packet['protocol'],
        packet['direction'],
        packet['cmd'],
        len(packet['payload'])
    )
    if payload_hex:
        line += " payload={}".format(payload_hex)
    if packet['protocol'] == 'v2':
        line += " flags=0x{:02X}".format(packet.get('flags', 0))
    print(line)

def rx_loop(ser, stop_event, packet_store, print_raw_text=False, verbose_packets=False):
    while not stop_event.is_set():
        b = ser.read(1)
        if not b:
            continue
        if print_raw_text and b != b'$':
            try:
                sys.stdout.write(b.decode('utf-8', errors='replace'))
                sys.stdout.flush()
            except Exception:
                pass
            continue
        if b != b'$':
            continue
        next2 = read_exact(ser, 2)
        if not next2:
            continue

        if next2 in (b'X<', b'X>'):
            hdr = read_exact(ser, 5)
            if not hdr:
                continue
            direction = chr(next2[1])
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

            crc = 0
            for ch in hdr:
                crc = crc8_dvb_s2_update(crc, ch)
            for ch in payload:
                crc = crc8_dvb_s2_update(crc, ch)
            if crc != crc_byte[0]:
                continue

            packet = {
                'protocol': 'v2',
                'direction': direction,
                'cmd': cmd,
                'payload': payload,
                'flags': flags,
            }
            packet_store.add(packet)
            process_msp_packet(cmd, payload)
            if verbose_packets:
                print_msp_packet(packet)
        elif next2 in (b'M<', b'M>', b'M!'):
            hdr = read_exact(ser, 2)
            if not hdr:
                continue
            direction = chr(next2[1])
            size = hdr[0]
            cmd = hdr[1]
            payload = read_exact(ser, size) if size else b''
            if payload is None:
                continue
            crc_byte = read_exact(ser, 1)
            if not crc_byte:
                continue

            chk = 0
            chk ^= hdr[0]
            chk ^= hdr[1]
            for ch in payload:
                chk ^= ch
            if chk != crc_byte[0]:
                continue

            packet = {
                'protocol': 'v1',
                'direction': direction,
                'cmd': cmd,
                'payload': payload,
            }
            packet_store.add(packet)
            process_msp_packet(cmd, payload)
            if verbose_packets:
                print_msp_packet(packet)

def send_and_wait(ser, frame, protocol, cmd, packet_store, timeout_sec):
    marker = packet_store.last_seq()
    ser.write(frame)
    response = packet_store.wait_response(protocol, cmd, marker, timeout_sec)
    return response

def run_cli(ser, stop_event, packet_store, response_timeout):
    print("CLI mode enabled")
    print("Commands:")
    print("  v2 <cmd> [payload_hex]      send MSPv2 and print response")
    print("  v1 <cmd> [payload_hex]      send MSPv1 and print response")
    print("  raw <bytes_hex>             e.g. raw 24583c0001000000d8")
    print("  cli <text>                  send plain CLI text + newline")
    print("  <text>                      shortcut for cli <text>")
    print("  timeout <seconds>           change response timeout")
    print("  exit                        quit")
    while not stop_event.is_set():
        try:
            line = input("msp> ").strip()
        except EOFError:
            break
        except KeyboardInterrupt:
            break

        if not line:
            continue

        low = line.lower()
        if low in ("exit", "quit"):
            break
        if low in ("help", "?"):
            print("  v2 <cmd> [payload_hex]")
            print("  v1 <cmd> [payload_hex]")
            print("  raw <bytes_hex>")
            print("  cli <text>")
            print("  <text>")
            print("  timeout <seconds>")
            print("  exit")
            continue

        parts = line.split(maxsplit=2)
        cmd_type = parts[0].lower()

        try:
            if cmd_type == "raw":
                if len(parts) < 2:
                    raise ValueError("raw requires hex bytes")
                frame = parse_hex_payload(parts[1] if len(parts) == 2 else (parts[1] + parts[2]))
                ser.write(frame)
                print("sent {} bytes".format(len(frame)))
            elif cmd_type == "v2":
                if len(parts) < 2:
                    raise ValueError("v2 requires a command id")
                try:
                    cmd = int(parts[1], 0)
                except ValueError:
                    raise ValueError("v2 command id must be a number (e.g. 123 or 0x7B)")
                payload = parse_hex_payload(parts[2]) if len(parts) > 2 else b''
                frame = build_msp_v2_frame(cmd, payload)
                response = send_and_wait(ser, frame, 'v2', cmd, packet_store, response_timeout[0])
                print("sent {} bytes".format(len(frame)))
                if response is None:
                    print("timeout waiting for v2 response to cmd {}".format(cmd))
                else:
                    print_msp_packet(response)
            elif cmd_type == "v1":
                if len(parts) < 2:
                    raise ValueError("v1 requires a command id")
                try:
                    cmd = int(parts[1], 0)
                except ValueError:
                    raise ValueError("v1 command id must be a number (e.g. 110 or 0x6E)")
                payload = parse_hex_payload(parts[2]) if len(parts) > 2 else b''
                frame = build_msp_v1_frame(cmd, payload)
                response = send_and_wait(ser, frame, 'v1', cmd, packet_store, response_timeout[0])
                print("sent {} bytes".format(len(frame)))
                if response is None:
                    print("timeout waiting for v1 response to cmd {}".format(cmd))
                else:
                    print_msp_packet(response)
            elif cmd_type == "cli":
                if len(parts) < 2:
                    raise ValueError("cli requires text, e.g. cli serial")
                text = line[len(parts[0]):].lstrip()
                frame = (text + "\n").encode('utf-8', errors='replace')
                ser.write(frame)
                print("sent {} bytes".format(len(frame)))
            elif cmd_type == "timeout":
                if len(parts) < 2:
                    raise ValueError("timeout requires a numeric value")
                value = float(parts[1])
                if value <= 0:
                    raise ValueError("timeout must be > 0")
                response_timeout[0] = value
                print("response timeout set to {:.2f}s".format(value))
            else:
                frame = (line + "\n").encode('utf-8', errors='replace')
                ser.write(frame)
                print("sent {} bytes".format(len(frame)))
        except ValueError as e:
            print("error:", e)

def main():
    parser = argparse.ArgumentParser(description="MSP debug reader with optional interactive CLI sender")
    parser.add_argument("device", help="serial device, e.g. /dev/ttyACM0")
    parser.add_argument("baud", nargs="?", type=int, default=115200, help="baud rate (default: 115200)")
    parser.add_argument("--cli", action="store_true", help="enable interactive command sender")
    parser.add_argument("--timeout", type=float, default=1.5, help="response wait timeout in seconds (default: 1.5)")
    parser.add_argument("--verbose-rx", action="store_true", help="print all received MSP packets")
    args = parser.parse_args()

    dev = args.device
    baud = args.baud
    ser = Serial(dev, baud, timeout=1)
    
    print("Opened", dev, "at", baud)

    stop_event = threading.Event()
    packet_store = PacketStore()
    rx_thread = threading.Thread(
        target=rx_loop,
        args=(ser, stop_event, packet_store, args.cli, args.verbose_rx),
        daemon=True,
    )
    rx_thread.start()
    response_timeout = [args.timeout]

    try:
        if args.cli:
            run_cli(ser, stop_event, packet_store, response_timeout)
        else:
            # Passive listening mode - LOG messages are pushed automatically
            print("Listening for debug messages (Ctrl+C to stop)...")
            print("Make sure 'serial 20 32769' is configured (LOG + MSP on VCP)")
            while True:
                time.sleep(1)  # Just keep alive, rx_thread handles incoming messages
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        rx_thread.join(timeout=1.5)
        ser.close()

if __name__ == "__main__":
    main()