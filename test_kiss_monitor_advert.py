#!/usr/bin/env python3
"""
MeshCore KISS Modem - Advertisement Monitor

Listens for incoming node advertisements and displays node information.

Usage:
    python test_kiss_monitor_advert.py [--port COM4]

Requirements:
    pip install pyserial
"""

import serial
import struct
import time
import argparse
from typing import Optional

FEND = 0xC0
FESC = 0xDB
TFEND = 0xDC
TFESC = 0xDD

CMD_DATA = 0x00
CMD_GET_IDENTITY = 0x01
CMD_RESP_IDENTITY = 0x11

ROUTE_TYPE_FLOOD = 0x01
PAYLOAD_TYPE_ADVERT = 0x04

ADV_TYPE_NODE = 0x00
ADV_TYPE_RELAY = 0x01
ADV_TYPE_GATEWAY = 0x02
ADV_TYPE_ROOM = 0x03

ADV_FLAG_HAS_LOCATION = 0x10
ADV_FLAG_HAS_NAME = 0x80


class KISSModem:
    def __init__(self, port: str, baudrate: int = 115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(0.5)
    
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
    
    def _escape(self, data: bytes) -> bytes:
        output = bytearray()
        for byte in data:
            if byte == FEND:
                output.extend([FESC, TFEND])
            elif byte == FESC:
                output.extend([FESC, TFESC])
            else:
                output.append(byte)
        return bytes(output)
    
    def _send_frame(self, command: int, data: bytes):
        frame = bytearray([FEND, command])
        frame.extend(self._escape(data))
        frame.append(FEND)
        self.ser.write(frame)
        self.ser.flush()
    
    def _receive_frame(self, timeout: float = 0.1) -> Optional[tuple]:
        start_time = time.time()
        in_frame = False
        escaped = False
        buffer = bytearray()
        
        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting:
                byte = self.ser.read(1)[0]
                
                if byte == FEND:
                    if in_frame and len(buffer) > 0:
                        return (buffer[0], bytes(buffer[1:]))
                    in_frame = True
                    buffer = bytearray()
                    escaped = False
                    continue
                
                if not in_frame:
                    continue
                
                if escaped:
                    buffer.append(FEND if byte == TFEND else FESC)
                    escaped = False
                elif byte == FESC:
                    escaped = True
                else:
                    buffer.append(byte)
        
        return None
    
    def get_identity(self) -> Optional[bytes]:
        self._send_frame(CMD_GET_IDENTITY, b'')
        frame = self._receive_frame(timeout=2.0)
        return frame[1] if frame and frame[0] == CMD_RESP_IDENTITY else None
    
    def listen_packets(self, callback):
        while True:
            frame = self._receive_frame(timeout=0.5)
            if frame and frame[0] == CMD_DATA:
                callback(frame[1])


def parse_advert_packet(packet: bytes) -> Optional[dict]:
    if len(packet) < 2:
        return None
    
    header = packet[0]
    path_len = packet[1]
    
    route_type = header & 0x03
    payload_type = (header >> 2) & 0x0F
    
    if payload_type != PAYLOAD_TYPE_ADVERT:
        return None
    
    payload_start = 2 + path_len
    if payload_start + 36 > len(packet):
        return None
    
    payload = packet[payload_start:]
    
    pub_key = payload[:32]
    timestamp = struct.unpack('<I', payload[32:36])[0]
    
    if len(payload) < 100:
        return None
    
    result = {
        'pub_key': pub_key.hex(),
        'node_hash': pub_key[0],
        'timestamp': timestamp,
        'snr': path_len,
        'type': 'node',
        'name': None,
        'location': None
    }
    
    if len(payload) > 100:
        appdata = payload[100:]
        if len(appdata) > 0:
            flags = appdata[0]
            offset = 1
            
            adv_type = flags & 0x0F
            type_names = {
                ADV_TYPE_NODE: 'node',
                ADV_TYPE_RELAY: 'relay',
                ADV_TYPE_GATEWAY: 'gateway',
                ADV_TYPE_ROOM: 'room_server'
            }
            result['type'] = type_names.get(adv_type, 'unknown')
            
            if flags & ADV_FLAG_HAS_LOCATION and len(appdata) >= offset + 8:
                lat = struct.unpack('<i', appdata[offset:offset+4])[0] / 1000000.0
                lon = struct.unpack('<i', appdata[offset+4:offset+8])[0] / 1000000.0
                result['location'] = (lat, lon)
                offset += 8
            
            if flags & ADV_FLAG_HAS_NAME and len(appdata) > offset:
                name = appdata[offset:].rstrip(b'\x00').decode('utf-8', errors='ignore')
                result['name'] = name
    
    return result


def main():
    parser = argparse.ArgumentParser(description='MeshCore KISS Modem - Advertisement Monitor')
    parser.add_argument('--port', default='COM4', help='Serial port')
    args = parser.parse_args()
    
    print("MeshCore KISS Modem - Advertisement Monitor")
    print(f"Port: {args.port}")
    print()
    
    try:
        modem = KISSModem(args.port)
        print("Connected")
        
        pub_key = modem.get_identity()
        if not pub_key:
            print("ERROR: Failed to get device identity")
            return
        print(f"Identity: {pub_key.hex()}")
        print()
        print("Listening for advertisements...")
        print("-" * 80)
        
        seen_nodes = set()
        
        def handle_packet(packet: bytes):
            result = parse_advert_packet(packet)
            if result:
                node_id = result['pub_key'][:16]
                
                if node_id not in seen_nodes:
                    seen_nodes.add(node_id)
                    
                    timestamp_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(result['timestamp']))
                    
                    print(f"Node: {result['node_hash']:#04x} [{result['type']}]")
                    print(f"  Public key: {result['pub_key']}")
                    print(f"  Timestamp: {timestamp_str}")
                    if result['name']:
                        print(f"  Name: {result['name']}")
                    if result['location']:
                        lat, lon = result['location']
                        print(f"  Location: {lat:.6f}, {lon:.6f}")
                    if result['snr'] > 0:
                        print(f"  Path length: {result['snr']}")
                    print()
        
        modem.listen_packets(handle_packet)
        
    except KeyboardInterrupt:
        print("\nStopped")
    except serial.SerialException as e:
        print(f"ERROR: {e}")
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

