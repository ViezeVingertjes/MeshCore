#!/usr/bin/env python3
"""
MeshCore KISS Modem - Room Server Advertisement

Usage:
    python test_kiss_advert.py [--port COM4] [--name "My Room"]
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
CMD_SIGN_DATA = 0x04
CMD_RESP_IDENTITY = 0x11
CMD_RESP_SIGNATURE = 0x14

ROUTE_TYPE_FLOOD = 0x01
PAYLOAD_TYPE_ADVERT = 0x04
ADV_TYPE_ROOM = 0x03
ADV_FLAG_HAS_NAME = 0x80


class KISSModem:
    def __init__(self, port: str, baudrate: int = 115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
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
    
    def _receive_frame(self, timeout: float = 2.0) -> Optional[tuple]:
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
        frame = self._receive_frame()
        return frame[1] if frame and frame[0] == CMD_RESP_IDENTITY else None
    
    def sign_data(self, data: bytes) -> Optional[bytes]:
        self._send_frame(CMD_SIGN_DATA, data)
        frame = self._receive_frame()
        return frame[1] if frame and frame[0] == CMD_RESP_SIGNATURE else None
    
    def send_packet(self, packet: bytes):
        self._send_frame(CMD_DATA, packet)


def create_room_advert(modem: KISSModem, pub_key: bytes, name: str) -> bytes:
    appdata = bytearray([ADV_TYPE_ROOM | ADV_FLAG_HAS_NAME])
    appdata.extend(name.encode('utf-8'))
    
    timestamp = int(time.time())
    message = pub_key + struct.pack('<I', timestamp) + appdata
    
    signature = modem.sign_data(message)
    if not signature or len(signature) != 64:
        raise RuntimeError("Failed to get signature from device")
    
    header = ROUTE_TYPE_FLOOD | (PAYLOAD_TYPE_ADVERT << 2)
    payload = pub_key + struct.pack('<I', timestamp) + signature + appdata
    
    return bytes([header, 0]) + payload


def main():
    parser = argparse.ArgumentParser(description='MeshCore KISS Modem - Room Server Advertisement')
    parser.add_argument('--port', default='COM4', help='Serial port')
    parser.add_argument('--name', default='Test Room', help='Room name')
    args = parser.parse_args()
    
    print("MeshCore KISS Modem - Room Server Advertisement")
    print(f"Port: {args.port}")
    print(f"Room: {args.name}")
    print()
    
    try:
        modem = KISSModem(args.port)
        print("Connected")
        
        pub_key = modem.get_identity()
        if not pub_key:
            print("ERROR: Failed to get device identity")
            return
        print(f"Identity: {pub_key.hex()}")
        
        packet = create_room_advert(modem, pub_key, args.name)
        print(f"Packet: {len(packet)} bytes")
        
        modem.send_packet(packet)
        print("Sent")
        
        modem.close()
        
    except serial.SerialException as e:
        print(f"ERROR: {e}")
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
