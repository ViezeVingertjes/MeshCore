#!/usr/bin/env python3
"""
MeshCore KISS Modem - Chat Message Monitor

Listens for incoming chat messages on the default public channel.

Usage:
    python test_kiss_monitor_chat.py [--port COM4]

Requirements:
    pip install pyserial
"""

import serial
import struct
import time
import argparse
import base64
from typing import Optional

FEND = 0xC0
FESC = 0xDB
TFEND = 0xDC
TFESC = 0xDD

CMD_DATA = 0x00
CMD_GET_IDENTITY = 0x01
CMD_DECRYPT_DATA = 0x06
CMD_HASH = 0x08
CMD_RESP_IDENTITY = 0x11
CMD_RESP_DECRYPTED = 0x16
CMD_RESP_HASH = 0x18

ROUTE_TYPE_FLOOD = 0x01
PAYLOAD_TYPE_GRP_TXT = 0x05

PUBLIC_CHANNEL_PSK = "izOH6cXN6mrJ5e26oRXNcg=="


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
    
    def decrypt_data(self, psk: bytes, mac_and_ciphertext: bytes) -> Optional[bytes]:
        self._send_frame(CMD_DECRYPT_DATA, psk + mac_and_ciphertext)
        frame = self._receive_frame(timeout=1.0)
        return frame[1] if frame and frame[0] == CMD_RESP_DECRYPTED else None
    
    def hash_data(self, data: bytes) -> Optional[bytes]:
        self._send_frame(CMD_HASH, data)
        frame = self._receive_frame(timeout=1.0)
        return frame[1] if frame and frame[0] == CMD_RESP_HASH else None
    
    def listen_packets(self, callback):
        while True:
            frame = self._receive_frame(timeout=0.5)
            if frame and frame[0] == CMD_DATA:
                callback(frame[1])


def parse_group_text_packet(modem: 'KISSModem', packet: bytes, psk: bytes, channel_hash: int) -> Optional[dict]:
    if len(packet) < 2:
        return None
    
    header = packet[0]
    path_len = packet[1]
    
    route_type = header & 0x03
    payload_type = (header >> 2) & 0x0F
    
    if payload_type != PAYLOAD_TYPE_GRP_TXT:
        return None
    
    payload_start = 2 + path_len
    if payload_start + 1 >= len(packet):
        return None
    
    payload = packet[payload_start:]
    
    pkt_channel_hash = payload[0]
    if pkt_channel_hash != channel_hash:
        return None
    
    mac_and_ciphertext = payload[1:]
    
    plaintext = modem.decrypt_data(psk, mac_and_ciphertext)
    if not plaintext:
        return None
    
    if len(plaintext) < 5:
        return None
    
    timestamp = struct.unpack('<I', plaintext[:4])[0]
    flags = plaintext[4]
    message = plaintext[5:].rstrip(b'\x00').decode('utf-8', errors='ignore')
    
    return {
        'timestamp': timestamp,
        'flags': flags,
        'message': message,
        'snr': path_len
    }


def main():
    parser = argparse.ArgumentParser(description='MeshCore KISS Modem - Chat Message Monitor')
    parser.add_argument('--port', default='COM4', help='Serial port')
    args = parser.parse_args()
    
    print("MeshCore KISS Modem - Chat Message Monitor")
    print(f"Port: {args.port}")
    print(f"Channel: Public")
    print()
    
    psk = base64.b64decode(PUBLIC_CHANNEL_PSK)
    
    try:
        modem = KISSModem(args.port)
        print("Connected")
        
        pub_key = modem.get_identity()
        if not pub_key:
            print("ERROR: Failed to get device identity")
            return
        print(f"Identity: {pub_key.hex()}")
        
        psk_hash = modem.hash_data(psk)
        if not psk_hash:
            print("ERROR: Failed to hash PSK")
            return
        channel_hash = psk_hash[0]
        
        print(f"Channel PSK: {psk.hex()}")
        print(f"Channel hash: {channel_hash:#04x}")
        print()
        print("Listening for messages...")
        print("-" * 60)
        
        def handle_packet(packet: bytes):
            result = parse_group_text_packet(modem, packet, psk, channel_hash)
            if result:
                timestamp_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(result['timestamp']))
                print(f"[{timestamp_str}] {result['message']}")
        
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

