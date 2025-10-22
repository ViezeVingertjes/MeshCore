#!/usr/bin/env python3
"""
MeshCore KISS Modem - Send Chat Message

Sends a chat message to the default public channel.

Usage:
    python test_kiss_send_chat.py [--port COM4] [--name "User"] [--message "Hello!"]

Requirements:
    pip install pyserial pycryptodome
"""

import serial
import struct
import time
import argparse
import hashlib
import base64
from typing import Optional
from Crypto.Cipher import AES
from Crypto.Hash import HMAC, SHA256

FEND = 0xC0
FESC = 0xDB
TFEND = 0xDC
TFESC = 0xDD

CMD_DATA = 0x00
CMD_GET_IDENTITY = 0x01
CMD_RESP_IDENTITY = 0x11

ROUTE_TYPE_FLOOD = 0x01
PAYLOAD_TYPE_GRP_TXT = 0x05

PUBLIC_CHANNEL_PSK = "izOH6cXN6mrJ5e26oRXNcg=="


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
    
    def send_packet(self, packet: bytes):
        self._send_frame(CMD_DATA, packet)


def encrypt_group_message(psk: bytes, plaintext: bytes) -> bytes:
    padding_len = (16 - (len(plaintext) % 16)) % 16
    padded = plaintext + b'\x00' * padding_len
    
    cipher = AES.new(psk[:16], AES.MODE_ECB)
    ciphertext = cipher.encrypt(padded)
    
    h = HMAC.new(psk, digestmod=SHA256)
    h.update(ciphertext)
    mac = h.digest()[:2]
    
    return mac + ciphertext


def create_group_chat_message(psk: bytes, sender_name: str, message: str) -> bytes:
    timestamp = int(time.time())
    flags = 0x00
    
    text = f"{sender_name}: {message}"
    
    plaintext = struct.pack('<I', timestamp) + bytes([flags]) + text.encode('utf-8')
    
    encrypted = encrypt_group_message(psk, plaintext)
    
    channel_hash = hashlib.sha256(psk).digest()[0]
    
    header = ROUTE_TYPE_FLOOD | (PAYLOAD_TYPE_GRP_TXT << 2)
    payload = bytes([channel_hash]) + encrypted
    
    return bytes([header, 0]) + payload


def main():
    parser = argparse.ArgumentParser(description='MeshCore KISS Modem - Send Chat Message')
    parser.add_argument('--port', default='COM4', help='Serial port')
    parser.add_argument('--name', default='User', help='Sender name')
    parser.add_argument('--message', default='Hello from KISS modem!', help='Message text')
    args = parser.parse_args()
    
    print("MeshCore KISS Modem - Send Chat Message")
    print(f"Port: {args.port}")
    print(f"Channel: Public")
    print(f"Name: {args.name}")
    print(f"Message: {args.message}")
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
        
        packet = create_group_chat_message(psk, args.name, args.message)
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

