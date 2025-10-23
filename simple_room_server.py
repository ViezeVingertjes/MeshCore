#!/usr/bin/env python3
"""
MeshCore Simple Room Server (Python)

Usage: python simple_room_server.py [--port COM4] [--name "Simple BBS"]
"""

import serial
import struct
import time
import argparse
import os
import hashlib
from typing import Optional

# KISS Protocol
FEND = 0xC0
FESC = 0xDB
TFEND = 0xDC
TFESC = 0xDD

CMD_DATA = 0x00
CMD_GET_IDENTITY = 0x01
CMD_SIGN_DATA = 0x04
CMD_ENCRYPT_DATA = 0x05
CMD_DECRYPT_DATA = 0x06
CMD_KEY_EXCHANGE = 0x07
CMD_RESP_IDENTITY = 0x11
CMD_RESP_SIGNATURE = 0x14
CMD_RESP_ENCRYPTED = 0x15
CMD_RESP_DECRYPTED = 0x16
CMD_RESP_SHARED_SECRET = 0x17

# Mesh Protocol
ROUTE_TYPE_FLOOD = 0x01
ROUTE_TYPE_DIRECT = 0x02
PAYLOAD_TYPE_RESPONSE = 0x01
PAYLOAD_TYPE_ADVERT = 0x04
PAYLOAD_TYPE_ANON_REQ = 0x07
PAYLOAD_TYPE_PATH = 0x08

ADV_TYPE_ROOM = 0x03
ADV_FLAG_HAS_NAME = 0x80
RESP_SERVER_LOGIN_OK = 0x00
PERM_ACL_READ_WRITE = 0x01

PUB_KEY_SIZE = 32
CIPHER_MAC_SIZE = 2

# Configuration
ADVERT_INTERVAL_SECONDS = 120
SERVER_RESPONSE_DELAY = 0.300
DEDUP_WINDOW_SECONDS = 3.0
DEDUP_CACHE_CLEANUP_INTERVAL = 10.0
FIRMWARE_VER_LEVEL = 1


class KISSModem:
    """KISS protocol interface for MeshCore devices"""
    
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
    
    def get_shared_secret(self, other_pub_key: bytes) -> Optional[bytes]:
        self._send_frame(CMD_KEY_EXCHANGE, other_pub_key)
        frame = self._receive_frame()
        return frame[1] if frame and frame[0] == CMD_RESP_SHARED_SECRET else None
    
    def decrypt_data(self, shared_secret: bytes, mac_and_ciphertext: bytes) -> Optional[bytes]:
        self._send_frame(CMD_DECRYPT_DATA, shared_secret + mac_and_ciphertext)
        frame = self._receive_frame()
        return frame[1] if frame and frame[0] == CMD_RESP_DECRYPTED else None
    
    def encrypt_data(self, shared_secret: bytes, plaintext: bytes) -> Optional[bytes]:
        self._send_frame(CMD_ENCRYPT_DATA, shared_secret + plaintext)
        frame = self._receive_frame()
        return frame[1] if frame and frame[0] == CMD_RESP_ENCRYPTED else None
    
    def send_packet(self, packet: bytes):
        self._send_frame(CMD_DATA, packet)
    
    def receive_packet(self, timeout: float = 0.1) -> Optional[bytes]:
        frame = self._receive_frame(timeout)
        return frame[1] if frame and frame[0] == CMD_DATA else None


class SimpleRoomServer:
    """Room server with login handling and packet deduplication"""
    
    def __init__(self, modem: KISSModem, pub_key: bytes, room_name: str):
        self.modem = modem
        self.pub_key = pub_key
        self.room_name = room_name
        self.my_hash = pub_key[0]
        self.next_advert_time = 0
        self.packet_cache = {}
        self.next_cache_cleanup = 0
    
    def _log(self, msg: str):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}")
    
    def create_room_advert(self) -> bytes:
        appdata = bytearray([ADV_TYPE_ROOM | ADV_FLAG_HAS_NAME])
        appdata.extend(self.room_name.encode('utf-8'))
        
        timestamp = int(time.time())
        message = self.pub_key + struct.pack('<I', timestamp) + appdata
        
        signature = self.modem.sign_data(message)
        if not signature or len(signature) != 64:
            raise RuntimeError("Failed to get signature")
        
        header = ROUTE_TYPE_FLOOD | (PAYLOAD_TYPE_ADVERT << 2)
        payload = self.pub_key + struct.pack('<I', timestamp) + signature + appdata
        
        return bytes([header, 0]) + payload
    
    def send_advertisement(self):
        try:
            self.modem.send_packet(self.create_room_advert())
        except Exception as e:
            self._log(f"Advertisement error: {e}")
    
    def parse_packet(self, packet: bytes) -> Optional[dict]:
        if len(packet) < 2:
            return None
        
        header, path_len = packet[0], packet[1]
        
        return {
            'route_type': header & 0x03,
            'payload_type': (header >> 2) & 0x0F,
            'path_len': path_len,
            'path': packet[2:2+path_len] if path_len > 0 else b'',
            'payload': packet[2+path_len:]
        }
    
    def get_packet_key(self, parsed: dict) -> Optional[tuple]:
        if parsed['payload_type'] != PAYLOAD_TYPE_ANON_REQ:
            return None
        
        payload = parsed['payload']
        if len(payload) < 1 + PUB_KEY_SIZE + CIPHER_MAC_SIZE + 16:
            return None
        
        sender_pub_key = payload[1:1 + PUB_KEY_SIZE]
        payload_hash = hashlib.sha256(payload).digest()[:4]
        
        return (bytes(sender_pub_key), bytes(payload_hash))
    
    def cache_packet(self, parsed: dict):
        packet_key = self.get_packet_key(parsed)
        if not packet_key:
            return
        
        path_len = parsed['path_len']
        
        if packet_key in self.packet_cache:
            cached_parsed, cached_time, cached_path_len = self.packet_cache[packet_key]
            if path_len < cached_path_len:
                self.packet_cache[packet_key] = (parsed, cached_time, path_len)
        else:
            self.packet_cache[packet_key] = (parsed, time.time(), path_len)
    
    def process_cached_packets(self):
        current_time = time.time()
        keys_to_remove = []
        
        for packet_key, (parsed, arrival_time, path_len) in self.packet_cache.items():
            if current_time - arrival_time >= DEDUP_WINDOW_SECONDS:
                self.handle_anon_request_immediate(parsed)
                keys_to_remove.append(packet_key)
        
        for key in keys_to_remove:
            del self.packet_cache[key]
    
    def cleanup_old_cache_entries(self):
        current_time = time.time()
        keys_to_remove = [
            key for key, (_, arrival_time, _) in self.packet_cache.items()
            if current_time - arrival_time > 5.0
        ]
        
        for key in keys_to_remove:
            del self.packet_cache[key]
    
    def handle_anon_request_immediate(self, parsed: dict):
        payload = parsed['payload']
        
        if len(payload) < 1 + PUB_KEY_SIZE + CIPHER_MAC_SIZE:
            return
        
        dest_hash = payload[0]
        if dest_hash != self.my_hash:
            return
        
        sender_pub_key = payload[1:1 + PUB_KEY_SIZE]
        mac_and_ciphertext = payload[1 + PUB_KEY_SIZE:]
        
        shared_secret = self.modem.get_shared_secret(sender_pub_key)
        if not shared_secret:
            self._log("ERROR: Failed to calculate shared secret")
            return
        
        decrypted = self.modem.decrypt_data(shared_secret, mac_and_ciphertext)
        if not decrypted or len(decrypted) < 8:
            self._log("ERROR: Failed to decrypt login request")
            return
        
        sender_timestamp = struct.unpack('<I', decrypted[0:4])[0]
        password = decrypted[8:].rstrip(b'\x00').decode('utf-8', errors='ignore')
        
        route_type = "DIRECT" if parsed['route_type'] == ROUTE_TYPE_DIRECT else "FLOOD"
        self._log(f"Login from {sender_pub_key[:4].hex()}... ({route_type}, {parsed['path_len']} hops, pw: '{password}')")
        
        response_data = bytearray()
        response_data.extend(struct.pack('<I', int(time.time())))
        response_data.append(RESP_SERVER_LOGIN_OK)
        response_data.append(0)
        response_data.append(0)
        response_data.append(PERM_ACL_READ_WRITE)
        response_data.extend(os.urandom(4))
        response_data.append(FIRMWARE_VER_LEVEL)
        
        sender_hash = sender_pub_key[0]
        
        if parsed['route_type'] == ROUTE_TYPE_FLOOD:
            response_packet = self._create_path_return(sender_hash, parsed['path'], shared_secret, bytes(response_data))
        else:
            encrypted = self.modem.encrypt_data(shared_secret, bytes(response_data))
            if not encrypted:
                self._log("ERROR: Failed to encrypt response")
                return
            response_packet = self._create_response(sender_hash, encrypted)
        
        if response_packet:
            time.sleep(SERVER_RESPONSE_DELAY)
            self.modem.send_packet(response_packet)
            self._log(f"Response sent to {sender_pub_key[:4].hex()}...")
    
    def _create_response(self, dest_hash: int, encrypted: bytes) -> bytes:
        header = ROUTE_TYPE_FLOOD | (PAYLOAD_TYPE_RESPONSE << 2)
        payload = bytearray([header, 0, dest_hash, self.my_hash])
        payload.extend(encrypted)
        return bytes(payload)
    
    def _create_path_return(self, dest_hash: int, path: bytes, shared_secret: bytes, response_data: bytes) -> bytes:
        """PATH packet with embedded RESPONSE and return route"""
        path_data = bytearray([len(path)])
        path_data.extend(path)
        path_data.append(PAYLOAD_TYPE_RESPONSE)
        path_data.extend(response_data)
        
        encrypted_path = self.modem.encrypt_data(shared_secret, bytes(path_data))
        if not encrypted_path:
            return None
        
        header = ROUTE_TYPE_FLOOD | (PAYLOAD_TYPE_PATH << 2)
        payload = bytearray([header, 0, dest_hash, self.my_hash])
        payload.extend(encrypted_path)
        
        return bytes(payload)
    
    def handle_packet(self, packet: bytes):
        parsed = self.parse_packet(packet)
        if parsed and parsed['payload_type'] == PAYLOAD_TYPE_ANON_REQ:
            self.cache_packet(parsed)
    
    def run(self):
        print(f"\nRoom: {self.room_name}")
        print(f"Hash: {self.my_hash:#04x}")
        print(f"Identity: {self.pub_key.hex()}")
        print("\nRunning... (Ctrl+C to stop)\n")
        
        self.send_advertisement()
        self.next_advert_time = time.time() + ADVERT_INTERVAL_SECONDS
        self.next_cache_cleanup = time.time() + DEDUP_CACHE_CLEANUP_INTERVAL
        
        try:
            while True:
                current_time = time.time()
                
                packet = self.modem.receive_packet(timeout=0.05)
                if packet:
                    self.handle_packet(packet)
                
                self.process_cached_packets()
                
                if current_time >= self.next_cache_cleanup:
                    self.cleanup_old_cache_entries()
                    self.next_cache_cleanup = current_time + DEDUP_CACHE_CLEANUP_INTERVAL
                
                if current_time >= self.next_advert_time:
                    self.send_advertisement()
                    self.next_advert_time = current_time + ADVERT_INTERVAL_SECONDS
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nStopped")


def main():
    parser = argparse.ArgumentParser(description='MeshCore Simple Room Server')
    parser.add_argument('--port', default='COM4', help='Serial port')
    parser.add_argument('--name', default='Simple BBS', help='Room name')
    args = parser.parse_args()
    
    print("MeshCore Simple Room Server")
    print(f"Port: {args.port}")
    
    try:
        modem = KISSModem(args.port)
        print("Connected")
        
        pub_key = modem.get_identity()
        if not pub_key:
            print("ERROR: Failed to get identity")
            return
        
        server = SimpleRoomServer(modem, pub_key, args.name)
        server.run()
        
    except serial.SerialException as e:
        print(f"ERROR: {e}")
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

