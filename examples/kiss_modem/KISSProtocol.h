#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

namespace kiss {

constexpr uint8_t FEND  = 0xC0;
constexpr uint8_t FESC  = 0xDB;
constexpr uint8_t TFEND = 0xDC;
constexpr uint8_t TFESC = 0xDD;

constexpr uint8_t CMD_DATA             = 0x00;
constexpr uint8_t CMD_GET_IDENTITY     = 0x01;
constexpr uint8_t CMD_SIGN_DATA        = 0x04;
constexpr uint8_t CMD_ENCRYPT_DATA     = 0x05;
constexpr uint8_t CMD_DECRYPT_DATA     = 0x06;
constexpr uint8_t CMD_KEY_EXCHANGE     = 0x07;
constexpr uint8_t CMD_HASH             = 0x08;
constexpr uint8_t CMD_RESP_IDENTITY    = 0x11;
constexpr uint8_t CMD_RESP_SIGNATURE   = 0x14;
constexpr uint8_t CMD_RESP_ENCRYPTED   = 0x15;
constexpr uint8_t CMD_RESP_DECRYPTED   = 0x16;
constexpr uint8_t CMD_RESP_SHARED_SECRET = 0x17;
constexpr uint8_t CMD_RESP_HASH        = 0x18;

class IKISSFrameHandler {
public:
  virtual ~IKISSFrameHandler() = default;
  virtual void onKISSFrame(uint8_t command, const uint8_t* data, size_t len) = 0;
};

class KISSProtocol {
private:
  static constexpr size_t MAX_FRAME_SIZE = 512;
  
  Stream* _serial;
  IKISSFrameHandler* _handler;
  
  uint8_t _rxBuffer[MAX_FRAME_SIZE];
  size_t _rxIndex;
  bool _inFrame;
  bool _escaped;
  
  void resetDecoder();
  void processReceivedByte(uint8_t byte);
  void handleCompleteFrame();
  
public:
  KISSProtocol(Stream& serial, IKISSFrameHandler& handler);
  void process();
  bool sendFrame(uint8_t command, const uint8_t* data, size_t len);
  void sendEscapedByte(uint8_t byte);
};

}

