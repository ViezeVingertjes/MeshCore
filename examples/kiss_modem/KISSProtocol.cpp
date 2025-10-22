#include "KISSProtocol.h"

namespace kiss {

KISSProtocol::KISSProtocol(Stream& serial, IKISSFrameHandler& handler)
  : _serial(&serial), _handler(&handler), _rxIndex(0), _inFrame(false), _escaped(false)
{
}

void KISSProtocol::resetDecoder() {
  _rxIndex = 0;
  _inFrame = false;
  _escaped = false;
}

void KISSProtocol::process() {
  while (_serial->available()) {
    uint8_t byte = _serial->read();
    processReceivedByte(byte);
  }
}

void KISSProtocol::processReceivedByte(uint8_t byte) {
  if (byte == FEND) {
    if (_inFrame && _rxIndex > 0) {
      handleCompleteFrame();
    }
    resetDecoder();
    _inFrame = true;
    return;
  }
  
  if (!_inFrame) return;
  
  if (_escaped) {
    _escaped = false;
    if (byte == TFEND) {
      byte = FEND;
    } else if (byte == TFESC) {
      byte = FESC;
    } else {
      resetDecoder();
      return;
    }
  } else if (byte == FESC) {
    _escaped = true;
    return;
  }
  
  if (_rxIndex < MAX_FRAME_SIZE) {
    _rxBuffer[_rxIndex++] = byte;
  } else {
    resetDecoder();
  }
}

void KISSProtocol::handleCompleteFrame() {
  if (_rxIndex < 1) return;
  
  uint8_t command = _rxBuffer[0];
  const uint8_t* data = &_rxBuffer[1];
  size_t dataLen = _rxIndex - 1;
  
  if (_handler) {
    _handler->onKISSFrame(command, data, dataLen);
  }
}

bool KISSProtocol::sendFrame(uint8_t command, const uint8_t* data, size_t len) {
  if (!_serial) return false;
  
  _serial->write(FEND);
  sendEscapedByte(command);
  
  for (size_t i = 0; i < len; i++) {
    sendEscapedByte(data[i]);
  }
  
  _serial->write(FEND);
  _serial->flush();
  
  return true;
}

void KISSProtocol::sendEscapedByte(uint8_t byte) {
  if (byte == FEND) {
    _serial->write(FESC);
    _serial->write(TFEND);
  } else if (byte == FESC) {
    _serial->write(FESC);
    _serial->write(TFESC);
  } else {
    _serial->write(byte);
  }
}

}

