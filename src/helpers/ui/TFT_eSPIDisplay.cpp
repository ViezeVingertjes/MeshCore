#include "TFT_eSPIDisplay.h"

#ifndef DISPLAY_ROTATION
  #define DISPLAY_ROTATION 0
#endif

#define SCALE_X  1.0f     // 128 / 128
#define SCALE_Y  2.5f     // 160 / 64

bool TFT_eSPIDisplay::begin() {
  if (!_isOn) {
    if (_peripher_power) _peripher_power->claim();

    #ifdef PIN_TFT_LEDA_CTL
    pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
    digitalWrite(PIN_TFT_LEDA_CTL, LOW);
    #endif

    #if PIN_TFT_RST != -1
    pinMode(PIN_TFT_RST, OUTPUT);
    digitalWrite(PIN_TFT_RST, HIGH);
    #endif

    display.init();                           // Use TFT_eSPI auto-initialization
    display.setRotation(DISPLAY_ROTATION);    // Portrait mode (0)
    display.fillScreen(TFT_BLACK);            // Clear to black
    
    // Set initial text properties
    display.setTextColor(TFT_WHITE);
    display.setTextSize(1);
    
    _isOn = true;
  }
  return true;
}

void TFT_eSPIDisplay::turnOn() {
  TFT_eSPIDisplay::begin();
}

void TFT_eSPIDisplay::turnOff() {
  if (_isOn) {
    #ifdef PIN_TFT_LEDA_CTL
      digitalWrite(PIN_TFT_LEDA_CTL, LOW);
    #endif
    
    #if PIN_TFT_RST != -1
      digitalWrite(PIN_TFT_RST, LOW);
    #endif
    
    _isOn = false;

    if (_peripher_power) _peripher_power->release();
  }
}

void TFT_eSPIDisplay::clear() {
  display.fillScreen(TFT_BLACK);
  #ifdef PIN_TFT_LEDA_CTL
    digitalWrite(PIN_TFT_LEDA_CTL, HIGH);
  #endif
}

void TFT_eSPIDisplay::startFrame(Color bkg) {
  display.fillScreen(TFT_BLACK);
  display.setTextColor(TFT_WHITE);
  display.setTextSize(1);
}

void TFT_eSPIDisplay::setTextSize(int sz) {
  display.setTextSize(sz);
}

void TFT_eSPIDisplay::setColor(Color c) {
  switch (c) {
    case DisplayDriver::DARK :
      _color = TFT_BLACK;
      break;
    case DisplayDriver::LIGHT : 
      _color = TFT_WHITE;
      break;
    case DisplayDriver::RED : 
      _color = TFT_RED;
      break;
    case DisplayDriver::GREEN : 
      _color = TFT_GREEN;
      break;
    case DisplayDriver::BLUE : 
      _color = TFT_BLUE;
      break;
    case DisplayDriver::YELLOW : 
      _color = TFT_YELLOW;
      break;
    case DisplayDriver::ORANGE : 
      _color = TFT_ORANGE;
      break;
    default:
      _color = TFT_WHITE;
      break;
  }
  display.setTextColor(_color);
}

void TFT_eSPIDisplay::setCursor(int x, int y) {
  display.setCursor(x*SCALE_X, y*SCALE_Y);
}

void TFT_eSPIDisplay::print(const char* str) {
  display.print(str);
  #ifdef PIN_TFT_LEDA_CTL
    digitalWrite(PIN_TFT_LEDA_CTL, HIGH);
  #endif
}

void TFT_eSPIDisplay::fillRect(int x, int y, int w, int h) {
  display.fillRect(x*SCALE_X, y*SCALE_Y, w*SCALE_X, h*SCALE_Y, _color);
  #ifdef PIN_TFT_LEDA_CTL
    digitalWrite(PIN_TFT_LEDA_CTL, HIGH);
  #endif
}

void TFT_eSPIDisplay::drawRect(int x, int y, int w, int h) {
  display.drawRect(x*SCALE_X, y*SCALE_Y, w*SCALE_X, h*SCALE_Y, _color);
  #ifdef PIN_TFT_LEDA_CTL
    digitalWrite(PIN_TFT_LEDA_CTL, HIGH);
  #endif
}

void TFT_eSPIDisplay::drawXbm(int x, int y, const uint8_t* bits, int w, int h) {
  display.drawBitmap(x*SCALE_X, y*SCALE_Y, bits, w, h, _color);
  #ifdef PIN_TFT_LEDA_CTL
    digitalWrite(PIN_TFT_LEDA_CTL, HIGH);
  #endif
}

uint16_t TFT_eSPIDisplay::getTextWidth(const char* str) {
  uint16_t w = display.textWidth(str);
  return w / SCALE_X;
}

void TFT_eSPIDisplay::endFrame() {
  #ifdef PIN_TFT_LEDA_CTL
    digitalWrite(PIN_TFT_LEDA_CTL, HIGH);
  #endif
}

