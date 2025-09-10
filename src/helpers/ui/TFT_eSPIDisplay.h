#pragma once

#include "DisplayDriver.h"
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <helpers/RefCountedDigitalPin.h>

class TFT_eSPIDisplay : public DisplayDriver {
  TFT_eSPI display;
  bool _isOn;
  uint16_t _color;
  RefCountedDigitalPin* _peripher_power;

public:
  TFT_eSPIDisplay(RefCountedDigitalPin* peripher_power=NULL) : DisplayDriver(128, 160),
      _peripher_power(peripher_power)
  {
    _isOn = false;
  }

  bool begin();

  bool isOn() override { return _isOn; }
  void turnOn() override;
  void turnOff() override;
  void clear() override;
  void startFrame(Color bkg = DARK) override;
  void setTextSize(int sz) override;
  void setColor(Color c) override;
  void setCursor(int x, int y) override;
  void print(const char* str) override;
  void fillRect(int x, int y, int w, int h) override;
  void drawRect(int x, int y, int w, int h) override;
  void drawXbm(int x, int y, const uint8_t* bits, int w, int h) override;
  uint16_t getTextWidth(const char* str) override;
  void endFrame() override;
};

