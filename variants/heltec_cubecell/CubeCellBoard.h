#pragma once

#include <MeshCore.h>
#include <Arduino.h>

class CubeCellBoard : public mesh::MainBoard {
  uint8_t _startup_reason;

public:
  CubeCellBoard() : _startup_reason(BD_STARTUP_NORMAL) {}

  void begin() {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, HIGH);
    pinMode(USER_KEY, INPUT);
  }

  uint16_t getBattMilliVolts() override {
    pinMode(VBAT_ADC_CTL, OUTPUT);
    digitalWrite(VBAT_ADC_CTL, LOW);
    delay(10);
    uint16_t raw = analogRead(ADC);
    digitalWrite(VBAT_ADC_CTL, HIGH);
    return (uint16_t)(raw * 2);
  }

  const char* getManufacturerName() const override {
    return DEVICE_NAME;
  }

  void reboot() override {
    NVIC_SystemReset();
  }

  uint8_t getStartupReason() const override {
    return _startup_reason;
  }

  void idle() {
    __WFI();
  }
};
