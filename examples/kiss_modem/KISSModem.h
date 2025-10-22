#pragma once

#include <Arduino.h>
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <helpers/AdvertDataHelpers.h>
#include <RTClib.h>
#include <target.h>

#include "KISSProtocol.h"

#ifndef FIRMWARE_BUILD_DATE
  #define FIRMWARE_BUILD_DATE   "22 Oct 2025"
#endif

#ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION   "v1.0.0"
#endif

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     250
#endif
#ifndef LORA_SF
  #define LORA_SF     10
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif

#define FIRMWARE_ROLE "kiss_modem"

struct ModemStats {
  uint32_t packets_received;
  uint32_t packets_sent;
  uint32_t frames_from_serial;
  uint32_t frames_to_serial;
  uint32_t total_airtime_secs;
  uint32_t total_uptime_secs;
  int16_t last_rssi;
  int16_t last_snr;
};

class KISSModem : public mesh::Mesh, public kiss::IKISSFrameHandler {
private:
  FILESYSTEM* _fs;
  kiss::KISSProtocol* _kiss;
  ModemStats _stats;
  bool _transmittingFromSerial;
  
  void sendIdentityResponse();
  void handleSignRequest(const uint8_t* data, size_t len);
  void handleDataFrame(const uint8_t* data, size_t len);
  bool isValidPacketData(const uint8_t* data, size_t len) const;
  void transmitPacket(mesh::Packet* packet);
  
protected:
  mesh::DispatcherAction onRecvPacket(mesh::Packet* pkt) override;
  
  float getAirtimeBudgetFactor() const override {
    return 10.0f;
  }
  
  bool allowPacketForward(const mesh::Packet* packet) override {
    return !_transmittingFromSerial;
  }
  
public:
  KISSModem(mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, 
            mesh::RTCClock& rtc, mesh::MeshTables& tables, Stream& serial);
  
  virtual ~KISSModem();
  
  void begin(FILESYSTEM* fs);
  void loop();
  const char* getFirmwareVersion() const { return FIRMWARE_VERSION; }
  const char* getFirmwareBuildDate() const { return FIRMWARE_BUILD_DATE; }
  const char* getFirmwareRole() const { return FIRMWARE_ROLE; }
  void onKISSFrame(uint8_t command, const uint8_t* data, size_t len) override;
  const ModemStats& getStats() const { return _stats; }
};
