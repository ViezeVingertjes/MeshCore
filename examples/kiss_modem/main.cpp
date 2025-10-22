#include <Arduino.h>
#include "KISSModem.h"

StdRNG fast_rng;
SimpleMeshTables tables;
KISSModem the_modem(radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables, Serial);

void halt() {
  while (1) delay(1000);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  board.begin();
  
  if (!radio_init()) halt();
  
  fast_rng.begin(radio_get_rng_seed());
  
  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  fs->remove("_main.id");  // Clear identity to generate new one
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  fs->remove("/identity/_main.id");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  fs->remove("/identity/_main.id");
#else
  #error "need to define filesystem"
#endif
  
  radio_set_params(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
  radio_set_tx_power(LORA_TX_POWER);
  
  the_modem.begin(fs);
}

void loop() {
  the_modem.loop();
  yield();
}

