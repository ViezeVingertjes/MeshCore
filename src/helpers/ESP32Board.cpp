#ifdef ESP_PLATFORM

#include "ESP32Board.h"
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

#if defined(ADMIN_PASSWORD) && !defined(DISABLE_WIFI_OTA)   // Repeater or Room Server only
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#include <SPIFFS.h>

bool ESP32Board::startOTAUpdate(const char* id, char reply[]) {
  WiFi.softAP("MeshCore-OTA", NULL);

  sprintf(reply, "Started: http://%s/update", WiFi.softAPIP().toString().c_str());
  MESH_DEBUG_PRINTLN("startOTAUpdate: %s", reply);

  static char id_buf[60];
  sprintf(id_buf, "%s (%s)", id, getManufacturerName());
  static char home_buf[90];
  sprintf(home_buf, "<H2>Hi! I am a MeshCore Repeater. ID: %s</H2>", id);

  AsyncWebServer* server = new AsyncWebServer(80);

  server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", home_buf);
  });
  server->on("/log", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/packet_log", "text/plain");
  });

  AsyncElegantOTA.setID(id_buf);
  AsyncElegantOTA.begin(server);    // Start ElegantOTA
  server->begin();

  return true;
}

#else
bool ESP32Board::startOTAUpdate(const char* id, char reply[]) {
  return false; // not supported
}
#endif

#endif

#ifdef ESP_PLATFORM

void ESP32Board::enterLightSleep(uint32_t idle_timeout_ms) {
  if (idle_timeout_ms == 0) return;
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

#ifdef P_LORA_DIO_1
  gpio_set_direction((gpio_num_t)P_LORA_DIO_1, GPIO_MODE_INPUT);
  gpio_wakeup_enable((gpio_num_t)P_LORA_DIO_1, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
#endif

  esp_sleep_enable_timer_wakeup((uint64_t)idle_timeout_ms * 1000ULL);
  esp_light_sleep_start();
}

void ESP32Board::enterDeepSleep(uint32_t seconds, int wake_pin) {
  if (wake_pin >= 0) {
    gpio_set_direction((gpio_num_t)wake_pin, GPIO_MODE_INPUT);
    esp_sleep_enable_ext1_wakeup(1ULL << wake_pin, ESP_EXT1_WAKEUP_ANY_HIGH);
  }
  if (seconds > 0) {
    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  }
  esp_deep_sleep_start();
}

#endif
