#pragma once

#include <MeshCore.h>
#include "CubeCellBoard.h"
#include "CubeCellRadioWrapper.h"
#include <helpers/ArduinoHelpers.h>
#include <helpers/SensorManager.h>

extern CubeCellBoard board;
extern CubeCellRadioWrapper radio_driver;
extern VolatileRTCClock rtc_clock;
extern SensorManager sensors;

bool radio_init();
uint32_t radio_get_rng_seed();
void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr);
void radio_set_tx_power(uint8_t dbm);
mesh::LocalIdentity radio_new_identity();
