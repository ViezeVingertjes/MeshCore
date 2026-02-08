#include <Arduino.h>
#include "target.h"
#include "innerWdt.h"

CubeCellBoard board;
CubeCellRadioWrapper radio_driver(board);
VolatileRTCClock rtc_clock;
SensorManager sensors;

bool radio_init()
{
  innerWdtEnable(true);

  return radio_driver.initRadio(
    LORA_FREQ,
    LORA_BW,
    LORA_SF,
    LORA_CR,
    LORA_TX_POWER
  );
}

uint32_t radio_get_rng_seed()
{
  return analogRead(ADC) ^ (millis() << 16);
}

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr)
{
  radio_driver.setParams(freq, bw, sf, cr);
}

void radio_set_tx_power(uint8_t dbm)
{
  radio_driver.setTxPower((int8_t)dbm);
}

mesh::LocalIdentity radio_new_identity()
{
  StdRNG rng;
  rng.begin(radio_get_rng_seed());
  return mesh::LocalIdentity(&rng);
}
