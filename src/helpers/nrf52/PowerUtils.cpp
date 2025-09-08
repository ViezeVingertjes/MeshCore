#include "PowerUtils.h"

#include <bluefruit.h>
#include <nrf_power.h>

namespace mesh {
namespace nrf52 {

void enableDcDc() {
  // Query high-voltage (VDDH) mode via nrfx HAL accessor.
  const bool isHighVoltageMode = (nrf_power_mainregstatus_get(NRF_POWER) == NRF_POWER_MAINREGSTATUS_HIGH);

  uint8_t sd_enabled = 0;
  (void) sd_softdevice_is_enabled(&sd_enabled);
  if (sd_enabled) {
    (void) sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    if (isHighVoltageMode) {
      (void) sd_power_dcdc0_mode_set(NRF_POWER_DCDC_ENABLE);
    }
  } else {
    NRF_POWER->DCDCEN = 1;
    if (isHighVoltageMode) {
      NRF_POWER->DCDCEN0 = 1;
    }
  }
}

} // namespace nrf52
} // namespace mesh


