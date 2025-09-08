#pragma once

namespace mesh {
namespace nrf52 {

// Enable nRF52 DC/DC regulator (REG1). If SoftDevice is enabled, uses sd_* APIs.
// If MESH_ENABLE_REG0_DCDC is defined and non-zero on nRF52840, enable REG0 as well.
void enableDcDc();

} // namespace nrf52
} // namespace mesh


