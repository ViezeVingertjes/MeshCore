#include "KISSModem.h"

KISSModem::KISSModem(mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, 
                     mesh::RTCClock& rtc, mesh::MeshTables& tables, Stream& serial)
  : mesh::Mesh(radio, ms, rng, rtc, *new StaticPoolPacketManager(32), tables),
    _fs(nullptr), _transmittingFromSerial(false)
{
  memset(&_stats, 0, sizeof(_stats));
  _kiss = new kiss::KISSProtocol(serial, *this);
}

KISSModem::~KISSModem() {
  delete _kiss;
}

void KISSModem::begin(FILESYSTEM* fs) {
  _fs = fs;
  mesh::Mesh::begin();
  
#if defined(NRF52_PLATFORM)
  IdentityStore store(*fs, "");
#elif defined(RP2040_PLATFORM)
  IdentityStore store(*fs, "/identity");
  store.begin();
#else
  IdentityStore store(*fs, "/identity");
#endif
  
  if (!store.load("_main", self_id)) {
    self_id = mesh::LocalIdentity(getRNG());
    int count = 0;
    while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {
      self_id = mesh::LocalIdentity(getRNG());
      count++;
    }
    store.save("_main", self_id);
  }
}

void KISSModem::loop() {
  _kiss->process();
  mesh::Mesh::loop();
  
  _stats.total_uptime_secs = _ms->getMillis() / 1000;
  _stats.total_airtime_secs = getTotalAirTime() / 1000;
  _stats.packets_received = radio_driver.getPacketsRecv();
  _stats.packets_sent = radio_driver.getPacketsSent();
  _stats.last_rssi = (int16_t)_radio->getLastRSSI();
  _stats.last_snr = (int16_t)(_radio->getLastSNR() * 4);
}

void KISSModem::onKISSFrame(uint8_t command, const uint8_t* data, size_t len) {
  _stats.frames_from_serial++;
  
  switch (command) {
    case kiss::CMD_DATA:
      handleDataFrame(data, len);
      break;
      
    case kiss::CMD_GET_IDENTITY:
      sendIdentityResponse();
      break;
      
    case kiss::CMD_SIGN_DATA:
      handleSignRequest(data, len);
      break;
      
    default:
      MESH_DEBUG_PRINTLN("Unknown KISS command: %02X", (uint32_t)command);
      break;
  }
}

void KISSModem::sendIdentityResponse() {
  uint8_t response[PUB_KEY_SIZE];
  memcpy(response, self_id.pub_key, PUB_KEY_SIZE);
  _kiss->sendFrame(kiss::CMD_RESP_IDENTITY, response, PUB_KEY_SIZE);
  _stats.frames_to_serial++;
}


void KISSModem::handleSignRequest(const uint8_t* data, size_t len) {
  if (len == 0 || len > 1024) return;
  
  uint8_t signature[SIGNATURE_SIZE];
  self_id.sign(signature, data, len);
  
  _kiss->sendFrame(kiss::CMD_RESP_SIGNATURE, signature, SIGNATURE_SIZE);
  _stats.frames_to_serial++;
}

void KISSModem::handleDataFrame(const uint8_t* data, size_t len) {
  if (!isValidPacketData(data, len)) return;
  
  mesh::Packet* packet = _mgr->allocNew();
  if (!packet) return;
  
  if (!packet->readFrom(data, len)) {
    _mgr->free(packet);
    return;
  }
  
  transmitPacket(packet);
}

bool KISSModem::isValidPacketData(const uint8_t* data, size_t len) const {
  if (len < 2 || len > MAX_TRANS_UNIT) return false;
  
  uint8_t header = data[0];
  uint8_t route_type = header & 0x03;
  uint8_t payload_type = (header >> 2) & 0x0F;
  
  if (route_type > 0x03 || payload_type > 0x0F) return false;
  
  return true;
}

void KISSModem::transmitPacket(mesh::Packet* packet) {
  _transmittingFromSerial = true;
  
  if (packet->isRouteFlood()) {
    sendFlood(packet);
  } else if (packet->isRouteDirect()) {
    if (packet->path_len > 0) {
      sendDirect(packet, packet->path, packet->path_len);
    } else {
      sendFlood(packet);
    }
  } else {
    sendFlood(packet);
  }
  
  _transmittingFromSerial = false;
}

mesh::DispatcherAction KISSModem::onRecvPacket(mesh::Packet* pkt) {
  uint8_t rawPacket[MAX_TRANS_UNIT];
  uint8_t len = pkt->writeTo(rawPacket);
  
  _kiss->sendFrame(kiss::CMD_DATA, rawPacket, len);
  _stats.frames_to_serial++;
  
  return mesh::Mesh::onRecvPacket(pkt);
}

