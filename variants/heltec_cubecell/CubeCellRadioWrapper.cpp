#include "CubeCellRadioWrapper.h"
#include <Arduino.h>
#include "LoRaWan_APP.h"

#define STATE_IDLE       0
#define STATE_RX         1
#define STATE_TX         2
#define STATE_PKT_READY  3
#define STATE_TX_DONE    4

static volatile uint8_t cc_state = STATE_IDLE;

static uint8_t  rx_pkt_buf[255];
static uint8_t  rx_pkt_len = 0;
static int16_t  rx_last_rssi = 0;
static int8_t   rx_last_snr  = 0;

#define NUM_NOISE_FLOOR_SAMPLES  64
#define SAMPLING_THRESHOLD       14

static void ccOnRxDone(uint8_t* payload, uint16_t size,
                       int16_t rssi, int8_t snr)
{
  if (size > sizeof(rx_pkt_buf)) size = sizeof(rx_pkt_buf);
  memcpy(rx_pkt_buf, payload, size);
  rx_pkt_len  = (uint8_t)size;
  rx_last_rssi = rssi;
  rx_last_snr  = snr;
  cc_state = STATE_PKT_READY;
}

static void ccOnTxDone(void)
{
  cc_state = STATE_TX_DONE;
}

static void ccOnRxTimeout(void)
{
  cc_state = STATE_IDLE;
}

static void ccOnRxError(void)
{
  cc_state = STATE_IDLE;
}

static void ccOnTxTimeout(void)
{
  cc_state = STATE_IDLE;
}

static uint8_t bwToCode(float bw_khz)
{
  if (bw_khz <=   7.8f) return 0;
  if (bw_khz <=  10.4f) return 8;
  if (bw_khz <=  15.6f) return 1;
  if (bw_khz <=  20.8f) return 9;
  if (bw_khz <=  31.25f) return 2;
  if (bw_khz <=  41.7f) return 10;
  if (bw_khz <=  62.5f) return 3;
  if (bw_khz <= 125.0f) return 4;
  if (bw_khz <= 250.0f) return 5;
  return 6;
}

static const float snr_threshold[] = {
  -7.5f,
  -10.0f,
  -12.5f,
  -15.0f,
  -17.5f,
  -20.0f
};

CubeCellRadioWrapper::CubeCellRadioWrapper(mesh::MainBoard& board)
  : _board(&board)
{
  n_recv = n_sent = n_recv_errors = 0;
  _noise_floor = 0;
  _threshold   = 0;
  _num_floor_samples = 0;
  _floor_sample_sum  = 0;
  _sf = 11;
  _bw_khz = 250.0f;
  _cr = 5;
  _tx_power = 22;
}

bool CubeCellRadioWrapper::initRadio(float freq_mhz, float bw_khz,
                                     uint8_t sf, uint8_t cr,
                                     int8_t tx_power)
{
  _sf       = sf;
  _bw_khz   = bw_khz;
  _cr       = cr;
  _tx_power = tx_power;

  static RadioEvents_t events;
  events.RxDone    = ccOnRxDone;
  events.TxDone    = ccOnTxDone;
  events.RxTimeout = ccOnRxTimeout;
  events.RxError   = ccOnRxError;
  events.TxTimeout = ccOnTxTimeout;

  ::Radio.Init(&events);

  uint32_t freq_hz = (uint32_t)(freq_mhz * 1e6f);
  ::Radio.SetChannel(freq_hz);

  uint8_t bw_code = bwToCode(bw_khz);

  ::Radio.SetRxConfig(
    MODEM_LORA, bw_code, sf, cr, 0, 16, 0,
    false, 0, true, false, 0, false, true
  );

  ::Radio.SetTxConfig(
    MODEM_LORA, tx_power, 0, bw_code, sf, cr, 16,
    false, true, false, 0, false, 0
  );

  ::Radio.SetSyncWord(0x12);

  return true;
}

void CubeCellRadioWrapper::begin()
{
  cc_state = STATE_IDLE;
  _noise_floor = 0;
  _threshold   = 0;
  _num_floor_samples = 0;
  _floor_sample_sum  = 0;
  n_recv = n_sent = n_recv_errors = 0;

  ::Radio.RxBoosted(0);
  cc_state = STATE_RX;
}

int CubeCellRadioWrapper::recvRaw(uint8_t* bytes, int sz)
{
  int len = 0;

  if (cc_state == STATE_PKT_READY) {
    len = rx_pkt_len;
    if (len > sz) len = sz;
    if (len > 0) {
      memcpy(bytes, rx_pkt_buf, len);
      n_recv++;
    }
    cc_state = STATE_IDLE;
  }

  if (cc_state == STATE_IDLE) {
    ::Radio.RxBoosted(0);
    cc_state = STATE_RX;
  }

  return len;
}

bool CubeCellRadioWrapper::startSendRaw(const uint8_t* bytes, int len)
{
  _board->onBeforeTransmit();
  ::Radio.Send((uint8_t*)bytes, (uint8_t)len);
  cc_state = STATE_TX;
  return true;
}

bool CubeCellRadioWrapper::isSendComplete()
{
  if (cc_state == STATE_TX_DONE) {
    cc_state = STATE_IDLE;
    n_sent++;
    return true;
  }
  return false;
}

void CubeCellRadioWrapper::onSendFinished()
{
  _board->onAfterTransmit();
  cc_state = STATE_IDLE;
}

void CubeCellRadioWrapper::loop()
{
  ::Radio.IrqProcess();

  if (cc_state == STATE_RX &&
      _num_floor_samples < NUM_NOISE_FLOOR_SAMPLES) {
    int rssi = (int)getCurrentRSSI();
    if (rssi < _noise_floor + SAMPLING_THRESHOLD) {
      _num_floor_samples++;
      _floor_sample_sum += rssi;
    }
  } else if (_num_floor_samples >= NUM_NOISE_FLOOR_SAMPLES &&
             _floor_sample_sum != 0) {
    _noise_floor = (int16_t)(_floor_sample_sum / NUM_NOISE_FLOOR_SAMPLES);
    if (_noise_floor < -120) _noise_floor = -120;
    _floor_sample_sum = 0;
  }
}

void CubeCellRadioWrapper::triggerNoiseFloorCalibrate(int threshold)
{
  _threshold = (int16_t)threshold;
  if (_num_floor_samples >= NUM_NOISE_FLOOR_SAMPLES) {
    _num_floor_samples = 0;
    _floor_sample_sum  = 0;
  }
}

void CubeCellRadioWrapper::resetAGC()
{
  if (cc_state == STATE_PKT_READY || cc_state == STATE_TX ||
      cc_state == STATE_TX_DONE)
    return;

  cc_state = STATE_IDLE;
}

bool CubeCellRadioWrapper::isInRecvMode() const
{
  return cc_state == STATE_RX || cc_state == STATE_PKT_READY;
}

bool CubeCellRadioWrapper::isReceiving()
{
  return isChannelActive();
}

bool CubeCellRadioWrapper::isChannelActive()
{
  if (_threshold == 0) return false;
  return getCurrentRSSI() > _noise_floor + _threshold;
}

float CubeCellRadioWrapper::getCurrentRSSI()
{
  return (float)::Radio.Rssi(MODEM_LORA);
}

float CubeCellRadioWrapper::getLastRSSI() const
{
  return (float)rx_last_rssi;
}

float CubeCellRadioWrapper::getLastSNR() const
{
  return (float)rx_last_snr;
}

uint32_t CubeCellRadioWrapper::getEstAirtimeFor(int len_bytes)
{
  return ::Radio.TimeOnAir(MODEM_LORA, (uint8_t)len_bytes);
}

float CubeCellRadioWrapper::packetScore(float snr, int packet_len)
{
  int sf = _sf;
  if (sf < 7) return 0.0f;
  if (snr < snr_threshold[sf - 7]) return 0.0f;

  float success = (snr - snr_threshold[sf - 7]) / 10.0f;
  float penalty = 1.0f - (packet_len / 256.0f);
  float score   = success * penalty;
  if (score < 0.0f) score = 0.0f;
  if (score > 1.0f) score = 1.0f;
  return score;
}

void CubeCellRadioWrapper::setParams(float freq_mhz, float bw_khz,
                                     uint8_t sf, uint8_t cr)
{
  _sf     = sf;
  _bw_khz = bw_khz;
  _cr     = cr;

  uint32_t freq_hz = (uint32_t)(freq_mhz * 1e6f);
  uint8_t  bw_code = bwToCode(bw_khz);

  ::Radio.SetChannel(freq_hz);

  ::Radio.SetRxConfig(MODEM_LORA, bw_code, sf, cr, 0, 16, 0,
                      false, 0, true, false, 0, false, true);

  ::Radio.SetTxConfig(MODEM_LORA, _tx_power, 0, bw_code, sf, cr, 16,
                      false, true, false, 0, false, 0);

  ::Radio.RxBoosted(0);
  cc_state = STATE_RX;
}

void CubeCellRadioWrapper::setTxPower(int8_t dbm)
{
  _tx_power = dbm;
  uint8_t bw_code = bwToCode(_bw_khz);

  ::Radio.SetTxConfig(MODEM_LORA, dbm, 0, bw_code, _sf, _cr, 16,
                      false, true, false, 0, false, 0);
}
