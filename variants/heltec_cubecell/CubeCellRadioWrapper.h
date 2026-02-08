#pragma once

#include <Dispatcher.h>
#include <MeshCore.h>

class CubeCellRadioWrapper : public mesh::Radio {
  mesh::MainBoard* _board;

  uint32_t n_recv, n_sent, n_recv_errors;

  int16_t  _noise_floor;
  int16_t  _threshold;
  uint16_t _num_floor_samples;
  int32_t  _floor_sample_sum;

  uint8_t _sf;
  float   _bw_khz;
  uint8_t _cr;
  int8_t  _tx_power;

  bool isChannelActive();

public:
  explicit CubeCellRadioWrapper(mesh::MainBoard& board);

  bool initRadio(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr, int8_t tx_power);
  void setParams(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr);
  void setTxPower(int8_t dbm);

  void begin() override;
  int  recvRaw(uint8_t* bytes, int sz) override;
  uint32_t getEstAirtimeFor(int len_bytes) override;
  float packetScore(float snr, int packet_len) override;
  bool  startSendRaw(const uint8_t* bytes, int len) override;
  bool  isSendComplete() override;
  void  onSendFinished() override;
  void  loop() override;
  int   getNoiseFloor() const override { return _noise_floor; }
  void  triggerNoiseFloorCalibrate(int threshold) override;
  void  resetAGC() override;
  bool  isInRecvMode() const override;
  bool  isReceiving() override;
  float getLastRSSI() const override;
  float getLastSNR() const override;

  float    getCurrentRSSI();
  uint32_t getPacketsRecv() const      { return n_recv; }
  uint32_t getPacketsSent() const      { return n_sent; }
  uint32_t getPacketsRecvErrors() const { return n_recv_errors; }
};
