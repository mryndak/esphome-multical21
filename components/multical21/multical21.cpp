#include "multical21.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cmath>
#include <vector>

namespace esphome {
namespace multical21 {

static const char *const TAG = "multical21";

// ---------------------------------------------------------------------------
// CC1101 registers, strobes and settings for wM-Bus mode C1 reception
// (100 kchip/s NRZ 2-FSK, sync word 0x543D, 26 MHz crystal).
// ---------------------------------------------------------------------------

namespace cc1101 {
static const uint8_t IOCFG2 = 0x00, IOCFG1 = 0x01, IOCFG0 = 0x02, FIFOTHR = 0x03, SYNC1 = 0x04, SYNC0 = 0x05,
                     PKTLEN = 0x06, PKTCTRL1 = 0x07, PKTCTRL0 = 0x08, ADDR = 0x09, CHANNR = 0x0A, FSCTRL1 = 0x0B,
                     FSCTRL0 = 0x0C, FREQ2 = 0x0D, FREQ1 = 0x0E, FREQ0 = 0x0F, MDMCFG4 = 0x10, MDMCFG3 = 0x11,
                     MDMCFG2 = 0x12, MDMCFG1 = 0x13, MDMCFG0 = 0x14, DEVIATN = 0x15, MCSM2 = 0x16, MCSM1 = 0x17,
                     MCSM0 = 0x18, FOCCFG = 0x19, BSCFG = 0x1A, AGCCTRL2 = 0x1B, AGCCTRL1 = 0x1C, AGCCTRL0 = 0x1D,
                     FREND1 = 0x21, FREND0 = 0x22, FSCAL3 = 0x23, FSCAL2 = 0x24, FSCAL1 = 0x25, FSCAL0 = 0x26,
                     FSTEST = 0x29, TEST2 = 0x2C, TEST1 = 0x2D, TEST0 = 0x2E;
// Status registers (read with the burst bit set)
static const uint8_t PARTNUM = 0x30, VERSION = 0x31, RSSI = 0x34, MARCSTATE = 0x35, RXBYTES = 0x3B;
// Command strobes
static const uint8_t SRES = 0x30, SRX = 0x34, SIDLE = 0x36, SFRX = 0x3A;
static const uint8_t FIFO = 0x3F;
static const uint8_t READ_SINGLE = 0x80, READ_BURST = 0xC0;
static const uint8_t MARCSTATE_IDLE = 0x01;
static const uint8_t RXBYTES_OVERFLOW = 0x80;
static const float CRYSTAL_HZ = 26e6f;

struct Setting {
  uint8_t address;
  uint8_t value;
};

static const Setting MODE_C1[] = {
    {IOCFG2, 0x2E},    // GDO2 high impedance (not used)
    {IOCFG1, 0x2E},    // GDO1 high impedance (shared with MISO)
    {IOCFG0, 0x06},    // GDO0 asserts on sync word (not required, handy for a scope/LED)
    {FIFOTHR, 0x07},   // RX FIFO threshold 32 bytes
    {SYNC1, 0x54},     // wM-Bus mode C sync word 0x543D
    {SYNC0, 0x3D},
    {PKTLEN, 0xFF},
    {PKTCTRL1, 0x00},  // no address check, no status bytes appended
    {PKTCTRL0, 0x02},  // infinite packet length, no hardware CRC, no whitening
    {ADDR, 0x00},
    {CHANNR, 0x00},
    {FSCTRL1, 0x08},   // IF 203 kHz
    {FSCTRL0, 0x00},
    {MDMCFG4, 0x5C},   // RX filter bandwidth 325 kHz, data rate exponent 12
    {MDMCFG3, 0x04},   // data rate ~103 kBaud
    {MDMCFG2, 0x02},   // 2-FSK, no Manchester, 16/16 sync word bits
    {MDMCFG1, 0x22},   // 4 preamble bytes (TX only), channel spacing exponent 2
    {MDMCFG0, 0xF8},
    {DEVIATN, 0x44},   // ~38 kHz deviation
    {MCSM2, 0x07},
    {MCSM1, 0x00},     // stay in IDLE after a packet (we restart RX ourselves)
    {MCSM0, 0x18},     // calibrate when going from IDLE to RX
    {FOCCFG, 0x2E},
    {BSCFG, 0xBF},
    {AGCCTRL2, 0x43},
    {AGCCTRL1, 0x09},
    {AGCCTRL0, 0xB5},
    {FREND1, 0xB6},
    {FREND0, 0x10},
    {FSCAL3, 0xEA},
    {FSCAL2, 0x2A},
    {FSCAL1, 0x00},
    {FSCAL0, 0x1F},
    {FSTEST, 0x59},
    {TEST2, 0x81},     // recommended for RX filter bandwidths <= 325 kHz
    {TEST1, 0x35},
    {TEST0, 0x09},
};

// wM-Bus mode C: the 0x543D sync word is followed by a second sync word,
// 0x54CD for frame format A or 0x543D for frame format B.
static const uint8_t SYNC_PREFIX = 0x54;
static const uint8_t FORMAT_A_MARKER = 0xCD;
static const uint8_t FORMAT_B_MARKER = 0x3D;
}  // namespace cc1101

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void Multical21Component::set_key(const std::string &hex_key) {
  uint8_t key[16];
  this->key_valid_ = false;
  if (hex_key.size() != 32)
    return;
  for (size_t i = 0; i < 16; i++) {
    char byte_text[3] = {hex_key[2 * i], hex_key[2 * i + 1], 0};
    char *end = nullptr;
    key[i] = static_cast<uint8_t>(strtoul(byte_text, &end, 16));
    if (end != byte_text + 2)
      return;
  }
  this->decoder_.set_key(key);
  this->key_valid_ = true;
}

// ---------------------------------------------------------------------------
// CC1101 SPI access
// ---------------------------------------------------------------------------

void Multical21Component::strobe_(uint8_t command) {
  this->enable();
  this->write_byte(command);
  this->disable();
}

void Multical21Component::write_register_(uint8_t address, uint8_t value) {
  this->enable();
  this->write_byte(address);
  this->write_byte(value);
  this->disable();
}

uint8_t Multical21Component::read_register_(uint8_t address) {
  this->enable();
  this->write_byte(address | cc1101::READ_SINGLE);
  const uint8_t value = this->read_byte();
  this->disable();
  return value;
}

uint8_t Multical21Component::read_status_(uint8_t address) {
  this->enable();
  this->write_byte(address | cc1101::READ_BURST);
  const uint8_t value = this->read_byte();
  this->disable();
  return value;
}

uint8_t Multical21Component::read_rx_bytes_() {
  // CC1101 errata: status registers may be read while changing, read until stable.
  uint8_t previous = this->read_status_(cc1101::RXBYTES);
  for (int i = 0; i < 4; i++) {
    const uint8_t current = this->read_status_(cc1101::RXBYTES);
    if (current == previous)
      return current;
    previous = current;
  }
  return previous;
}

void Multical21Component::read_fifo_(uint8_t *buffer, size_t length) {
  this->enable();
  this->write_byte(cc1101::FIFO | cc1101::READ_BURST);
  this->read_array(buffer, length);
  this->disable();
}

int8_t Multical21Component::read_rssi_() {
  const int raw = this->read_status_(cc1101::RSSI);
  const int dbm = (raw >= 128 ? raw - 256 : raw) / 2 - 74;
  return static_cast<int8_t>(dbm < -128 ? -128 : dbm);
}

bool Multical21Component::init_radio_() {
  this->strobe_(cc1101::SRES);
  delay(2);

  const uint8_t partnum = this->read_status_(cc1101::PARTNUM);
  this->chip_version_ = this->read_status_(cc1101::VERSION);
  if (this->chip_version_ == 0x00 || this->chip_version_ == 0xFF) {
    ESP_LOGE(TAG, "CC1101 not responding (PARTNUM 0x%02X, VERSION 0x%02X). Check wiring and cs_pin.", partnum,
             this->chip_version_);
    return false;
  }

  for (const auto &setting : cc1101::MODE_C1)
    this->write_register_(setting.address, setting.value);

  const uint32_t freq = static_cast<uint32_t>(lroundf(this->frequency_ / cc1101::CRYSTAL_HZ * 65536.0f));
  this->write_register_(cc1101::FREQ2, (freq >> 16) & 0xFF);
  this->write_register_(cc1101::FREQ1, (freq >> 8) & 0xFF);
  this->write_register_(cc1101::FREQ0, freq & 0xFF);

  if (this->read_register_(cc1101::SYNC1) != 0x54 || this->read_register_(cc1101::MDMCFG4) != 0x5C) {
    ESP_LOGE(TAG, "CC1101 register read-back failed. Check SPI wiring.");
    return false;
  }
  return true;
}

void Multical21Component::start_receiving_() {
  this->strobe_(cc1101::SIDLE);
  for (int i = 0; i < 100 && this->read_status_(cc1101::MARCSTATE) != cc1101::MARCSTATE_IDLE; i++)
    delayMicroseconds(50);
  this->strobe_(cc1101::SFRX);
  this->strobe_(cc1101::SRX);
}

// ---------------------------------------------------------------------------
// Radio task: pulls frames out of the CC1101 FIFO and hands them to loop()
// ---------------------------------------------------------------------------

void Multical21Component::radio_task_(void *arg) { static_cast<Multical21Component *>(arg)->radio_loop_(); }

void Multical21Component::radio_loop_() {
  const TickType_t poll_delay = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;
  uint32_t last_activity = millis();
  this->start_receiving_();
  while (true) {
    const uint8_t available = this->read_rx_bytes_();
    this->stats_.polls++;
    if (available & cc1101::RXBYTES_OVERFLOW) {
      this->stats_.overflow++;
      this->start_receiving_();
      continue;
    }
    // Wait for the second sync word + L-field (3 bytes), plus one byte left in the FIFO (errata).
    if (available < 4) {
      // Re-arm (and re-calibrate) the receiver now and then if nothing is heard.
      if (millis() - last_activity > 60000) {
        this->start_receiving_();
        last_activity = millis();
      }
      vTaskDelay(poll_delay);
      continue;
    }

    last_activity = millis();
    this->stats_.syncs++;
    if (this->receive_frame_(this->task_frame_)) {
      this->stats_.received++;
      xQueueSend(this->queue_, &this->task_frame_, 0);  // drop the frame if loop() is behind
    }
    this->start_receiving_();
  }
}

bool Multical21Component::receive_frame_(RadioFrame &frame) {
  const TickType_t poll_delay = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;
  uint8_t header[3];  // second sync word (0x54CD = format A, 0x543D = format B), L-field
  this->read_fifo_(header, 3);
  frame.rssi = this->read_rssi_();
  this->stats_.last_marker = header[1];

  if (header[0] == cc1101::SYNC_PREFIX && header[1] == cc1101::FORMAT_A_MARKER) {
    frame.format = FrameFormat::A;
  } else if (header[0] == cc1101::SYNC_PREFIX && header[1] == cc1101::FORMAT_B_MARKER) {
    frame.format = FrameFormat::B;
  } else {
    this->stats_.bad_marker++;
    return false;  // not a mode C frame (noise or other protocol)
  }

  const size_t total = frame_length_on_air(frame.format, header[2]);
  if (total == 0 || total > MAX_RAW_FRAME) {
    this->stats_.bad_length++;
    return false;
  }

  frame.data[0] = header[2];
  size_t received = 1;
  uint32_t last_progress = millis();
  while (received < total) {
    const uint8_t available = this->read_rx_bytes_();
    if (available & cc1101::RXBYTES_OVERFLOW) {
      this->stats_.overflow++;
      return false;
    }
    const size_t remaining = total - received;
    size_t count = available;
    // CC1101 errata: never empty the FIFO while the packet is still arriving.
    if (count < remaining && count > 0)
      count--;
    if (count > remaining)
      count = remaining;
    if (count > 0) {
      this->read_fifo_(frame.data + received, count);
      received += count;
      last_progress = millis();
    } else {
      if (millis() - last_progress > 20) {
        this->stats_.timeout++;
        return false;  // signal lost
      }
      vTaskDelay(poll_delay);
    }
  }
  frame.length = static_cast<uint16_t>(total);
  return true;
}

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

void Multical21Component::setup() {
  if (!this->key_valid_) {
    ESP_LOGE(TAG, "Invalid key, expected 32 hex characters");
    this->mark_failed();
    return;
  }
  this->spi_setup();
  if (!this->init_radio_()) {
    this->mark_failed();
    return;
  }

  this->queue_ = xQueueCreate(4, sizeof(RadioFrame));
  if (this->queue_ == nullptr ||
      xTaskCreate(Multical21Component::radio_task_, "multical21", 4096, this, 5, &this->task_) != pdPASS) {
    ESP_LOGE(TAG, "Failed to start the radio task");
    this->mark_failed();
    return;
  }
}

void Multical21Component::loop() {
  if (this->queue_ == nullptr)
    return;
  while (xQueueReceive(this->queue_, &this->loop_frame_, 0) == pdTRUE)
    this->handle_frame_(this->loop_frame_);

  const uint32_t now = millis();
  if (now - this->last_stats_log_ms_ >= 600000) {  // every 10 minutes
    this->last_stats_log_ms_ = now;
    ESP_LOGD(TAG,
             "Radio: %" PRIu32 " polls, %" PRIu32 " syncs, %" PRIu32 " frames (%" PRIu32 " bad marker, last 0x%02X; %" PRIu32
             " bad length, %" PRIu32 " timeouts, %" PRIu32 " overflows); %" PRIu32 " CRC errors, %" PRIu32
             " other meters, %" PRIu32 " telegrams",
             this->stats_.polls, this->stats_.syncs, this->stats_.received, this->stats_.bad_marker,
             this->stats_.last_marker, this->stats_.bad_length, this->stats_.timeout, this->stats_.overflow,
             this->frames_crc_error_, this->frames_other_meters_, this->telegrams_ok_);
  }
}

void Multical21Component::handle_frame_(const RadioFrame &frame) {
  this->frames_total_++;
  std::vector<uint8_t> link_frame;
  if (!strip_block_crcs(frame.format, frame.data, frame.length, link_frame)) {
    this->frames_crc_error_++;
    ESP_LOGV(TAG, "Frame with CRC error (RSSI %d dBm)", frame.rssi);
    return;
  }

  uint32_t meter_id = 0;
  if (!Decoder::peek_meter_id(link_frame.data(), link_frame.size(), meter_id))
    return;
  if (meter_id != this->decoder_.get_meter_id()) {
    this->frames_other_meters_++;
    ESP_LOGD(TAG, "Frame from another meter %08" PRIX32 " (RSSI %d dBm)", meter_id, frame.rssi);
    return;
  }

  Reading reading;
  const DecodeResult result = this->decoder_.decode(link_frame.data(), link_frame.size(), reading);
  if (result != DecodeResult::OK) {
    if (result == DecodeResult::PAYLOAD_CRC_MISMATCH) {
      ESP_LOGW(TAG, "Telegram from %08" PRIX32 " could not be decrypted - check the key", meter_id);
    } else {
      ESP_LOGD(TAG, "Telegram from %08" PRIX32 " skipped: %s", meter_id, decode_result_to_string(result));
    }
    return;
  }

  this->telegrams_ok_++;
  this->last_telegram_ms_ = millis();
  ESP_LOGD(TAG, "Telegram (%s): %.3f m3, target %.3f m3, %d/%d C, %s, RSSI %d dBm",
           reading.compact ? "compact" : "full", reading.total_l / 1000.0f, reading.target_l / 1000.0f,
           reading.flow_temperature_c, reading.ambient_temperature_c, info_codes_status(reading.info_codes).c_str(),
           frame.rssi);
  this->publish_(reading, frame.rssi);
}

void Multical21Component::publish_(const Reading &reading, int8_t rssi) {
  if (this->total_volume_sensor_ != nullptr && reading.has_total)
    this->total_volume_sensor_->publish_state(reading.total_l / 1000.0f);
  if (this->target_volume_sensor_ != nullptr && reading.has_target)
    this->target_volume_sensor_->publish_state(reading.target_l / 1000.0f);
  if (this->flow_temperature_sensor_ != nullptr && reading.has_flow_temperature)
    this->flow_temperature_sensor_->publish_state(reading.flow_temperature_c);
  if (this->ambient_temperature_sensor_ != nullptr && reading.has_ambient_temperature)
    this->ambient_temperature_sensor_->publish_state(reading.ambient_temperature_c);
  if (this->rssi_sensor_ != nullptr)
    this->rssi_sensor_->publish_state(rssi);

  if (!reading.has_info_codes)
    return;
  const uint16_t info = reading.info_codes;
  if (this->status_text_sensor_ != nullptr)
    this->status_text_sensor_->publish_state(info_codes_status(info));
  if (this->alarm_history_text_sensor_ != nullptr) {
    const std::string history = info_codes_history(info);
    this->alarm_history_text_sensor_->publish_state(history.empty() ? "none" : history);
  }
  if (this->leak_binary_sensor_ != nullptr)
    this->leak_binary_sensor_->publish_state(info & INFO_LEAK);
  if (this->burst_binary_sensor_ != nullptr)
    this->burst_binary_sensor_->publish_state(info & INFO_BURST);
  if (this->dry_binary_sensor_ != nullptr)
    this->dry_binary_sensor_->publish_state(info & INFO_DRY);
  if (this->reverse_binary_sensor_ != nullptr)
    this->reverse_binary_sensor_->publish_state(info & INFO_REVERSE);
}

void Multical21Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Kamstrup MULTICAL 21:");
  ESP_LOGCONFIG(TAG, "  Meter ID: %08" PRIX32, this->decoder_.get_meter_id());
  ESP_LOGCONFIG(TAG, "  Frequency: %.3f MHz", this->frequency_ / 1e6f);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Setup failed - see the errors above");
    return;
  }
  ESP_LOGCONFIG(TAG, "  CC1101 version: 0x%02X", this->chip_version_);
  LOG_SENSOR("  ", "Total volume", this->total_volume_sensor_);
  LOG_SENSOR("  ", "Target volume", this->target_volume_sensor_);
  LOG_SENSOR("  ", "Flow temperature", this->flow_temperature_sensor_);
  LOG_SENSOR("  ", "Ambient temperature", this->ambient_temperature_sensor_);
  LOG_SENSOR("  ", "RSSI", this->rssi_sensor_);
  LOG_TEXT_SENSOR("  ", "Status", this->status_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Alarm history", this->alarm_history_text_sensor_);
  LOG_BINARY_SENSOR("  ", "Leak", this->leak_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "Burst", this->burst_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "Dry", this->dry_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "Reverse flow", this->reverse_binary_sensor_);
}

}  // namespace multical21
}  // namespace esphome
