#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <string>

#include "protocol.h"

namespace esphome {
namespace multical21 {

// Longest wM-Bus mode C frame on air: format A with L = 255 (17 block CRCs).
static const size_t MAX_RAW_FRAME = 1 + 255 + 2 * 17;

struct RadioFrame {
  FrameFormat format;
  int8_t rssi;
  uint16_t length;
  uint8_t data[MAX_RAW_FRAME];  // L-field first, block CRCs still included
};

class Multical21Component : public Component,
                            public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                                  spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_4MHZ> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_meter_id(uint32_t meter_id) { this->decoder_.set_meter_id(meter_id); }
  void set_key(const std::string &hex_key);
  void set_frequency(float frequency) { this->frequency_ = frequency; }

  void set_total_volume_sensor(sensor::Sensor *s) { this->total_volume_sensor_ = s; }
  void set_target_volume_sensor(sensor::Sensor *s) { this->target_volume_sensor_ = s; }
  void set_flow_temperature_sensor(sensor::Sensor *s) { this->flow_temperature_sensor_ = s; }
  void set_ambient_temperature_sensor(sensor::Sensor *s) { this->ambient_temperature_sensor_ = s; }
  void set_rssi_sensor(sensor::Sensor *s) { this->rssi_sensor_ = s; }
  void set_status_text_sensor(text_sensor::TextSensor *s) { this->status_text_sensor_ = s; }
  void set_alarm_history_text_sensor(text_sensor::TextSensor *s) { this->alarm_history_text_sensor_ = s; }
  void set_leak_binary_sensor(binary_sensor::BinarySensor *s) { this->leak_binary_sensor_ = s; }
  void set_burst_binary_sensor(binary_sensor::BinarySensor *s) { this->burst_binary_sensor_ = s; }
  void set_dry_binary_sensor(binary_sensor::BinarySensor *s) { this->dry_binary_sensor_ = s; }
  void set_reverse_binary_sensor(binary_sensor::BinarySensor *s) { this->reverse_binary_sensor_ = s; }

 protected:
  // CC1101 access (after setup() only used from the radio task)
  void strobe_(uint8_t command);
  void write_register_(uint8_t address, uint8_t value);
  uint8_t read_register_(uint8_t address);
  uint8_t read_status_(uint8_t address);
  uint8_t read_rx_bytes_();
  void read_fifo_(uint8_t *buffer, size_t length);
  bool init_radio_();
  void start_receiving_();
  int8_t read_rssi_();

  static void radio_task_(void *arg);
  void radio_loop_();
  bool receive_frame_(RadioFrame &frame);

  void handle_frame_(const RadioFrame &frame);
  void publish_(const Reading &reading, int8_t rssi);

  Decoder decoder_;
  bool key_valid_{false};
  float frequency_{868.95e6f};
  uint8_t chip_version_{0};

  QueueHandle_t queue_{nullptr};
  TaskHandle_t task_{nullptr};
  RadioFrame task_frame_{};  // used only by the radio task
  RadioFrame loop_frame_{};  // used only by loop()

  // Written by the radio task, read by loop() for diagnostics only.
  struct {
    volatile uint32_t polls{0};
    volatile uint32_t syncs{0};
    volatile uint32_t received{0};
    volatile uint32_t bad_marker{0};
    volatile uint32_t bad_length{0};
    volatile uint32_t timeout{0};
    volatile uint32_t overflow{0};
    volatile uint8_t last_marker{0};
  } stats_;
  uint32_t last_stats_log_ms_{0};

  uint32_t frames_total_{0};
  uint32_t frames_crc_error_{0};
  uint32_t frames_other_meters_{0};
  uint32_t telegrams_ok_{0};
  uint32_t last_telegram_ms_{0};

  sensor::Sensor *total_volume_sensor_{nullptr};
  sensor::Sensor *target_volume_sensor_{nullptr};
  sensor::Sensor *flow_temperature_sensor_{nullptr};
  sensor::Sensor *ambient_temperature_sensor_{nullptr};
  sensor::Sensor *rssi_sensor_{nullptr};
  text_sensor::TextSensor *status_text_sensor_{nullptr};
  text_sensor::TextSensor *alarm_history_text_sensor_{nullptr};
  binary_sensor::BinarySensor *leak_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *burst_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *dry_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *reverse_binary_sensor_{nullptr};
};

}  // namespace multical21
}  // namespace esphome
