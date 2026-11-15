#pragma once

// Platform-independent wM-Bus / Kamstrup MULTICAL 21 protocol code.
// No ESPHome or ESP-IDF dependencies, so it can be unit-tested on a PC.

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace esphome {
namespace multical21 {

// CRC-16 used by EN 13757 (polynomial 0x3D65, init 0x0000, final XOR 0xFFFF).
uint16_t crc16_en13757(const uint8_t *data, size_t len);

// Minimal AES-128 (encryption direction only, which is all CTR mode needs).
class Aes128 {
 public:
  explicit Aes128(const uint8_t key[16]);
  void encrypt_block(const uint8_t in[16], uint8_t out[16]) const;

 private:
  uint8_t round_keys_[176];
};

// AES-128-CTR. The counter is the full 16-byte block, incremented big-endian.
void aes128_ctr(const uint8_t key[16], const uint8_t iv[16], const uint8_t *in, uint8_t *out, size_t len);

// wM-Bus mode C frame formats. On air, format A is preceded by sync byte 0xCD,
// format B by 0x3D (after the common 0x543D sync word).
enum class FrameFormat : uint8_t { A, B };

// Number of bytes on air for a frame (L-field included, block CRCs included).
// Returns 0 if the L-field is not valid for the format.
size_t frame_length_on_air(FrameFormat format, uint8_t l_field);

// Verifies and removes the block CRCs. `raw` starts with the L-field.
// On success `out` holds the link-layer frame without CRCs (L-field first).
bool strip_block_crcs(FrameFormat format, const uint8_t *raw, size_t raw_len, std::vector<uint8_t> &out);

// Info codes (Kamstrup "error flags"), current state bits.
static const uint16_t INFO_DRY = 0x0001;
static const uint16_t INFO_REVERSE = 0x0002;
static const uint16_t INFO_LEAK = 0x0004;
static const uint16_t INFO_BURST = 0x0008;

struct Reading {
  uint32_t meter_id{0};
  bool compact{false};
  bool has_info_codes{false};
  uint16_t info_codes{0};
  bool has_total{false};
  uint32_t total_l{0};  // litres
  bool has_target{false};
  uint32_t target_l{0};  // litres, value at the last target date (end of month)
  bool has_flow_temperature{false};
  int8_t flow_temperature_c{0};
  bool has_ambient_temperature{false};
  int8_t ambient_temperature_c{0};
};

enum class DecodeResult : uint8_t {
  OK,
  TOO_SHORT,
  OTHER_METER,
  UNSUPPORTED_CI,
  UNSUPPORTED_ENCRYPTION,
  PAYLOAD_CRC_MISMATCH,  // usually a wrong key
  UNKNOWN_COMPACT_FORMAT,
  FULL_FRAME_CRC_MISMATCH,
  MALFORMED,
};

const char *decode_result_to_string(DecodeResult result);

// "OK" or a comma separated list of active flags, e.g. "LEAK, BURST".
std::string info_codes_status(uint16_t info_codes);
// Alarm history kept by the meter, e.g. "leak 1-8 hours". Empty if none.
std::string info_codes_history(uint16_t info_codes);

class Decoder {
 public:
  Decoder();
  void set_meter_id(uint32_t meter_id) { this->meter_id_ = meter_id; }
  uint32_t get_meter_id() const { return this->meter_id_; }
  void set_key(const uint8_t key[16]);

  // `frame` is a link-layer frame without block CRCs, L-field first.
  DecodeResult decode(const uint8_t *frame, size_t len, Reading &out);

  // Meter id from a link-layer frame (no decryption). Returns false if too short.
  static bool peek_meter_id(const uint8_t *frame, size_t len, uint32_t &meter_id);

 protected:
  DecodeResult decode_application_(const uint8_t *payload, size_t len, Reading &out);
  bool decode_records_(const uint8_t *records, size_t len, Reading &out, std::vector<uint8_t> *headers);

  uint32_t meter_id_{0};
  uint8_t key_[16]{};
  // Compact frame formats: signature (CRC of the record headers) -> record headers.
  std::map<uint16_t, std::vector<uint8_t>> formats_;
};

}  // namespace multical21
}  // namespace esphome
