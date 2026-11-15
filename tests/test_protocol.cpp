// Host unit tests for the protocol layer. Build and run from the repository root:
//   make -C tests

#include <cstdio>
#include <cstring>
#include <vector>

#include "protocol.h"
#include "test_vectors.h"

using namespace esphome::multical21;

static int failures = 0;
static int checks = 0;

#define CHECK(cond) \
  do { \
    checks++; \
    if (!(cond)) { \
      failures++; \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
  } while (0)

#define CHECK_RESULT(actual, expected) \
  do { \
    checks++; \
    DecodeResult a_ = (actual); \
    if (a_ != (expected)) { \
      failures++; \
      printf("FAIL %s:%d: got '%s', expected '%s'\n", __FILE__, __LINE__, decode_result_to_string(a_), \
             decode_result_to_string(expected)); \
    } \
  } while (0)

static Decoder make_decoder() {
  Decoder decoder;
  decoder.set_meter_id(TEST_METER_ID);
  decoder.set_key(TEST_KEY);
  return decoder;
}

static void test_crc() {
  // Record headers of a MULTICAL 21 full frame; the meter sends this as the compact signature 0xA8ED.
  const uint8_t format[] = {0x02, 0xFF, 0x20, 0x04, 0x13, 0x44, 0x13, 0x61, 0x5B, 0x61, 0x67};
  CHECK(crc16_en13757(format, sizeof(format)) == 0xA8ED);
}

static void test_aes() {
  // FIPS-197 appendix C.1
  const uint8_t key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                           0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
  const uint8_t plain[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                             0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
  const uint8_t expected[16] = {0x69, 0xC4, 0xE0, 0xD8, 0x6A, 0x7B, 0x04, 0x30,
                                0xD8, 0xCD, 0xB7, 0x80, 0x70, 0xB4, 0xC5, 0x5A};
  uint8_t out[16];
  Aes128(key).encrypt_block(plain, out);
  CHECK(memcmp(out, expected, 16) == 0);

  // CTR is symmetric and handles partial blocks.
  uint8_t iv[16] = {0};
  iv[15] = 0xFF;  // forces a carry into the next byte
  uint8_t data[40], encrypted[40], decrypted[40];
  for (int i = 0; i < 40; i++)
    data[i] = static_cast<uint8_t>(i * 7);
  aes128_ctr(key, iv, data, encrypted, sizeof(data));
  aes128_ctr(key, iv, encrypted, decrypted, sizeof(data));
  CHECK(memcmp(data, decrypted, sizeof(data)) == 0);
  CHECK(memcmp(data, encrypted, sizeof(data)) != 0);
}

static void test_frame_length() {
  CHECK(frame_length_on_air(FrameFormat::A, 0x2A) == 1 + 0x2A + 2 * 4);  // 10 + 16 + 16 + 1 bytes
  CHECK(frame_length_on_air(FrameFormat::A, 0x23) == 1 + 0x23 + 2 * 3);  // 10 + 16 + 10 bytes
  CHECK(frame_length_on_air(FrameFormat::A, 9) == 1 + 9 + 2);
  CHECK(frame_length_on_air(FrameFormat::A, 8) == 0);
  CHECK(frame_length_on_air(FrameFormat::B, 0x2C) == 1 + 0x2C);
}

static void test_strip_crcs() {
  std::vector<uint8_t> out;
  CHECK(strip_block_crcs(FrameFormat::A, FULL_FRAME_RAW_A, sizeof(FULL_FRAME_RAW_A), out));
  CHECK(out.size() == sizeof(FULL_FRAME));
  CHECK(memcmp(out.data(), FULL_FRAME, sizeof(FULL_FRAME)) == 0);

  // A single flipped bit must be detected.
  std::vector<uint8_t> corrupted(FULL_FRAME_RAW_A, FULL_FRAME_RAW_A + sizeof(FULL_FRAME_RAW_A));
  corrupted[20] ^= 0x01;
  CHECK(!strip_block_crcs(FrameFormat::A, corrupted.data(), corrupted.size(), out));

  // Truncated frame.
  CHECK(!strip_block_crcs(FrameFormat::A, FULL_FRAME_RAW_A, sizeof(FULL_FRAME_RAW_A) - 1, out));

  // Format B: one CRC over the whole (short) frame. L counts the CRC.
  std::vector<uint8_t> frame_b(FULL_FRAME, FULL_FRAME + sizeof(FULL_FRAME));
  frame_b[0] = static_cast<uint8_t>(frame_b.size() + 1);
  const uint16_t crc = crc16_en13757(frame_b.data(), frame_b.size());
  frame_b.push_back(static_cast<uint8_t>(crc >> 8));
  frame_b.push_back(static_cast<uint8_t>(crc & 0xFF));
  CHECK(strip_block_crcs(FrameFormat::B, frame_b.data(), frame_b.size(), out));
  CHECK(out.size() == sizeof(FULL_FRAME));
  CHECK(memcmp(out.data() + 1, FULL_FRAME + 1, sizeof(FULL_FRAME) - 1) == 0);
}

static void test_full_frame() {
  Decoder decoder = make_decoder();
  Reading reading;
  CHECK_RESULT(decoder.decode(FULL_FRAME, sizeof(FULL_FRAME), reading), DecodeResult::OK);
  CHECK(reading.meter_id == TEST_METER_ID);
  CHECK(!reading.compact);
  CHECK(reading.has_total && reading.total_l == 123456);
  CHECK(reading.has_target && reading.target_l == 120000);
  CHECK(reading.has_flow_temperature && reading.flow_temperature_c == 12);
  CHECK(reading.has_ambient_temperature && reading.ambient_temperature_c == 21);
  CHECK(reading.has_info_codes && reading.info_codes == 0);
}

static void test_compact_frame() {
  // Works right after boot thanks to the built-in MULTICAL 21 format.
  Decoder decoder = make_decoder();
  Reading reading;
  CHECK_RESULT(decoder.decode(COMPACT_FRAME, sizeof(COMPACT_FRAME), reading), DecodeResult::OK);
  CHECK(reading.compact);
  CHECK(reading.total_l == 123456);
  CHECK(reading.target_l == 120000);
  CHECK(reading.flow_temperature_c == 12);
  CHECK(reading.ambient_temperature_c == 21);

  // Corrupt a data byte after decryption is impossible to do directly, so flip a ciphertext
  // byte of the volume: the payload CRC catches it.
  std::vector<uint8_t> corrupted(COMPACT_FRAME, COMPACT_FRAME + sizeof(COMPACT_FRAME));
  corrupted[sizeof(COMPACT_FRAME) - 5] ^= 0x10;
  CHECK_RESULT(decoder.decode(corrupted.data(), corrupted.size(), reading), DecodeResult::PAYLOAD_CRC_MISMATCH);
}

static void test_alarms() {
  Decoder decoder = make_decoder();
  Reading reading;
  CHECK_RESULT(decoder.decode(COMPACT_ALARM_FRAME, sizeof(COMPACT_ALARM_FRAME), reading), DecodeResult::OK);
  CHECK(reading.info_codes == 0x0404);
  CHECK(reading.info_codes & INFO_LEAK);
  CHECK(!(reading.info_codes & INFO_BURST));
  CHECK(info_codes_status(reading.info_codes) == "LEAK");
  CHECK(info_codes_history(reading.info_codes) == "leak 1-8 hours");
  CHECK(info_codes_status(0) == "OK");
  CHECK(info_codes_history(0).empty());
  CHECK(info_codes_status(INFO_DRY | INFO_BURST) == "DRY, BURST");
  CHECK(info_codes_history(0x0010 | 0xE000) == "dry 1-8 hours, burst 22-31 days");
}

static void test_negative_temperature() {
  Decoder decoder = make_decoder();
  Reading reading;
  CHECK_RESULT(decoder.decode(NEGATIVE_TEMP_FRAME, sizeof(NEGATIVE_TEMP_FRAME), reading), DecodeResult::OK);
  CHECK(reading.flow_temperature_c == 2);
  CHECK(reading.ambient_temperature_c == -5);
}

static void test_rejections() {
  Decoder decoder = make_decoder();
  Reading reading;
  CHECK_RESULT(decoder.decode(OTHER_METER_FRAME, sizeof(OTHER_METER_FRAME), reading), DecodeResult::OTHER_METER);
  CHECK_RESULT(decoder.decode(WRONG_KEY_FRAME, sizeof(WRONG_KEY_FRAME), reading),
               DecodeResult::PAYLOAD_CRC_MISMATCH);
  CHECK_RESULT(decoder.decode(FULL_FRAME, 9, reading), DecodeResult::TOO_SHORT);

  std::vector<uint8_t> other_ci(FULL_FRAME, FULL_FRAME + sizeof(FULL_FRAME));
  other_ci[10] = 0x72;
  CHECK_RESULT(decoder.decode(other_ci.data(), other_ci.size(), reading), DecodeResult::UNSUPPORTED_CI);

  uint32_t id = 0;
  CHECK(Decoder::peek_meter_id(OTHER_METER_FRAME, sizeof(OTHER_METER_FRAME), id) && id == 0x87654321);
}

int main() {
  test_crc();
  test_aes();
  test_frame_length();
  test_strip_crcs();
  test_full_frame();
  test_compact_frame();
  test_alarms();
  test_negative_temperature();
  test_rejections();
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
