#include "protocol.h"

#include <cstring>

namespace esphome {
namespace multical21 {

// ---------------------------------------------------------------------------
// CRC
// ---------------------------------------------------------------------------

uint16_t crc16_en13757(const uint8_t *data, size_t len) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x3D65);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return static_cast<uint16_t>(~crc);
}

// ---------------------------------------------------------------------------
// AES-128 (FIPS-197). The S-box is computed instead of stored as a table.
// ---------------------------------------------------------------------------

namespace {

uint8_t gf_mul(uint8_t a, uint8_t b) {
  uint8_t product = 0;
  while (b) {
    if (b & 1)
      product ^= a;
    a = static_cast<uint8_t>((a << 1) ^ ((a & 0x80) ? 0x1B : 0x00));
    b >>= 1;
  }
  return product;
}

uint8_t rotl8(uint8_t x, int shift) { return static_cast<uint8_t>((x << shift) | (x >> (8 - shift))); }

const uint8_t *sbox() {
  static uint8_t table[256];
  static bool ready = false;
  if (!ready) {
    for (int i = 0; i < 256; i++) {
      uint8_t inverse = 0;
      if (i != 0) {
        for (int x = 1; x < 256; x++) {
          if (gf_mul(static_cast<uint8_t>(i), static_cast<uint8_t>(x)) == 1) {
            inverse = static_cast<uint8_t>(x);
            break;
          }
        }
      }
      table[i] = static_cast<uint8_t>(inverse ^ rotl8(inverse, 1) ^ rotl8(inverse, 2) ^ rotl8(inverse, 3) ^
                                      rotl8(inverse, 4) ^ 0x63);
    }
    ready = true;
  }
  return table;
}

}  // namespace

Aes128::Aes128(const uint8_t key[16]) {
  const uint8_t *s = sbox();
  static const uint8_t RCON[10] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36};
  memcpy(this->round_keys_, key, 16);
  for (int i = 4; i < 44; i++) {
    uint8_t temp[4];
    memcpy(temp, &this->round_keys_[(i - 1) * 4], 4);
    if (i % 4 == 0) {
      const uint8_t first = temp[0];
      temp[0] = static_cast<uint8_t>(s[temp[1]] ^ RCON[i / 4 - 1]);
      temp[1] = s[temp[2]];
      temp[2] = s[temp[3]];
      temp[3] = s[first];
    }
    for (int j = 0; j < 4; j++)
      this->round_keys_[i * 4 + j] = static_cast<uint8_t>(this->round_keys_[(i - 4) * 4 + j] ^ temp[j]);
  }
}

void Aes128::encrypt_block(const uint8_t in[16], uint8_t out[16]) const {
  const uint8_t *s = sbox();
  uint8_t state[16];
  for (int i = 0; i < 16; i++)
    state[i] = static_cast<uint8_t>(in[i] ^ this->round_keys_[i]);

  for (int round = 1; round <= 10; round++) {
    // SubBytes + ShiftRows (state is column-major: index = column * 4 + row)
    uint8_t shifted[16];
    for (int column = 0; column < 4; column++) {
      for (int row = 0; row < 4; row++)
        shifted[column * 4 + row] = s[state[((column + row) % 4) * 4 + row]];
    }
    // MixColumns (skipped in the last round)
    if (round != 10) {
      for (int column = 0; column < 4; column++) {
        const uint8_t *a = &shifted[column * 4];
        uint8_t mixed[4];
        mixed[0] = static_cast<uint8_t>(gf_mul(a[0], 2) ^ gf_mul(a[1], 3) ^ a[2] ^ a[3]);
        mixed[1] = static_cast<uint8_t>(a[0] ^ gf_mul(a[1], 2) ^ gf_mul(a[2], 3) ^ a[3]);
        mixed[2] = static_cast<uint8_t>(a[0] ^ a[1] ^ gf_mul(a[2], 2) ^ gf_mul(a[3], 3));
        mixed[3] = static_cast<uint8_t>(gf_mul(a[0], 3) ^ a[1] ^ a[2] ^ gf_mul(a[3], 2));
        memcpy(&shifted[column * 4], mixed, 4);
      }
    }
    // AddRoundKey
    for (int i = 0; i < 16; i++)
      state[i] = static_cast<uint8_t>(shifted[i] ^ this->round_keys_[round * 16 + i]);
  }
  memcpy(out, state, 16);
}

void aes128_ctr(const uint8_t key[16], const uint8_t iv[16], const uint8_t *in, uint8_t *out, size_t len) {
  Aes128 aes(key);
  uint8_t counter[16];
  uint8_t keystream[16];
  memcpy(counter, iv, 16);
  for (size_t offset = 0; offset < len; offset += 16) {
    aes.encrypt_block(counter, keystream);
    const size_t chunk = (len - offset) < 16 ? (len - offset) : 16;
    for (size_t i = 0; i < chunk; i++)
      out[offset + i] = static_cast<uint8_t>(in[offset + i] ^ keystream[i]);
    for (int i = 15; i >= 0; i--) {
      if (++counter[i] != 0)
        break;
    }
  }
}

// ---------------------------------------------------------------------------
// Link layer: frame length and block CRCs
// ---------------------------------------------------------------------------

// Format A: first block = L-field + 9 bytes, then blocks of 16 bytes; every block
// is followed by a 2-byte CRC (MSB first). The L-field does not count the CRCs.
// Format B: the L-field counts everything after itself, CRCs included. Block 1
// (10 bytes) and block 2 (up to 118 bytes, last 2 = CRC over blocks 1+2) form the
// first 128 bytes; an optional block 3 carries its own trailing CRC.

size_t frame_length_on_air(FrameFormat format, uint8_t l_field) {
  if (format == FrameFormat::A) {
    if (l_field < 9)
      return 0;
    const size_t remaining = l_field - 9;
    const size_t crc_count = 1 + (remaining + 15) / 16;
    return 1 + l_field + 2 * crc_count;
  }
  if (l_field < 12)
    return 0;
  return 1 + static_cast<size_t>(l_field);
}

static bool check_crc(const uint8_t *data, size_t len, const uint8_t *crc) {
  const uint16_t expected = static_cast<uint16_t>((crc[0] << 8) | crc[1]);
  return crc16_en13757(data, len) == expected;
}

bool strip_block_crcs(FrameFormat format, const uint8_t *raw, size_t raw_len, std::vector<uint8_t> &out) {
  out.clear();
  if (raw_len < 1)
    return false;
  const size_t expected = frame_length_on_air(format, raw[0]);
  if (expected == 0 || raw_len < expected)
    return false;

  if (format == FrameFormat::A) {
    if (!check_crc(raw, 10, raw + 10))
      return false;
    out.insert(out.end(), raw, raw + 10);
    size_t pos = 12;
    size_t remaining = raw[0] - 9;
    while (remaining > 0) {
      const size_t block = remaining < 16 ? remaining : 16;
      if (!check_crc(raw + pos, block, raw + pos + block))
        return false;
      out.insert(out.end(), raw + pos, raw + pos + block);
      pos += block + 2;
      remaining -= block;
    }
    return true;
  }

  // Format B
  const size_t first_part = expected < 128 ? expected : 128;  // blocks 1+2 including their CRC
  if (!check_crc(raw, first_part - 2, raw + first_part - 2))
    return false;
  out.insert(out.end(), raw, raw + first_part - 2);
  if (expected > 128) {
    const size_t block3 = expected - 128;
    if (block3 < 3 || !check_crc(raw + 128, block3 - 2, raw + 128 + block3 - 2))
      return false;
    out.insert(out.end(), raw + 128, raw + 128 + block3 - 2);
  }
  return true;
}

// ---------------------------------------------------------------------------
// Info codes
// ---------------------------------------------------------------------------

std::string info_codes_status(uint16_t info_codes) {
  std::string result;
  const struct {
    uint16_t bit;
    const char *name;
  } flags[] = {{INFO_DRY, "DRY"}, {INFO_REVERSE, "REVERSE"}, {INFO_LEAK, "LEAK"}, {INFO_BURST, "BURST"}};
  for (const auto &flag : flags) {
    if (info_codes & flag.bit) {
      if (!result.empty())
        result += ", ";
      result += flag.name;
    }
  }
  return result.empty() ? "OK" : result;
}

std::string info_codes_history(uint16_t info_codes) {
  static const char *const DURATIONS[8] = {"",         "1-8 hours",  "9-24 hours",  "2-3 days",
                                           "4-7 days", "8-14 days", "15-21 days", "22-31 days"};
  const struct {
    int shift;
    const char *name;
  } fields[] = {{4, "dry"}, {7, "reverse"}, {10, "leak"}, {13, "burst"}};
  std::string result;
  for (const auto &field : fields) {
    const int duration = (info_codes >> field.shift) & 0x07;
    if (duration == 0)
      continue;
    if (!result.empty())
      result += ", ";
    result += field.name;
    result += " ";
    result += DURATIONS[duration];
  }
  return result;
}

const char *decode_result_to_string(DecodeResult result) {
  switch (result) {
    case DecodeResult::OK:
      return "OK";
    case DecodeResult::TOO_SHORT:
      return "frame too short";
    case DecodeResult::OTHER_METER:
      return "frame from another meter";
    case DecodeResult::UNSUPPORTED_CI:
      return "unsupported CI field";
    case DecodeResult::UNSUPPORTED_ENCRYPTION:
      return "unsupported encryption mode";
    case DecodeResult::PAYLOAD_CRC_MISMATCH:
      return "payload CRC mismatch (wrong key?)";
    case DecodeResult::UNKNOWN_COMPACT_FORMAT:
      return "unknown compact frame format (waiting for a full frame)";
    case DecodeResult::FULL_FRAME_CRC_MISMATCH:
      return "compact frame CRC mismatch";
    case DecodeResult::MALFORMED:
      return "malformed data records";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// Application layer
// ---------------------------------------------------------------------------

namespace {

// Parses one DIF/DIFE/VIF/VIFE header. Returns false for unsupported records.
bool parse_record_header(const uint8_t *p, size_t len, size_t &header_len, size_t &data_len) {
  size_t i = 0;
  if (i >= len)
    return false;
  const uint8_t dif = p[i++];
  switch (dif & 0x0F) {
    case 0x0:
      data_len = 0;
      break;
    case 0x1:
      data_len = 1;
      break;
    case 0x2:
      data_len = 2;
      break;
    case 0x3:
      data_len = 3;
      break;
    case 0x4:
    case 0x5:
      data_len = 4;
      break;
    case 0x6:
      data_len = 6;
      break;
    case 0x7:
      data_len = 8;
      break;
    default:
      return false;  // variable length, BCD, manufacturer specific data: not used by MULTICAL 21
  }
  bool extension = dif & 0x80;
  while (extension) {
    if (i >= len)
      return false;
    extension = p[i++] & 0x80;
  }
  if (i >= len)
    return false;
  extension = p[i++] & 0x80;
  while (extension) {
    if (i >= len)
      return false;
    extension = p[i++] & 0x80;
  }
  header_len = i;
  return true;
}

uint32_t read_le(const uint8_t *p, size_t len) {
  uint32_t value = 0;
  for (size_t i = 0; i < len && i < 4; i++)
    value |= static_cast<uint32_t>(p[i]) << (8 * i);
  return value;
}

// Record layout of a MULTICAL 21 full frame: info codes, volume, target volume,
// flow temperature, ambient temperature. Lets compact frames decode right after boot.
const uint8_t DEFAULT_FORMAT[] = {0x02, 0xFF, 0x20, 0x04, 0x13, 0x44, 0x13, 0x61, 0x5B, 0x61, 0x67};

}  // namespace

Decoder::Decoder() {
  std::vector<uint8_t> headers(DEFAULT_FORMAT, DEFAULT_FORMAT + sizeof(DEFAULT_FORMAT));
  this->formats_[crc16_en13757(headers.data(), headers.size())] = headers;
}

void Decoder::set_key(const uint8_t key[16]) { memcpy(this->key_, key, 16); }

bool Decoder::peek_meter_id(const uint8_t *frame, size_t len, uint32_t &meter_id) {
  if (len < 10)
    return false;
  meter_id = read_le(frame + 4, 4);
  return true;
}

DecodeResult Decoder::decode(const uint8_t *frame, size_t len, Reading &out) {
  uint32_t meter_id;
  if (!peek_meter_id(frame, len, meter_id) || len < 11)
    return DecodeResult::TOO_SHORT;
  if (meter_id != this->meter_id_)
    return DecodeResult::OTHER_METER;

  // Extended link layer, CI 0x8D: CC, ACC, SN (4 bytes), then encrypted PayloadCRC + payload.
  if (frame[10] != 0x8D)
    return DecodeResult::UNSUPPORTED_CI;
  if (len < 20)
    return DecodeResult::TOO_SHORT;

  const uint8_t cc = frame[11];
  const uint32_t session = read_le(frame + 13, 4);
  const uint8_t encryption = (session >> 29) & 0x07;
  const uint8_t *data = frame + 17;
  const size_t data_len = len - 17;

  std::vector<uint8_t> plain(data_len);
  if (encryption == 0) {
    memcpy(plain.data(), data, data_len);
  } else if (encryption == 1) {
    // IV: M (2) | A (6) | CC | SN (4) | FN = 0 (2) | BC = 0
    uint8_t iv[16];
    memcpy(iv, frame + 2, 8);
    iv[8] = cc;
    memcpy(iv + 9, frame + 13, 4);
    iv[13] = 0;
    iv[14] = 0;
    iv[15] = 0;
    aes128_ctr(this->key_, iv, data, plain.data(), data_len);
  } else {
    return DecodeResult::UNSUPPORTED_ENCRYPTION;
  }

  const uint16_t payload_crc = static_cast<uint16_t>(plain[0] | (plain[1] << 8));
  if (crc16_en13757(plain.data() + 2, plain.size() - 2) != payload_crc)
    return DecodeResult::PAYLOAD_CRC_MISMATCH;

  out = Reading{};
  out.meter_id = meter_id;
  return this->decode_application_(plain.data() + 2, plain.size() - 2, out);
}

DecodeResult Decoder::decode_application_(const uint8_t *payload, size_t len, Reading &out) {
  if (len < 1)
    return DecodeResult::TOO_SHORT;

  if (payload[0] == 0x78) {  // full frame: DIF/VIF records
    std::vector<uint8_t> headers;
    if (!this->decode_records_(payload + 1, len - 1, out, &headers))
      return DecodeResult::MALFORMED;
    this->formats_[crc16_en13757(headers.data(), headers.size())] = headers;
    out.compact = false;
    return DecodeResult::OK;
  }

  if (payload[0] == 0x79) {  // compact frame: signature, full frame CRC, data only
    if (len < 5)
      return DecodeResult::TOO_SHORT;
    const uint16_t signature = static_cast<uint16_t>(payload[1] | (payload[2] << 8));
    const uint16_t full_crc = static_cast<uint16_t>(payload[3] | (payload[4] << 8));
    auto format = this->formats_.find(signature);
    if (format == this->formats_.end())
      return DecodeResult::UNKNOWN_COMPACT_FORMAT;

    // Rebuild the full frame records (header + data) so the CRC can be verified.
    const std::vector<uint8_t> &headers = format->second;
    std::vector<uint8_t> records;
    size_t header_pos = 0;
    size_t data_pos = 5;
    while (header_pos < headers.size()) {
      size_t header_len, data_len;
      if (!parse_record_header(&headers[header_pos], headers.size() - header_pos, header_len, data_len))
        return DecodeResult::MALFORMED;
      if (data_pos + data_len > len)
        return DecodeResult::MALFORMED;
      records.insert(records.end(), headers.begin() + header_pos, headers.begin() + header_pos + header_len);
      records.insert(records.end(), payload + data_pos, payload + data_pos + data_len);
      header_pos += header_len;
      data_pos += data_len;
    }
    if (crc16_en13757(records.data(), records.size()) != full_crc)
      return DecodeResult::FULL_FRAME_CRC_MISMATCH;
    if (!this->decode_records_(records.data(), records.size(), out, nullptr))
      return DecodeResult::MALFORMED;
    out.compact = true;
    return DecodeResult::OK;
  }

  return DecodeResult::UNSUPPORTED_CI;
}

bool Decoder::decode_records_(const uint8_t *records, size_t len, Reading &out, std::vector<uint8_t> *headers) {
  size_t pos = 0;
  while (pos < len) {
    if (records[pos] == 0x2F) {  // idle filler
      pos++;
      continue;
    }
    size_t header_len, data_len;
    if (!parse_record_header(records + pos, len - pos, header_len, data_len))
      return false;
    if (pos + header_len + data_len > len)
      return false;

    const uint8_t *header = records + pos;
    const uint8_t *data = records + pos + header_len;
    if (headers != nullptr)
      headers->insert(headers->end(), header, header + header_len);

    const uint8_t dif = header[0];
    const bool has_dife = dif & 0x80;
    const bool storage_1 = dif & 0x40;
    const uint8_t vif = header[1];
    if (!has_dife) {
      if (vif == 0xFF && header_len >= 3 && header[2] == 0x20 && data_len == 2) {
        out.has_info_codes = true;
        out.info_codes = static_cast<uint16_t>(read_le(data, 2));
      } else if (vif == 0x13 && data_len == 4) {  // volume, 1 litre resolution
        if (storage_1) {
          out.has_target = true;
          out.target_l = read_le(data, 4);
        } else {
          out.has_total = true;
          out.total_l = read_le(data, 4);
        }
      } else if (vif == 0x5B && data_len == 1) {  // flow temperature, 1 °C
        out.has_flow_temperature = true;
        out.flow_temperature_c = static_cast<int8_t>(data[0]);
      } else if (vif == 0x67 && data_len == 1) {  // external (ambient) temperature, 1 °C
        out.has_ambient_temperature = true;
        out.ambient_temperature_c = static_cast<int8_t>(data[0]);
      }
    }
    pos += header_len + data_len;
  }
  return true;
}

}  // namespace multical21
}  // namespace esphome
