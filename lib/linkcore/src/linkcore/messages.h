// Compact, versioned application message schema carried by the link layer.
//
//   message = version:u8 (=1) type:u8 body
//
// All integers are little-endian; values are fixed-point integers so the
// encoding is exact on every target. Bodies:
//
//   0x01 battery    bus_mv:u16 current_ua:i32 power_uw:i32 charge_uah:i32
//                   soc_permille:u16 flags:u8                      (17 bytes)
//                   fields as in the ina219-battery-monitor project's $BM1 line
//   0x02 odometry   t_ms:u32 x_mm:i32 y_mm:i32 theta_mrad:i16 v_mm_s:i16
//                   w_mrad_s:i16                                   (18 bytes)
//   0x03 imu        t_ms:u32 accel_mg:i16[3] gyro_cdps:i16[3] temp_cc:i16
//                                                                   (18 bytes)
//   0x04 key_value  count:u8 then count x
//                   ( key_len:u8 (1..15) key:ascii tag:u8 value )
//                   tag 0 int32 (4) | 1 float32 (4) | 2 bool (1) |
//                       3 string (len:u8 0..32, bytes)
//   0x10 command    opcode:u8 arg:i32                              (5 bytes)
//
// Shared test vectors: test/vectors/link_vectors.json ("messages").
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace linkcore {

constexpr uint8_t kSchemaVersion = 1;

enum class MsgType : uint8_t {
  kBattery = 0x01,
  kOdometry = 0x02,
  kImu = 0x03,
  kKeyValue = 0x04,
  kCommand = 0x10,
};

enum class CommandOp : uint8_t {
  kSetTelemetryPeriodMs = 1,  // arg: period in ms (20..60000)
  kSetPaLevel = 2,            // arg: 0..3 (-18, -12, -6, 0 dBm)
  kRequestStatus = 3,         // node answers with a key_value status message
  kIdentify = 4,              // arg: blink the LED for this many ms
};

/// Battery flag bits (same meaning as in ina219-battery-monitor).
constexpr uint8_t kBatteryFlagOverflow = 0x01;
constexpr uint8_t kBatteryFlagNoCurrentSensor = 0x80;

struct Battery {
  uint16_t bus_mv = 0;
  int32_t current_ua = 0;
  int32_t power_uw = 0;
  int32_t charge_uah = 0;
  uint16_t soc_permille = 0;
  uint8_t flags = 0;
};

struct Odometry {
  uint32_t t_ms = 0;
  int32_t x_mm = 0;
  int32_t y_mm = 0;
  int16_t theta_mrad = 0;
  int16_t v_mm_s = 0;
  int16_t w_mrad_s = 0;
};

struct Imu {
  uint32_t t_ms = 0;
  int16_t accel_mg[3] = {0, 0, 0};
  int16_t gyro_cdps[3] = {0, 0, 0};
  int16_t temp_cc = 0;
};

struct Command {
  uint8_t opcode = 0;
  int32_t arg = 0;
};

constexpr size_t kBatterySize = 2 + 17;
constexpr size_t kOdometrySize = 2 + 18;
constexpr size_t kImuSize = 2 + 18;
constexpr size_t kCommandSize = 2 + 5;

/// Checks the version byte and returns the type. False if too short or
/// another schema version.
bool message_type(const uint8_t* data, size_t length, MsgType* type);

// Encoders return the encoded size, or 0 if `capacity` is too small.
size_t encode_battery(const Battery& m, uint8_t* out, size_t capacity);
size_t encode_odometry(const Odometry& m, uint8_t* out, size_t capacity);
size_t encode_imu(const Imu& m, uint8_t* out, size_t capacity);
size_t encode_command(const Command& m, uint8_t* out, size_t capacity);

// Decoders require the exact length and the right version and type.
bool decode_battery(const uint8_t* data, size_t length, Battery* m);
bool decode_odometry(const uint8_t* data, size_t length, Odometry* m);
bool decode_imu(const uint8_t* data, size_t length, Imu* m);
bool decode_command(const uint8_t* data, size_t length, Command* m);

enum class KvType : uint8_t { kInt = 0, kFloat = 1, kBool = 2, kString = 3 };

constexpr uint8_t kKvMaxKey = 15;
constexpr uint8_t kKvMaxString = 32;
constexpr uint8_t kKvMaxEntries = 15;

/// Streams key/value entries straight into a caller buffer (no heap, no
/// intermediate struct, so it fits an ATmega328P).
class KvWriter {
 public:
  KvWriter(uint8_t* out, size_t capacity);
  bool add_int(const char* key, int32_t value);
  bool add_float(const char* key, float value);
  bool add_bool(const char* key, bool value);
  bool add_string(const char* key, const char* value);
  /// Encoded size, or 0 if any add failed (overflow, bad key, too many entries).
  size_t finish() const { return ok_ ? length_ : 0; }
  bool ok() const { return ok_; }

 private:
  bool begin_entry(const char* key, KvType type, size_t value_size);

  uint8_t* out_;
  size_t capacity_;
  size_t length_ = 0;
  bool ok_ = true;
};

struct KvEntry {
  const char* key = nullptr;  // not NUL-terminated
  uint8_t key_length = 0;
  KvType type = KvType::kInt;
  int32_t int_value = 0;
  float float_value = 0.0f;
  bool bool_value = false;
  const char* string_value = nullptr;  // not NUL-terminated
  uint8_t string_length = 0;
};

/// Validates a whole key_value message up front; next() then walks it.
class KvReader {
 public:
  KvReader(const uint8_t* data, size_t length);
  bool valid() const { return valid_; }
  uint8_t count() const { return count_; }
  bool next(KvEntry* entry);

 private:
  static bool parse(const uint8_t* data, size_t length, size_t* pos, KvEntry* entry);

  const uint8_t* data_;
  size_t length_;
  size_t pos_ = 3;
  uint8_t count_ = 0;
  uint8_t read_ = 0;
  bool valid_ = false;
};

}  // namespace linkcore
