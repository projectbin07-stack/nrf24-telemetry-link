#include "linkcore/messages.h"

#include <string.h>

#include "linkcore/bytes.h"

namespace linkcore {

static size_t start(MsgType type, size_t size, uint8_t* out, size_t capacity) {
  if (capacity < size) {
    return 0;
  }
  out[0] = kSchemaVersion;
  out[1] = static_cast<uint8_t>(type);
  return size;
}

static bool check(const uint8_t* data, size_t length, MsgType type, size_t size) {
  MsgType t;
  return length == size && message_type(data, length, &t) && t == type;
}

static int16_t get_i16(const uint8_t* p) { return static_cast<int16_t>(get_u16(p)); }
static int32_t get_i32(const uint8_t* p) { return static_cast<int32_t>(get_u32(p)); }

bool message_type(const uint8_t* data, size_t length, MsgType* type) {
  if (length < 2 || data[0] != kSchemaVersion) {
    return false;
  }
  *type = static_cast<MsgType>(data[1]);
  return true;
}

size_t encode_battery(const Battery& m, uint8_t* out, size_t capacity) {
  if (start(MsgType::kBattery, kBatterySize, out, capacity) == 0) return 0;
  put_u16(out + 2, m.bus_mv);
  put_u32(out + 4, static_cast<uint32_t>(m.current_ua));
  put_u32(out + 8, static_cast<uint32_t>(m.power_uw));
  put_u32(out + 12, static_cast<uint32_t>(m.charge_uah));
  put_u16(out + 16, m.soc_permille);
  out[18] = m.flags;
  return kBatterySize;
}

bool decode_battery(const uint8_t* d, size_t length, Battery* m) {
  if (!check(d, length, MsgType::kBattery, kBatterySize)) return false;
  m->bus_mv = get_u16(d + 2);
  m->current_ua = get_i32(d + 4);
  m->power_uw = get_i32(d + 8);
  m->charge_uah = get_i32(d + 12);
  m->soc_permille = get_u16(d + 16);
  m->flags = d[18];
  return true;
}

size_t encode_odometry(const Odometry& m, uint8_t* out, size_t capacity) {
  if (start(MsgType::kOdometry, kOdometrySize, out, capacity) == 0) return 0;
  put_u32(out + 2, m.t_ms);
  put_u32(out + 6, static_cast<uint32_t>(m.x_mm));
  put_u32(out + 10, static_cast<uint32_t>(m.y_mm));
  put_u16(out + 14, static_cast<uint16_t>(m.theta_mrad));
  put_u16(out + 16, static_cast<uint16_t>(m.v_mm_s));
  put_u16(out + 18, static_cast<uint16_t>(m.w_mrad_s));
  return kOdometrySize;
}

bool decode_odometry(const uint8_t* d, size_t length, Odometry* m) {
  if (!check(d, length, MsgType::kOdometry, kOdometrySize)) return false;
  m->t_ms = get_u32(d + 2);
  m->x_mm = get_i32(d + 6);
  m->y_mm = get_i32(d + 10);
  m->theta_mrad = get_i16(d + 14);
  m->v_mm_s = get_i16(d + 16);
  m->w_mrad_s = get_i16(d + 18);
  return true;
}

size_t encode_imu(const Imu& m, uint8_t* out, size_t capacity) {
  if (start(MsgType::kImu, kImuSize, out, capacity) == 0) return 0;
  put_u32(out + 2, m.t_ms);
  for (uint8_t i = 0; i < 3; ++i) {
    put_u16(out + 6 + 2 * i, static_cast<uint16_t>(m.accel_mg[i]));
    put_u16(out + 12 + 2 * i, static_cast<uint16_t>(m.gyro_cdps[i]));
  }
  put_u16(out + 18, static_cast<uint16_t>(m.temp_cc));
  return kImuSize;
}

bool decode_imu(const uint8_t* d, size_t length, Imu* m) {
  if (!check(d, length, MsgType::kImu, kImuSize)) return false;
  m->t_ms = get_u32(d + 2);
  for (uint8_t i = 0; i < 3; ++i) {
    m->accel_mg[i] = get_i16(d + 6 + 2 * i);
    m->gyro_cdps[i] = get_i16(d + 12 + 2 * i);
  }
  m->temp_cc = get_i16(d + 18);
  return true;
}

size_t encode_command(const Command& m, uint8_t* out, size_t capacity) {
  if (start(MsgType::kCommand, kCommandSize, out, capacity) == 0) return 0;
  out[2] = m.opcode;
  put_u32(out + 3, static_cast<uint32_t>(m.arg));
  return kCommandSize;
}

bool decode_command(const uint8_t* d, size_t length, Command* m) {
  if (!check(d, length, MsgType::kCommand, kCommandSize)) return false;
  m->opcode = d[2];
  m->arg = get_i32(d + 3);
  return true;
}

// --- key/value ---

static bool valid_key(const char* key, size_t* length) {
  if (key == nullptr) return false;
  const size_t n = strlen(key);
  if (n == 0 || n > kKvMaxKey) return false;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t c = static_cast<uint8_t>(key[i]);
    if (c < 0x21 || c > 0x7E) return false;
  }
  *length = n;
  return true;
}

KvWriter::KvWriter(uint8_t* out, size_t capacity) : out_(out), capacity_(capacity) {
  if (capacity_ < 3) {
    ok_ = false;
    return;
  }
  out_[0] = kSchemaVersion;
  out_[1] = static_cast<uint8_t>(MsgType::kKeyValue);
  out_[2] = 0;
  length_ = 3;
}

bool KvWriter::begin_entry(const char* key, KvType type, size_t value_size) {
  size_t key_length = 0;
  if (!ok_ || !valid_key(key, &key_length) || out_[2] >= kKvMaxEntries ||
      length_ + 1 + key_length + 1 + value_size > capacity_) {
    ok_ = false;
    return false;
  }
  out_[length_++] = static_cast<uint8_t>(key_length);
  memcpy(out_ + length_, key, key_length);
  length_ += key_length;
  out_[length_++] = static_cast<uint8_t>(type);
  out_[2] = static_cast<uint8_t>(out_[2] + 1);
  return true;
}

bool KvWriter::add_int(const char* key, int32_t value) {
  if (!begin_entry(key, KvType::kInt, 4)) return false;
  put_u32(out_ + length_, static_cast<uint32_t>(value));
  length_ += 4;
  return true;
}

bool KvWriter::add_float(const char* key, float value) {
  if (!begin_entry(key, KvType::kFloat, 4)) return false;
  put_f32(out_ + length_, value);
  length_ += 4;
  return true;
}

bool KvWriter::add_bool(const char* key, bool value) {
  if (!begin_entry(key, KvType::kBool, 1)) return false;
  out_[length_++] = value ? 1 : 0;
  return true;
}

bool KvWriter::add_string(const char* key, const char* value) {
  const size_t n = value == nullptr ? 0 : strlen(value);
  if (n > kKvMaxString) {
    ok_ = false;
    return false;
  }
  if (!begin_entry(key, KvType::kString, 1 + n)) return false;
  out_[length_++] = static_cast<uint8_t>(n);
  if (n > 0) {
    memcpy(out_ + length_, value, n);
  }
  length_ += n;
  return true;
}

bool KvReader::parse(const uint8_t* d, size_t length, size_t* pos, KvEntry* e) {
  size_t p = *pos;
  if (p >= length) return false;
  const uint8_t key_length = d[p++];
  if (key_length == 0 || key_length > kKvMaxKey || p + key_length + 1 > length) return false;
  for (uint8_t i = 0; i < key_length; ++i) {
    if (d[p + i] < 0x21 || d[p + i] > 0x7E) return false;
  }
  e->key = reinterpret_cast<const char*>(d + p);
  e->key_length = key_length;
  p += key_length;
  const uint8_t tag = d[p++];
  switch (tag) {
    case 0:
    case 1:
      if (p + 4 > length) return false;
      e->type = tag == 0 ? KvType::kInt : KvType::kFloat;
      e->int_value = static_cast<int32_t>(get_u32(d + p));
      e->float_value = get_f32(d + p);
      p += 4;
      break;
    case 2:
      if (p + 1 > length || d[p] > 1) return false;
      e->type = KvType::kBool;
      e->bool_value = d[p] == 1;
      p += 1;
      break;
    case 3: {
      if (p + 1 > length) return false;
      const uint8_t n = d[p++];
      if (n > kKvMaxString || p + n > length) return false;
      e->type = KvType::kString;
      e->string_value = reinterpret_cast<const char*>(d + p);
      e->string_length = n;
      p += n;
      break;
    }
    default:
      return false;
  }
  *pos = p;
  return true;
}

KvReader::KvReader(const uint8_t* data, size_t length) : data_(data), length_(length) {
  MsgType type;
  if (length < 3 || !message_type(data, length, &type) || type != MsgType::kKeyValue ||
      data[2] > kKvMaxEntries) {
    return;
  }
  count_ = data[2];
  size_t pos = 3;
  for (uint8_t i = 0; i < count_; ++i) {
    KvEntry e;
    if (!parse(data, length, &pos, &e)) return;
  }
  valid_ = pos == length;
}

bool KvReader::next(KvEntry* entry) {
  if (!valid_ || read_ >= count_) return false;
  KvEntry e;
  if (!parse(data_, length_, &pos_, &e)) return false;
  ++read_;
  *entry = e;
  return true;
}

}  // namespace linkcore
