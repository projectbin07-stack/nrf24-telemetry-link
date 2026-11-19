// Minimal JSON reader for the shared test vectors (host tests only).
//
// Supports objects, arrays, strings with simple escapes, numbers, true,
// false and null - enough for test/vectors/link_vectors.json. Not a
// general-purpose parser: no \u escapes beyond ASCII, no error recovery.
// (Same reader as in the encoder-odometry-firmware project.)
#pragma once

#include <stdlib.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace minijson {

struct Value {
  enum Type { kNull, kBool, kNumber, kString, kArray, kObject };
  Type type = kNull;
  bool boolean = false;
  double number = 0.0;
  std::string string;
  std::vector<Value> items;         // array elements or object values
  std::vector<std::string> keys;    // object keys, parallel to items

  const Value& operator[](const std::string& key) const {
    static const Value missing;
    for (size_t i = 0; i < keys.size(); ++i) {
      if (keys[i] == key) {
        return items[i];
      }
    }
    return missing;
  }
  const Value& at(size_t i) const { return items[i]; }
  size_t size() const { return items.size(); }
  bool has(const std::string& key) const {
    for (const std::string& k : keys) {
      if (k == key) {
        return true;
      }
    }
    return false;
  }
};

class Parser {
 public:
  explicit Parser(const std::string& text) : s_(text) {}

  bool parse(Value* out) {
    ok_ = true;
    pos_ = 0;
    *out = value();
    ws();
    return ok_ && pos_ == s_.size();
  }

 private:
  void ws() {
    while (pos_ < s_.size() && (s_[pos_] == ' ' || s_[pos_] == '\n' || s_[pos_] == '\r' ||
                                s_[pos_] == '\t')) {
      ++pos_;
    }
  }
  bool eat(char c) {
    ws();
    if (pos_ < s_.size() && s_[pos_] == c) {
      ++pos_;
      return true;
    }
    return false;
  }
  bool literal(const char* word) {
    const std::string w(word);
    if (s_.compare(pos_, w.size(), w) == 0) {
      pos_ += w.size();
      return true;
    }
    return false;
  }
  std::string str() {
    std::string out;
    if (!eat('"')) {
      ok_ = false;
      return out;
    }
    while (pos_ < s_.size() && s_[pos_] != '"') {
      char c = s_[pos_++];
      if (c == '\\' && pos_ < s_.size()) {
        const char e = s_[pos_++];
        switch (e) {
          case 'n': c = '\n'; break;
          case 't': c = '\t'; break;
          case 'r': c = '\r'; break;
          case 'u':
            // \u00XX only; enough for ASCII.
            c = static_cast<char>(strtol(s_.substr(pos_, 4).c_str(), nullptr, 16));
            pos_ += 4;
            break;
          default: c = e; break;
        }
      }
      out.push_back(c);
    }
    if (pos_ >= s_.size()) {
      ok_ = false;
      return out;
    }
    ++pos_;  // closing quote
    return out;
  }
  Value value() {
    Value v;
    ws();
    if (pos_ >= s_.size()) {
      ok_ = false;
      return v;
    }
    const char c = s_[pos_];
    if (c == '{') {
      ++pos_;
      v.type = Value::kObject;
      if (eat('}')) {
        return v;
      }
      do {
        ws();
        v.keys.push_back(str());
        if (!eat(':')) {
          ok_ = false;
          return v;
        }
        v.items.push_back(value());
      } while (ok_ && eat(','));
      if (!eat('}')) {
        ok_ = false;
      }
    } else if (c == '[') {
      ++pos_;
      v.type = Value::kArray;
      if (eat(']')) {
        return v;
      }
      do {
        v.items.push_back(value());
      } while (ok_ && eat(','));
      if (!eat(']')) {
        ok_ = false;
      }
    } else if (c == '"') {
      v.type = Value::kString;
      v.string = str();
    } else if (literal("true")) {
      v.type = Value::kBool;
      v.boolean = true;
    } else if (literal("false")) {
      v.type = Value::kBool;
    } else if (literal("null")) {
      v.type = Value::kNull;
    } else {
      const char* begin = s_.c_str() + pos_;
      char* end = nullptr;
      v.number = strtod(begin, &end);
      if (end == begin) {
        ok_ = false;
      } else {
        v.type = Value::kNumber;
        pos_ += static_cast<size_t>(end - begin);
      }
    }
    return v;
  }

  const std::string& s_;
  size_t pos_ = 0;
  bool ok_ = true;
};

inline bool read_file(const std::string& path, std::string* out) {
  std::ifstream in(path.c_str(), std::ios::in | std::ios::binary);
  if (!in) {
    return false;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  *out = ss.str();
  return true;
}

}  // namespace minijson
