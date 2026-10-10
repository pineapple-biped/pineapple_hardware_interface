#pragma once
#include <algorithm>
#include <cstdint>
#include <vector>
#include "crc.h"

// Adapter envelope: A5, type, little-endian payload length, CAN payload,
// CRC16(length + payload), 5A. CAN payload contains one or more CAN records.
class UsbRxStream {
 public:
  uint64_t frames = 0, rejected = 0;
  size_t pending() const { return bytes_.size(); }
  template<class Emit>
  void feed(const uint8_t* data, size_t size, Emit emit) {
    for (size_t i = 0; i < size; ++i) {
      bytes_.push_back(data[i]);
      while (!bytes_.empty()) {
        if (bytes_[0] != 0xa5) { discard(); continue; }
        if (bytes_.size() < 4) break;
        const size_t payload = bytes_[2] | (size_t(bytes_[3]) << 8);
        if (payload < 12 || payload > 65535) { discard(); continue; }
        const size_t total = payload + 7;
        if (bytes_.size() < total) break;
        const auto crc = uint16_t(bytes_[payload + 4]) |
                         (uint16_t(bytes_[payload + 5]) << 8);
        if (bytes_[total - 1] != 0x5a ||
            CRC16(bytes_.data() + 2, payload + 2) != crc) {
          discard(); continue;
        }
        emit(bytes_.data() + 2, payload + 4);
        ++frames;
        bytes_.erase(bytes_.begin(), bytes_.begin() + total);
      }
    }
  }
 private:
  void discard() { bytes_.erase(bytes_.begin()); ++rejected; }
  std::vector<uint8_t> bytes_ = [] { std::vector<uint8_t> v; v.reserve(65542); return v; }();
};
