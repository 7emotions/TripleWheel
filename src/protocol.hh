#pragma once

#include <cstdint>

namespace protocol {

struct __attribute__((packed)) Package {
  uint8_t header_a = 0x2c;
  uint8_t header_b = 0x12;
  float v;
  float w;
  uint8_t end = 0x5b;
};
} // namespace protocol