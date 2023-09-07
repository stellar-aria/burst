#pragma once
#include <cstdint>
#include <numeric>

/** xoroshiro32++. Very fast, not-cryptographic random number generator. */
class Xoroshiro32PlusPlus {
 public:
  using result_type = uint16_t;

  void Seed(uint8_t s00, uint8_t s01, uint8_t s10, uint8_t s11) {
    state[0] = (s00 << 8) | s01;
    state[1] = (s10 << 8) | s11;
    // A bad seed will give a bad first result, so shift the state
    operator()();
  }

  bool is_seeded() { return state[0] || state[1]; }

  uint16_t operator()() {
    uint16_t s0 = state[0];
    uint16_t s1 = state[1];
    uint16_t result = rotl(s0 + s1, 7) + s0;

    s1 ^= s0;
    state[0] = rotl(s0, 13) ^ s1 ^ (s1 << 3);
    state[1] = rotl(s1, 9);

    return result;
  }
  constexpr uint16_t min() const {
    return std::numeric_limits<result_type>::min();
  }
  constexpr uint16_t max() const {
    return std::numeric_limits<result_type>::max();
  }

 private:
  static constexpr uint16_t rotl(uint16_t x, int k) {
    return (x << k) | (x >> (16 - k));
  }
  uint16_t state[2] = {};
};