#pragma once
#include <array>
#include <cstddef>

#include "../util/misc.h"
#include "pin.h"

template <size_t num_pins>
class PinGroup {
 public:
  const std::array<Pin, num_pins> pins;
  constexpr static std::array<Port, NUM_PORTS> OUTPUT_ORDER = {Port::B, Port::C, Port::D};

  PinGroup(const PinGroup &) = default;

  template <AllSame<Pin>... Pins>
  constexpr PinGroup(Pins... pins) : pins{{std::forward<Pins>(pins)...}} {}
  // constexpr PinGroup(const std::array<Pin, num_pins> pins) : pins(pins){};

  void Init(Pin::Mode m) const {
    for (const Pin &pin : pins) {
      pin.Init(m);
    }
  }

  template <typename T>
  void Write(const T value) const {
    for (T i = 0; i < pins.size(); i++) {
      pins[i].Write(bit::Read(value, i));
    }
  }

  template <typename T = uint8_t>
  T Read() const {
    T out = 0;
    for (T i = 0; i < pins.size(); i++) {
      bit::Write(out, i, pins[i].Read());
    }
    return out;
  }

  constexpr size_t size() const { return num_pins; }

  /**
   * @brief This writes the contents of a PinGroup to the corresponding port(s) as close
   *        to as simultaneously as possible, with an order dictated by OUTPUT_ORDER. This
   *        is accomplished with direct PORT manipulation.
   *
   * @param value The value to write to the PinGroup
   */
  template <typename T>
  void WriteSimultaneous(T value) const {
    std::array<uint8_t, NUM_PORTS> port_outputs{};
    for (size_t bit = 0; bit < pins.size(); bit++) {
      const Pin &pin = pins[bit];
      const auto port_idx = std::to_underlying(pin.port());
      const bool bit_val = bit::Read<T>(value, bit);
      bit::Write(port_outputs[port_idx], pin.bit(), bit_val);
    }

    for (const Port &port : OUTPUT_ORDER) {
      const auto port_idx = std::to_underlying(port);
      if (inverse_bitmasks_[port_idx] == 0xFF) {
        // port is not part of pingroup (bitmask is all 1)
        continue;
      }
      volatile uint8_t &port_reg = Pin::state_addr(port);
      port_reg = (port_reg & inverse_bitmasks_[port_idx]) | port_outputs[port_idx];
    }
  }

 private:
  /** @brief Calculates the inverse bitmasks (i.e. which pins are used in the PinGroup) for all ports */
  static constexpr std::array<uint8_t, NUM_PORTS> calculate_inverse_bitmasks(const std::array<Pin, num_pins> &pins) {
    std::array<uint8_t, NUM_PORTS> port_masks{};  // all zeroes
    for (const Pin &pin : pins) {
      const uint8_t port_idx = std::to_underlying(pin.port());
      bit::Set(port_masks[port_idx], pin.bit());
    }

    for (uint8_t &mask : port_masks) {
      mask = ~mask;
    }

    return port_masks;
  }

  const std::array<uint8_t, NUM_PORTS> inverse_bitmasks_ = calculate_inverse_bitmasks(pins);
};