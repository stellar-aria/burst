#pragma once
#include <cstddef>
#include <type_traits>

#include "../sys/pin.h"
#include "../sys/pin_group.h"

/**
 * @brief A multiplexer allows access to 2^n pins using only n + 1 pins,
 *        (selectors + signal) albeit only one at a time
 *
 * @tparam num_pins
 */
template <size_t num_pins>
class Mux {
  static_assert(num_pins <= 8,
                "You've tried to use a multiplexer with more than eight selector lines. "
                "Please investigate a port expander or alternate MCU");

  PinGroup<num_pins> selector_;
  Pin data_;

 public:
  /**
   * @brief Construct a new Mux object
   *
   * @param selector the pins used for selection in the mux, ordered from LSB to MSB
   * @param data The pin used for data input or output
   */
  constexpr Mux(PinGroup<num_pins> selector, Pin data) : selector_{selector}, data_{data} {};

  /**
   * @brief Construct a new Mux object with no data pin
   * @param selector the pins used for selection in the mux, ordered from LSB to MSB
   */
  constexpr Mux(PinGroup<num_pins> selector) : selector_{selector}, data_{Port::X, 0} {};

  void Init() const { selector_.Init(Pin::Mode::OUTPUT); }

  void Select(uint8_t mux_pin) const { selector_.WriteSimultaneous(mux_pin); }

  // void Write(bool data) const { data_.Write(data); }

  void Write(uint8_t mux_pin, bool data) const {
    Select(mux_pin);
    Write(data);
  }

  bool Read() const { return data_.Read(); }

  bool Read(uint8_t mux_pin) const {
    Select(mux_pin);
    return Read();
  }

  auto ReadAnalog() const { return data_.ReadAnalog(); }

  auto ReadAnalog(uint8_t mux_pin) const {
    Select(mux_pin);
    return ReadAnalog();
  }

  constexpr size_t size() const { return num_pins; }
  const PinGroup<num_pins> &selector() const { return selector_; }
  const Pin &data() const { return data_; }
};