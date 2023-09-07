#pragma once
#include <avr/io.h>

#include <cstdint>
#include <tuple>

#include "../util/bit_ops.h"
#include "../util/misc.h"

constexpr uint16_t MAX_ADC_VALUE = 0b1111111111;

constexpr uint8_t NUM_PORTS = 3;
enum class Port : uint8_t { B, C, D, X };

class Pin {
  const Port port_;
  const uint8_t bit_;

 public:
  enum class Mode { INPUT, INPUT_PULLUP, OUTPUT };

  constexpr Pin(Port p, uint8_t b) : port_{p}, bit_{b} {};

  void Init(Mode m) const {
    auto [dir, state] = dir_and_state_for_mode(m);
    bit::Write(dir_addr(port_), bit_, dir);
    bit::Write(state_addr(port_), bit_, state);
  }

  template <typename T>
  void Write(T value) const {
    bit::Write(state_addr(port_), bit_, value);
  }

  template <typename T = bool>
  T Read() const {
    return bit::Read<uint8_t>(input_addr(port_), bit_);
  }

  template <typename T>
  T ReadAnalog(bool inverted = false) const;

  constexpr Port port() const { return port_; }
  constexpr uint8_t bit() const { return bit_; }

  auto operator<=>(const Pin &) const = default;

 private:
  void SampleADC() const {
    // set the analog reference (high two bits of ADMUX) and select the
    // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
    // to 0 (the default).
    ADMUX = _BV(6) | (bit_ & 0x07);

    // start the conversion
    ADCSRA = ADCSRA | _BV(ADSC);

    // ADSC is cleared when the conversion finishes
    while (bit_is_set(ADCSRA, ADSC))
      ;
  }

 public:
  static constexpr std::pair<uint8_t, uint8_t> dir_and_state_for_mode(Mode m) {
    using enum Mode;
    switch (m) {
      case INPUT: return std::make_pair(0x00, 0x00);
      case INPUT_PULLUP: return std::make_pair(0x00, 0x01);
      case OUTPUT: return std::make_pair(0x01, 0x00);
      default: unreachable();
    }
  }

  static constexpr volatile uint8_t &state_addr(Port p) {
    using enum Port;
    switch (p) {
      case B: return PORTB;
      case C: return PORTC;
      case D: return PORTD;
      default: unreachable();
    }
  }

  static constexpr volatile uint8_t &dir_addr(Port p) {
    using enum Port;
    switch (p) {
      case B: return DDRB;
      case C: return DDRC;
      case D: return DDRD;
      default: unreachable();
    }
  }

  static constexpr volatile uint8_t &input_addr(Port p) {
    using enum Port;
    switch (p) {
      case B: return PINB;
      case C: return PINC;
      case D: return PIND;
      default: unreachable();
    }
  }
};

template <typename T = uint16_t>
T Pin::ReadAnalog(bool inverted) const {
  if (port_ != Port::C || bit_ > 5) {
    return 0;
  }
  SampleADC();

  // ADC macro takes care of reading ADC register.
  // avr-gcc implements the proper reading order: ADCL is read first.
  return inverted ? MAX_ADC_VALUE - ADCW : ADCW;
}

template <>
uint8_t Pin::ReadAnalog(bool inverted) const {
  if (port_ != Port::C || bit_ > 5) {
    return 0;
  }
  SampleADC();
  return inverted ? 0xFF - ADCH : ADCH;
}