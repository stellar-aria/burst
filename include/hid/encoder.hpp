#pragma once
#include <cstdint>

template <typename SwitchType>
struct Encoder {
  // Note: 'switch' is a reserved keyword :(
  SwitchType sw; // The built-in switch of an encoder

  void Process(bool quadrature_a, bool quadrature_b) {
    quadrature_a_ = (quadrature_a_ << 1) | quadrature_a;
    quadrature_b_ = (quadrature_b_ << 1) | quadrature_b;
  }  

  void Process(bool button_state, bool quadrature_a, bool quadrature_b) {
    sw.Debounce(button_state);
    Process(quadrature_a, quadrature_b);
  }

  int8_t increment() const {
    uint8_t a = quadrature_a_ & 0b11; // only care about last two samples
    uint8_t b = quadrature_b_ & 0b11;
    if (a == 0b10 && b == 0b00) {
      return -1;
    } else if (a == 0b00 && b == 0b10) {
      return 1;
    } else {
      return 0;
    }
  }

private:
  uint8_t quadrature_a_ = 0x00;
  uint8_t quadrature_b_ = 0x00;
};