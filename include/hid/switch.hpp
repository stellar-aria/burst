#pragma once
#include <stdint.h>

class Switch {
public:
  Switch() = default;

  void Debounce(bool new_state) {
    switch_state_ = (switch_state_ << 1) | new_state;
  }

  bool just_released() const { return switch_state_ == 0x80; }
  bool released() const { return just_released() || switch_state_ == 0x00; }

  bool just_pressed() const { return switch_state_ == 0x7f; }
  bool pressed() const { return just_pressed() || switch_state_ == 0xFF; }

private:
  uint8_t switch_state_ = 0x00;
};