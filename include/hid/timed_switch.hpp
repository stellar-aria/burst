#pragma once
#include <stdint.h>

class TimedSwitch {
public:
  TimedSwitch() = default;
  ~TimedSwitch() = default;

  void Debounce(bool new_state) {
    switch_state_ = (switch_state_ << 1) | new_state;
  }

  /**
   * @brief Debounce the switch and update the internal time tracking
   *        This is useful for keeping track of time pressed
   *
   * @param new_state the new state of the switch (usually read from hardware)
   * @param time_now The current system time
   */
  void Debounce(bool new_state, uint32_t time_now) {
    switch_state_ = (switch_state_ << 1) | new_state;

    // long-press tracking
    if (this->just_pressed()) {
      counter_ = time_now;
    } else if (this->just_released()) {
      counter_ = 0;
    }    
  }

  bool just_released() const { return switch_state_ == 0x80; }
  bool released() const { return just_released() || switch_state_ == 0x00; }

  bool just_pressed() const { return switch_state_ == 0x7f; }
  bool pressed() const { return just_pressed() || switch_state_ == 0xFF; }

  /**
   * @brief Checks if the switch is being held for longer than a given duration
   *
   * @param duration The span of time the switch must be held for
   * @param time_now The current system time
   * @return true The switch was held for the required duration
   * @return false The switch was not held for the duration
   */
  bool pressed_for(uint32_t duration, uint32_t time_now) const {
    return this->pressed() && (time_now - counter_ > duration);
  }

  void ResetCounter(uint32_t time_now) {
    counter_ = time_now;
  }

private:
  uint8_t switch_state_ = 0x00;
  uint32_t counter_ = 0;
};