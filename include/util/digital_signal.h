#pragma once
#include <cstdint>

class DigitalSignal {
  enum class State {
    LOW,
    HIGH,
    RISING,
    FALLING,
  };

  State state_ = State::LOW;

 public:
  DigitalSignal() = default;

  void Process(bool new_state) {
    using enum State;
    if (this->high()) {    // currently HIGH
      if (new_state) {     // now HIGH
        state_ = HIGH;     //<
      } else {             // now LOW
        state_ = FALLING;  //<
      }
    } else {              // currently LOW
      if (new_state) {    // now HIGH
        state_ = RISING;  //<
      } else {            // now low
        state_ = LOW;     //<
      }
    }
  }

  void Reset() { state_ = State::LOW; }

  bool high() const { return state_ == State::HIGH || state_ == State::RISING; }
  bool low() const { return state_ == State::LOW || state_ == State::FALLING; }
  bool rising() const { return state_ == State::RISING; }
  bool falling() const { return state_ == State::FALLING; }

  operator bool() const { return this->high(); }
  void operator=(const bool new_state) { this->Process(new_state); }
};

using GateSignal = DigitalSignal;
using ClockSignal = DigitalSignal;
using TriggerSignal = DigitalSignal;