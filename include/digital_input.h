#pragma once
#include "hid/switch.hpp"
#include "sys/pin.h"
#include "util/digital_signal.h"


class DigitalInput {
  DigitalSignal signal_;
  const bool pullup;

 public:
  const Pin pin;

  DigitalInput(const Pin pin, bool pullup = false) : pullup(pullup), pin(pin) {}

  void Init() const {
    using enum Pin::Mode;
    pin.Init(pullup ? INPUT_PULLUP : INPUT);
  }

  bool Read() {
    const bool reading = pin.Read();
    signal_.Process(pullup ? !reading : reading);
    return signal_.high();
  }

  bool high() const { return signal_.high(); }
  bool rising() const { return signal_.rising(); }
  bool low() const { return signal_.low(); }
  bool falling() const { return signal_.falling(); }
};

using GateInput = DigitalInput;
using ClockInput = DigitalInput;
using TriggerInput = DigitalInput;