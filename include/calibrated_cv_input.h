#pragma once
#include <stdint.h>

#include <algorithm>

#include "sys/pin.h"
#include "util/misc.h"

class CalibratedCVInput {
  constexpr static uint16_t ANALOG_MIN = 20;    // 0 + 20
  constexpr static uint16_t ANALOG_MAX = 1003;  // 1023 - 20

 public:
  Pin pin;

  CalibratedCVInput(Pin pin, uint16_t &calibration, int16_t start, int16_t midpoint, int16_t stop,
                    bool inverted = false)
      : pin(pin), calibration(calibration), start_(start), midpoint_(midpoint), stop_(stop), inverted_(inverted) {};

  void Init() { pin.Init(Pin::Mode::INPUT); }

  uint16_t Calibrate() const { return calibration = pin.ReadAnalog(inverted_); }

  uint16_t Read() {
    const uint16_t raw = pin.ReadAnalog<uint16_t>(inverted_);
    last_value_ = value_;
    value_ = Scale(raw);
    return raw;
  }

  uint16_t Scaled() const { return value_; }
  uint16_t Last() const { return last_value_; }

  bool changed() { return value_ != last_value_; }

 private:
  int16_t Scale(uint16_t value) const {
    // buffer of -5 from calibration
    uint16_t calibrated_value = calibration - 5;

    // chop 20 off of both sides
    uint16_t val = std::clamp(value, ANALOG_MIN, ANALOG_MAX);

    uint16_t in_min, in_max;
    int16_t out_min, out_max;
    if (val <= calibrated_value) {
      in_min = ANALOG_MIN;
      in_max = calibrated_value + 1;
      out_min = start_;
      out_max = midpoint_ + (start_ > stop_ ? -1 : 1);
    } else {
      in_min = calibrated_value;
      in_max = ANALOG_MAX + 1;
      out_min = midpoint_;
      out_max = stop_ + (start_ > stop_ ? -1 : 1);
    }
    return map<int16_t>(val, in_min, in_max, out_min, out_max);
  }

  uint16_t value_ = 0;
  uint16_t last_value_ = 0;

  uint16_t &calibration;
  int16_t start_;
  int16_t midpoint_;
  int16_t stop_;
  bool inverted_;
};