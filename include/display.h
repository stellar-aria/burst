#pragma once
#include <util/delay.h>

#include <functional>
#include <numeric>

#include "burst.h"
#include "dev/mux.h"
#include "util/misc.h"

namespace befaco::burst {

class Display {
  constexpr static uint16_t PARAMETER_TIMEOUT = 750;
  constexpr static uint16_t QUANTITY_TIMEOUT = 350;

  struct View {
    uint8_t led = 0;
    uint32_t end_time = 0;

    constexpr static View Empty() { return View{}; }

    constexpr static View Timed(uint8_t led, uint32_t end_time) { return {.led = led, .end_time = end_time}; }

    constexpr static View Persistent(uint8_t led) { return {.led = led, .end_time = 0xFFFFFFFF}; }
  };

 public:
  enum class Mode : uint8_t {
    BURST,
    QUANTITY,
    PARAMETER,
  };

 private:
  // In hardware, the Burst's display is simply a mux (with an extra selector pin being a transistor)
  const Mux<4> mux;

  std::array<View, 3> views = {View::Empty(), View::Persistent(0), View::Empty()};
  Mode mode_ = Mode::QUANTITY;

 public:
  Display(PinGroup<4> pins) : mux{pins} {};

  void Init() const { mux.Init(); }

  void set_mode(Mode mode) { mode_ = mode; };

  // void set(uint8_t led) { leds.Write(led); }
  void Set(Mode mode, View view) { views[std::to_underlying(mode)] = view; }

  void SetBurst(uint8_t led) {
    Set(Mode::BURST, View::Persistent(led));
  }

  void ClearBurst() { Set(Mode::BURST, View::Empty()); }

  void SetQuantity(uint8_t led, uint32_t time_now) {
    Set(Mode::QUANTITY, View::Timed(led, time_now + QUANTITY_TIMEOUT));
    mode_ = Mode::QUANTITY;
  }

  void SetParameter(uint8_t led, uint32_t time_now) {
    Set(Mode::PARAMETER, View::Timed(led, time_now + PARAMETER_TIMEOUT));
    mode_ = Mode::PARAMETER;
  }

  void Show(uint32_t time_now) {
    auto mode_idx = std::to_underlying(mode_);
    if (time_now > views[mode_idx].end_time) {
      if (mode_ == Mode::QUANTITY || mode_ == Mode::PARAMETER) { // timeout from quantity and parameter into burst
        mode_ = Mode::BURST;
      } 
      if (views.at(std::to_underlying(Mode::BURST)).end_time == 0x00) { // if burst is empty, change to quantity
        mode_ = Mode::QUANTITY;
      }
      mode_idx = std::to_underlying(mode_);
    }
    mux.Select(views.at(std::to_underlying(mode_)).led);
  }

  void display_multi(std::function<bool(uint8_t)> cond) {
    for (uint8_t i = 0; i < 16; i++) {
      if (cond(i)) {
        mux.Select(i);
      }
    }
  }

  void display_multi(uint16_t view) {
    for (uint16_t i = 0; i < 16; i++) {
      if (bit::Read(view, i)) {
        mux.Select(i);
      }
    }
  }

  void Dance(const uint16_t duration = 0x3FF) {
    for (size_t i = duration; i > 0; i--) {
      display_multi([](uint8_t idx) { return idx % 2; });
    }
    for (size_t i = duration; i > 0; i--) {
      display_multi([](uint8_t idx) { return idx % 2 == 0; });
    }
  }

  void Flourish(uint8_t time_on = 10) {
    for (uint8_t led = 0; led < 16; led++) {
      mux.Select(led);
      _delay_ms(time_on);
    }
  }

  void LightShow() {
    for (int16_t led = 15; led >= 0; led--) {
      for (int16_t pin = 4; pin >= 0; pin--) {
        mux.selector().pins[pin].Write(bit::Read(led, pin));
        _delay_ms(2);
      }
      _delay_ms(10);
    }

    for (int16_t led = 0; led < 16; led++) {
      for (int16_t pin = 4; pin >= 0; pin--) {
        mux.selector().pins[pin].Write(bit::Read(led, pin));
        _delay_ms(2);
      }
      _delay_ms(10);
    }
  }
};
}  // namespace befaco::burst