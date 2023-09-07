#pragma once
#include "sys/pin_group.h"

namespace befaco::burst {
  ///////////////// PIN DEFINITIONS

namespace pins {
////// ANALOG INS
constexpr Pin CV_DIVISIONS = Pin(Port::C, 0);
constexpr Pin CV_PROBABILITY = Pin(Port::C, 1);
constexpr Pin CV_REPETITIONS = Pin(Port::C, 2);
constexpr Pin CV_DISTRIBUTION = Pin(Port::C, 3);

////// DIGITAL INS
constexpr Pin CYCLE_SWITCH = Pin(Port::C, 4);

constexpr Pin PING_STATE = Pin(Port::D, 2);
constexpr Pin ENCODER_BUTTON = Pin(Port::D, 3);;
constexpr Pin ENCODER_A = Pin(Port::D, 5);
constexpr Pin ENCODER_B = Pin(Port::D, 4);
constexpr Pin TRIGGER_STATE = Pin(Port::B, 0);
constexpr Pin TRIGGER_BUTTON = Pin(Port::B, 3);

////// DIGITAL OUTS
constexpr Pin TEMPO_STATE = Pin(Port::D, 0);
constexpr Pin OUT_LED = Pin(Port::D, 1);
constexpr Pin OUT_STATE = Pin(Port::D, 7);
constexpr Pin EOC_STATE = Pin(Port::B, 1);

constexpr PinGroup<4> LED_MUX = {Pin(Port::B, 4), Pin(Port::B, 5), Pin(Port::B, 2), Pin(Port::D, 6)};
}  // namespace pins
}