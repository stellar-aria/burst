/************************************
   BURST 1.1 - BEFACO
   written by Eloi Flores and Jeremy Bernstein

   NOTES, KNOWN BUGS and IMPROVEMENTS :

   When cycle on, no trigger, and external ping. There is a process to resync
   the burst with the ping, because if not the burst is not in phase with the
   ping. This is done taking the difference time between the cycle and the
   moment when the trigger button has been pressed. A proportion is calculated
   when the trigger button is pressed to provide the position of the trigger in
   relation with the time window. This calculation uses float, known as the evil
   datatype in terms of processing consume. Another way to do that?

   The resync doesn't work properly with the combination of :
   - Distribution   o     o    o   o  o oo
   - Small amount of state.repetitions (depending on tempo but 8 7 or less; as
   lower tempo, the amount of state.repetitions where we have de-sync is higher)

   Testing the module we noticed that an attenuator in the quantity input could
   be very useful. One solution could be to use [encoder + encoder button hold]
   to adjust this attenuationof quantity CV in 32 steps.


   At high frequencies there is a gap at the end of the burst
 */

///////////////// LIBRARIES
#include <stdint.h>
#define __cpp_rtti 0

#include "burst.h"

#include <random>

#include "digital_input.h"
#include "calibrated_cv_input.h"
#include "display.h"
#include "hid/encoder.hpp"
#include "hid/timed_switch.hpp"
#include "settings.h"
#include "sys/arduino_compat.h"
#include "util/xoroshiro32plusplus.hpp"

using namespace befaco::burst;

/// Global software settings
static State state{};
static State new_state{};
static Settings settings;

////// HID

Encoder<TimedSwitch> encoder;
TimedSwitch trigger_button;
TimedSwitch cycle_switch;
Display display(pins::LED_MUX);

///// CV INPUTS

CalibratedCVInput cv_divisions{pins::CV_DIVISIONS, settings.calibration_data.divisions, -4, 0, 4};
CalibratedCVInput cv_repetitions{pins::CV_REPETITIONS, settings.calibration_data.repetitions, 32, 0, -16};
CalibratedCVInput cv_probability{pins::CV_PROBABILITY, settings.calibration_data.probability, 0, 10, 20};
CalibratedCVInput cv_distribution{pins::CV_DISTRIBUTION, settings.calibration_data.distribution, -8, 0, 8};

TriggerInput cv_trigger(pins::TRIGGER_STATE, true);
ClockInput cv_ping(pins::PING_STATE, true);

/// RANDOM DEVICE
Xoroshiro32PlusPlus xoroshiro;

////// VARIABLES

uint32_t firstBurstTime = 0;  /// the moment when the burst start
uint32_t burstTimeStart = 0;  /// the moment when the burst start
uint32_t burstTimeAccu = 0;   /// the accumulation of all state.repetitions times
                              /// since the burst have started

/// the time when the previous repetition pulse have raised
uint32_t repetitionRaiseStart = 0;

/// the current repetition number since the burst has started
uint8_t repetitionCounter = 0;

/// the difference between previous repetition time and current
/// repetition time. the difference between one repetition and the next
/// one
uint32_t elapsedTimeSincePrevRepetition = 0;

/// the position of the new repetition inside the time window
uint32_t elapsedTimeSincePrevRepetitionNew = 0;

/// the position of the previous repetition inside the time window
uint32_t elapsedTimeSincePrevRepetitionOld = 0;

bool inEoc = false;
bool wantsEoc = false;

/// a counter to turn off the eoc led and the eoc outputn
uint32_t eocCounter = 0;

// divisions
int16_t divisionCounter = 0;

////// Repetitions
constexpr uint8_t MIN_REPETITIONS = 1;
constexpr uint8_t MAX_REPETITIONS = 32;  /// max number of repetitions

///// Distribution

/// the curve we apply to the  pow function to calculate the distribution
/// of repetitions
constexpr uint8_t curve = 5;

consteval std::array<float, 9> createDistributionIndex() {
  std::array<float, 9> out;
  for (uint8_t i = 0; i <= 8; i++) {
    // invert and scale in advance: positive numbers give more weight to high
    // end on output convert linear scale into logarithmic exponent for pow
    // function in fscale()
    float val = map<float>(i, 0, 8, 0, curve);
    val = std::clamp(val, -10.f, 10.f);
    val *= -0.1;
    val = std::pow(10, val);
    out[i] = val;
  }
  return out;
}

/// used to calculate the position of the state.repetitions
constexpr std::array<float, 9> distributionIndexArray = createDistributionIndex();

//// Trigger
TriggerSignal trigger{};  /// the result of both trigger button and trigger input

//// Cycle
bool cycle = false;

bool resetPhase = false;

/// ping
bool ping_read = false;
bool ping_processed = false;

/// output
bool outputState = false;

// recycle = start a new burst within a set of bursts (mult)
bool recycle = false;

// resync = start a new burst, but ensure that we're correctly phased
bool resync = false;

//// encoder button and tap tempo
bool encoder_pressed = false;
bool encoder_processed = false;

uint8_t silentBurst = false;

bool wantsMoreBursts = false;

constexpr uint8_t TAP_TEMPO_AVG_COUNT = 2;

// 10ms is a good length to be picked up by other modules
constexpr uint8_t TRIGGER_LENGTH = 10;

uint16_t trigLen = TRIGGER_LENGTH;
uint32_t encoderLastTime = 0;
uint32_t encoderTaps[TAP_TEMPO_AVG_COUNT] = {0, 0};
uint32_t encoderDuration = 0;
uint8_t encoderTapsCurrent = 0;

// 2 means use the duration between encoderTaps[0] and encoderTaps[1];
// 3 means average of [0-2]
uint8_t encoderTapsTotal = 0;

uint32_t pingLastTime = 0;
uint32_t pingDuration = 0;

/// flags
bool burstStarted = false;  // if the burst is active or not

void doLedFlourish(int16_t num_times) {
  while (num_times--) {
    display.Flourish();
    _delay_ms(20);
  }
}

void doLightShow(int16_t num_times = 3) {
  while (num_times--) {
    display.LightShow();
    _delay_ms(20);
  }
}

void handleInputs(uint32_t now) {
  trigger_button.Debounce(!pins::TRIGGER_BUTTON.Read(), now);

  encoder.Process(!pins::ENCODER_A.Read(), !pins::ENCODER_B.Read());
  encoder.sw.Debounce(!pins::ENCODER_BUTTON.Read(), now);

  cv_distribution.Read();
  cv_divisions.Read();
  cv_probability.Read();
  cv_repetitions.Read();

  cv_trigger.Read();
  //cv_ping.Read();
}

void Calibrate() {
  // repetitions input is our zero
  uint16_t zero = cv_repetitions.Read();
  settings.calibration_data = {
      .repetitions = zero,
      .distribution = zero,
      .divisions = zero,
      .probability = zero,
  };
  settings.WriteCalibrationData();
  doLedFlourish(3);
}

bool checkCalibrationMode() {
  const bool trigger_pressed = !pins::TRIGGER_BUTTON.Read();
  const bool encoder_pressed = !pins::ENCODER_BUTTON.Read();

  if (encoder_pressed && trigger_pressed) {
    Calibrate();
    return true;
  } else {
    settings.calibration_data.Validate();
    return false;
  }
}

void calcTimePortions() {
  if (new_state.masterClock < 1) new_state.masterClock = 1;  // ensure state.masterClock > 0

  if (state.divisions > 0) {
    new_state.clockDivided = new_state.masterClock * state.divisions;
  } else if (state.divisions < 0) {
    new_state.clockDivided = new_state.masterClock / (-state.divisions);
  } else if (state.divisions == 0) {
    new_state.clockDivided = new_state.masterClock;
  }
  new_state.timePortions = (float)new_state.clockDivided / (float)new_state.repetitions;
}

void calculateClock(uint32_t now) {
  ////  Read the encoder button state
  encoder_pressed = encoder.sw.pressed();

  /// Read the ping input state
  ping_read = cv_ping.rising();

  // Edit Settings
  if (encoder_pressed && encoder_processed) {
    if (encoder.sw.pressed_for(5000, now)) {  // held longer than 5s
      if (trigger_button.pressed_for(5000, now)) {
        settings.probability_affects_eoc = !settings.probability_affects_eoc;
      } else {
        settings.retrigger = !settings.retrigger;
      }
      doLedFlourish(2);
      encoderLastTime = 0;
      encoderTapsCurrent = 0;
      encoderTapsTotal = 0;
      encoderDuration = 0;

      encoder.sw.ResetCounter(now);
    }
    return;
  }

  bool ignore = false;
  uint32_t duration;
  uint32_t average;

  if (encoder_pressed && !encoder_processed) {
    duration = (now - encoderLastTime);

    // this logic is weird, but it works:
    // if we've stored a duration, test the current duration against the
    // stored
    //    if that's an outlier, reset everything, incl. the duration, to 0 and
    //    ignore. otherwise, store the duration
    // if we haven't stored a duration, check whether there's a previous time
    // (if there isn't we just started up, I guess). And if there's no
    // previous time store the previous time and ignore. If there is a
    // previous time, store the duration. Basically, this ensures that
    // outliers start a new tap tempo collection, and that the first incoming
    // taps are properly handled.
    if ((encoderDuration && (duration > (encoderDuration * 2) || (duration < (encoderDuration >> 1)))) ||
        (!(encoderDuration || encoderLastTime))) {
      ignore = true;
      encoderTapsCurrent = 0;
      encoderTapsTotal = 0;  // reset collection, we had an outlier
      encoderDuration = 0;
    } else {
      encoderDuration = duration;
    }
    encoderLastTime = now;

    if (!ignore) {
      new_state.tempoTic = now;
      encoderTaps[encoderTapsCurrent++] = duration;
      if (encoderTapsCurrent >= TAP_TEMPO_AVG_COUNT) {
        encoderTapsCurrent = 0;
      }
      encoderTapsTotal++;
      if (encoderTapsTotal > TAP_TEMPO_AVG_COUNT) {
        encoderTapsTotal = TAP_TEMPO_AVG_COUNT;
      }
      if (encoderTapsTotal > 1) {  // currently this is 2, but it could change
        average = 0;
        for (uint8_t i = 0; i < encoderTapsTotal; i++) {
          average += encoderTaps[i];
        }
        new_state.masterClock = (average / encoderTapsTotal);
      } else {
        new_state.masterClock = encoderTaps[0];  // should be encoderDuration
      }
      calcTimePortions();
    }

    // we could do something like -- tapping the encoder will immediately
    // enable the encoder's last tempo if it has been overridden by the ping
    // input. So you could tap a fast tempo, override it with a slow ping,
    // pull the ping cable and then tap the encoder to switch back. Similarly,
    // sending a single ping clock would enable the last ping in tempo. In
    // this way, it would be possible to switch between two tempi. Not sure if
    // this is a YAGNI feature, but it would be possible.

    encoder_processed = true;
    // TODO: Save master_clock here
  }

  if (ping_read && !ping_processed) {
    duration = (now - pingLastTime);

    if ((pingDuration && (duration > (pingDuration * 2) || (duration < (pingDuration >> 1)))) ||
        (!(pingDuration || pingLastTime))) {
      ignore = true;
      pingDuration = 0;
    } else {
      pingDuration = duration;
    }
    pingLastTime = now;

    if (!ignore) {
      new_state.tempoTic = now;
      new_state.masterClock = pingDuration;
      calcTimePortions();
    }
    ping_processed = true;
  }

  if (encoder_processed && !encoder_pressed) {
    encoder_processed = false;
  }
  if (ping_processed && !ping_read) {
    ping_processed = false;
  }
}

void readTrigger(uint32_t now) {
  if (trigger_button.pressed()) {
    if (!(encoder_pressed || encoder_processed) && trigger_button.pressed_for(5000, now)) {
      // toggle initial ping output
      settings.disable_first_clock = !settings.disable_first_clock;
      trigger_button.ResetCounter(now);
      doLedFlourish(2);
    }
  }

  trigger = trigger_button.pressed() || cv_trigger.high();

  if (trigger && burstTimeStart != 0 && !settings.retrigger) {
    trigger.Reset();
  }
}

int32_t fscale(int32_t originalMin, int32_t originalMax, int32_t newBegin, int32_t newEnd, int32_t inputValue,
               float curve) {
  int32_t originalRange = 0;
  int32_t newRange = 0;
  int32_t zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  int32_t rangedValue = 0;
  bool invFlag = 0;

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  originalRange = originalMax - originalMin;

  if (newEnd > newBegin) {
    newRange = newEnd - newBegin;
  } else {
    newRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;

  // normalize to 0 - 1 float
  normalizedCurVal = ((float)zeroRefCurVal / (float)originalRange);

  // Check for originalMin > originalMax  - the math for all other cases i.e.
  // negative numbers seems to work out fine
  if (originalMin > originalMax) {
    return 0;
  }

  double powr = pow(normalizedCurVal, curve) * newRange;

  if (invFlag == 0) {
    rangedValue = (int32_t)(powr + newBegin);
  } else {  // invert the ranges
    rangedValue = (int32_t)(newBegin - powr);
  }

  return rangedValue;
}

void startBurstInit(uint32_t now) {
  burstTimeStart = now;
  burstStarted = true;
  recycle = false;

  repetitionCounter = 0;
  elapsedTimeSincePrevRepetitionOld = 0;
  burstTimeAccu = 0;

  // enable the output so that we continue to count the state.repetitions,
  // whether we actually write to the output pin or not
  outputState = 0x01;

  std::uniform_int_distribution rand_dist(0, 100);
  int16_t randomDif = state.randomPot - rand_dist(xoroshiro);
  bool triggerOverride = trigger_button.pressed();

  // trigger button overrides probability (unless disableFirstClock is enabled)
  silentBurst = triggerOverride ? 0 : (randomDif <= 0);
  wantsMoreBursts = triggerOverride || (!silentBurst) || wantsMoreBursts;

  bool firstTriggerOut = !settings.disable_first_clock;

  pins::OUT_STATE.Write(!firstTriggerOut);
  pins::OUT_LED.Write(firstTriggerOut);

  switch (state.distributionSign) {
    case DistributionSign::POSITIVE:
      elapsedTimeSincePrevRepetitionNew =
          fscale(0, state.clockDivided, 0, state.clockDivided, state.timePortions, state.distribution);
      elapsedTimeSincePrevRepetition = elapsedTimeSincePrevRepetitionNew;
      break;
    case DistributionSign::NEGATIVE:
      if (state.repetitions > 1) {
        elapsedTimeSincePrevRepetitionOld = fscale(0, state.clockDivided, 0, state.clockDivided,
                                                   round(state.timePortions * state.repetitions), state.distribution);
        elapsedTimeSincePrevRepetitionNew =
            fscale(0, state.clockDivided, 0, state.clockDivided, round(state.timePortions * (state.repetitions - 1)),
                   state.distribution);
        elapsedTimeSincePrevRepetition = elapsedTimeSincePrevRepetitionOld - elapsedTimeSincePrevRepetitionNew;
      } else {
        elapsedTimeSincePrevRepetitionNew = state.timePortions;
        elapsedTimeSincePrevRepetition = state.timePortions;
      }
      break;
    case DistributionSign::ZERO:
      elapsedTimeSincePrevRepetitionNew = state.timePortions;
      elapsedTimeSincePrevRepetition = state.timePortions;
      break;
  }
}

//// read divisions
void readDivision(uint32_t now) {
  int16_t divisionsVal = cv_divisions.Scaled();

  if (divisionsVal > 0) {
    divisionsVal += 1;
  } else if (divisionsVal < 0) {
    divisionsVal -= 1;
  } else {
    // skip 1/-1
  }

  if (cv_divisions.changed()) {
    uint8_t bitDivisions = 0;
    if (divisionsVal < 0) {
      bitDivisions = -(divisionsVal)-1;
    } else if (divisionsVal > 0) {
      bitDivisions = (16 - divisionsVal) + 1;
    }

    display.SetParameter(bitDivisions, now);

    new_state.divisions = divisionsVal;
  }
}

void readRepetitions(uint32_t now) {
  const uint8_t repetitions_prev = new_state.repetitions;

  // only change if we're greater than 1 or we're incrementing
  if (new_state.repetitionsEncoder > 1 || encoder.increment() > 0) {
    new_state.repetitionsEncoder += encoder.increment();
  }

  new_state.repetitionsEncoder = std::clamp(new_state.repetitionsEncoder, MIN_REPETITIONS, MAX_REPETITIONS);

  // Read input
  int16_t encoder_and_cv = new_state.repetitionsEncoder + cv_repetitions.Scaled();
  new_state.repetitions = std::clamp<int16_t>(encoder_and_cv, MIN_REPETITIONS, MAX_REPETITIONS);

  if (new_state.repetitions != repetitions_prev) {
    display.SetQuantity(new_state.repetitions - 1, now);
  }
}

void readRandom(uint32_t now) {
  int16_t randomVal = cv_probability.Scaled();
  randomVal *= 5;

  if (randomVal != new_state.randomPot) {
    uint8_t bitRandom = map<uint8_t>(randomVal, 0, 100, 15, 0);
    display.SetParameter(bitRandom, now);
    new_state.randomPot = randomVal;
  }
}

constexpr DistributionSign getDistributionSign(int val) {
  using enum DistributionSign;
  if (val > 0) {
    return POSITIVE;
  } else if (val < 0) {
    return NEGATIVE;
  } else {
    return ZERO;
  }
}

void readDistribution(uint32_t now) {
  int16_t distributionVal = cv_distribution.Scaled();

  float dist = distributionIndexArray[abs(distributionVal)];
  DistributionSign distSign = getDistributionSign(distributionVal);

  if (cv_distribution.changed() || dist != new_state.distribution) {
    uint8_t bitDistribution = 0;
    if (distributionVal < 0) {
      bitDistribution = std::abs(distributionVal);
    } else if (distributionVal > 0) {
      bitDistribution = (16 - distributionVal);
    }

    display.SetParameter(bitDistribution, now);

    new_state.distribution = dist;
    new_state.distributionSign = distSign;
  }
}

bool readCycle() {
  cycle_switch.Debounce(!pins::CYCLE_SWITCH.Read());

  if (cycle_switch.just_pressed() && trigger_button.pressed()) {
    resetPhase = true;
    doLedFlourish(2);
  }

  return cycle_switch.pressed();
}

void handleLEDs(uint32_t now) {
  if (burstStarted && !silentBurst) {
    display.SetBurst(repetitionCounter);
  } else {
    display.ClearBurst();
  }
  display.Show(now);
}

void handlePulseDown(uint32_t now) {
  if (outputState && burstStarted) {
    if (now >= (burstTimeStart + burstTimeAccu + trigLen)) {
      outputState = false;
      pins::OUT_STATE.Write(true);
      pins::OUT_LED.Write(false);
    }
  }
}

void handleEOC(uint32_t now, int16_t width) {
  if (inEoc && now >= eocCounter + width) {
    pins::EOC_STATE.Write(true);
    inEoc = false;
  }
}

void enableEOC(uint32_t now) {
  if (inEoc) {
    eocCounter = now;
    handleEOC(now, 0);  // turn off the EOC if necessary
  }

  if (!(silentBurst && settings.probability_affects_eoc)) {
    // turn it (back) on
    pins::EOC_STATE.Write(false);
    inEoc = true;
    eocCounter = now;
  }
  wantsEoc = false;
}

void handlePulseUp(uint32_t now) {
  int16_t inputValue;

  // pulse up - burst time
  if (!outputState && burstStarted) {
    if (now >= (burstTimeStart + elapsedTimeSincePrevRepetition + burstTimeAccu)) {  // time for a repetition
      if (repetitionCounter < state.repetitions - 1) {                               // is it the last repetition?
        outputState = 0x01;
        pins::OUT_STATE.Write(!wantsMoreBursts);
        pins::OUT_LED.Write(wantsMoreBursts);
        burstTimeAccu += elapsedTimeSincePrevRepetition;

        repetitionCounter++;

        switch (state.distributionSign) {
          case DistributionSign::POSITIVE:
            inputValue = round(state.timePortions * (repetitionCounter + 1));
            elapsedTimeSincePrevRepetitionOld = elapsedTimeSincePrevRepetitionNew;
            elapsedTimeSincePrevRepetitionNew =
                fscale(0, state.clockDivided, 0, state.clockDivided, inputValue, state.distribution);
            elapsedTimeSincePrevRepetition = elapsedTimeSincePrevRepetitionNew - elapsedTimeSincePrevRepetitionOld;
            break;
          case DistributionSign::NEGATIVE:
            elapsedTimeSincePrevRepetitionOld = elapsedTimeSincePrevRepetitionNew;

            inputValue = round(state.timePortions * ((state.repetitions - 1) - repetitionCounter));
            if (!inputValue && state.divisions <= 0) {  // no EOC if we're dividing
              wantsEoc = true;                          // we may not reach the else block below if the
                                                        // next trigger comes too quickly
            }
            elapsedTimeSincePrevRepetitionNew =
                fscale(0, state.clockDivided, 0, state.clockDivided, inputValue, state.distribution);
            elapsedTimeSincePrevRepetition = elapsedTimeSincePrevRepetitionOld - elapsedTimeSincePrevRepetitionNew;
            break;
          case DistributionSign::ZERO:
            elapsedTimeSincePrevRepetitionOld = elapsedTimeSincePrevRepetitionNew;
            elapsedTimeSincePrevRepetitionNew = round(state.timePortions * (repetitionCounter + 1));
            elapsedTimeSincePrevRepetition = elapsedTimeSincePrevRepetitionNew - elapsedTimeSincePrevRepetitionOld;
            break;
        }
      } else {  // it's the end of the burst, but not necessarily the end of a
                // set of bursts (mult)
        enableEOC(now);

        wantsMoreBursts = true;
        burstStarted = false;

        cycle = readCycle();

        if (cycle) {
          recycle = true;
          divisionCounter++;
          if (state.divisions >= 0 || divisionCounter >= -(state.divisions)) {
            resync = true;
            // superstitious, this will also be reset in doResync()
            divisionCounter = 0;
          }
        } else {
          wantsMoreBursts = false;
          burstTimeStart = 0;
          firstBurstTime = 0;
        }
      }
    }
  }
}

void handleTempo(uint32_t now) {
  static bool tempoInitialized = false;
  static uint32_t tempoStart = 0;

  if (!tempoInitialized) {
    state.tempoTimer = now;
    new_state.tempoTimer = now + state.masterClock;
    tempoInitialized = true;
  }

  // using a different variable here (state.tempoTimer) to avoid side effects
  // when updating state.tempoTic. Now the state.tempoTimer is entirely
  // independent of the cycling, etc.

  if (!tempoStart && (now >= state.tempoTimer) && (now < state.tempoTimer + trigLen)) {
    pins::TEMPO_STATE.Write(0x00);
    tempoStart = now;
  } else if (tempoStart && (now - tempoStart) > trigLen) {
    pins::TEMPO_STATE.Write(0x01);
    tempoStart = 0;
  }

  if (!tempoStart) {  // only if we're outside of the pulse
    if (now >= state.tempoTimer) {
      state.tempoTimer = new_state.tempoTimer;
    }
    while (now >= state.tempoTimer) {
      state.tempoTimer += state.masterClock;
    }
  }
}

void doResync(uint32_t now) {
  // get the new value in advance of calctimeportions();
  state.divisions = new_state.divisions;
  calcTimePortions();

  // don't advance the timer unless we've reached the next tick
  // (could happen with manual trigger)
  if (now >= new_state.tempoTic) {
    state.tempoTic = new_state.tempoTic ? new_state.tempoTic : now;
    if (new_state.masterClock != state.masterClock) {
      state.masterClock = new_state.masterClock;
    }
    new_state.tempoTic = state.tempoTic + state.masterClock;
    new_state.tempoTimer = new_state.tempoTic;
  }

  // update other params
  state.repetitions = new_state.repetitions;
  state.clockDivided = new_state.clockDivided;
  state.timePortions = new_state.timePortions;
  state.distribution = new_state.distribution;
  state.distributionSign = new_state.distributionSign;
  state.randomPot = new_state.randomPot;

  if (state.timePortions < TRIGGER_LENGTH) {
    trigLen = (int16_t)state.timePortions - 1;
    if (trigLen < 1) trigLen = 1;
  } else {
    trigLen = TRIGGER_LENGTH;
  }

  cycle = readCycle();
  resync = false;

  // otherwise multiple manual bursts will screw up the division count
  divisionCounter = 0;

  // time of first burst of a group of bursts
  firstBurstTime = now;
}

void HwInit() {
  wiring::init();

  //// remove to activate TEMPO_STATE and OUT_LED
#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("HOLA");
#endif

  xoroshiro.Seed(pins::CV_DISTRIBUTION.ReadAnalog<uint8_t>(), ~pins::CV_DIVISIONS.ReadAnalog<uint8_t>(),
                 pins::CV_PROBABILITY.ReadAnalog<uint8_t>(), ~pins::CV_REPETITIONS.ReadAnalog<uint8_t>());

  // CV inputs
  cv_probability.Init();
  cv_distribution.Init();
  cv_divisions.Init();
  cv_probability.Init();

  cv_ping.Init();
  cv_trigger.Init();

  // Encoder pins
  pins::ENCODER_A.Init(Pin::Mode::INPUT_PULLUP);
  pins::ENCODER_B.Init(Pin::Mode::INPUT_PULLUP);
  pins::ENCODER_BUTTON.Init(Pin::Mode::INPUT_PULLUP);

  // Switches
  pins::CYCLE_SWITCH.Init(Pin::Mode::INPUT_PULLUP);
  pins::TRIGGER_BUTTON.Init(Pin::Mode::INPUT_PULLUP);

  // Outputs
  pins::TEMPO_STATE.Init(Pin::Mode::OUTPUT);
  pins::TEMPO_STATE.Write(true);

  pins::OUT_LED.Init(Pin::Mode::OUTPUT);

  pins::OUT_STATE.Init(Pin::Mode::OUTPUT);
  pins::OUT_STATE.Write(true);

  pins::EOC_STATE.Init(Pin::Mode::OUTPUT);
  pins::EOC_STATE.Write(true);

  display.Init();
}

int main() {
  HwInit();

  settings.Load();
  if (settings.first_run) {
    settings = Settings();
    Calibrate();
    settings.Save();
  }

  state.masterClock = settings.master_clock;

  if (state.masterClock < 1) {
    state.masterClock = 120;
  }

  state.repetitions = std::clamp(settings.repetitions, MIN_REPETITIONS, MAX_REPETITIONS);

  new_state.masterClock = state.masterClock;
  new_state.repetitions = state.repetitions;
  new_state.repetitionsEncoder = state.repetitions;
  state.repetitionsEncoder = state.repetitions;

  if (!checkCalibrationMode()) {
    doLightShow();
  }

  cycle = readCycle();
  if (cycle) {
    trigger = true;
  }

  readDivision(0);
  calcTimePortions();

  while (true) {
    uint32_t currentTime = wiring::millis();

    uint32_t nextSyncPoint = firstBurstTime + ((state.divisions > 0) ? state.clockDivided : state.masterClock);

    if (burstTimeStart && firstBurstTime && cycle && currentTime >= nextSyncPoint) {  // force resync + tempo adjustment
      currentTime = nextSyncPoint;  // spoof the current time so that we always
                                    // remain in sync with the tempo
      trigger = true;
      wantsEoc = true;
    }

    /// we read the values and pots and inputs, and store the time difference
    /// between ping clock and trigger
    if (trigger.rising() || (resetPhase == true && currentTime >= new_state.tempoTic)) {
      if (wantsEoc) {
        enableEOC(currentTime);
      }
      repetitionCounter = 0;

      if (state.repetitionsEncoder != new_state.repetitionsEncoder) {
        state.repetitionsEncoder = new_state.repetitionsEncoder;
        //EEPROM.write(4, state.repetitionsEncoder);
      }

      doResync(currentTime);
      wantsMoreBursts = true;
      startBurstInit(currentTime);
      resetPhase = false;
    }

    handleInputs(currentTime);

    // we read the ping in and the encoder button to get :
    // master clock, clock divided and state.timePortions
    calculateClock(currentTime);

    // we read the trigger input and the trigger button to see if it is high or
    // low, if it is high and it is the first time it is.
    readTrigger(currentTime);

    // we read the number of repetitions in the encoder, we have to attend
    // this process often to avoid missing encoder tics.
    readRepetitions(currentTime);

    readDivision(currentTime);
    readDistribution(currentTime);
    readRandom(currentTime);

    handleEOC(currentTime, 30);

    handleLEDs(currentTime);

    // do this before cycle resync
    handlePulseDown(currentTime);
    handlePulseUp(currentTime);

    if (burstTimeStart && cycle) {  // CYCLE ON
      if (recycle
          // ensure that the current cycle is really over
          && (currentTime >= (burstTimeStart + state.clockDivided))
          // ensure that the total burst (incl mult) is really over
          && (!resync || currentTime >= (firstBurstTime + state.masterClock))) {
        if (resync) {
          if (state.repetitionsEncoder != new_state.repetitionsEncoder) {
            state.repetitionsEncoder = new_state.repetitionsEncoder;
            //EEPROM.write(4, state.repetitionsEncoder);
          }
          doResync(currentTime);
        }
        startBurstInit(currentTime);
      }
    } else if (!burstTimeStart || !cycle) {
      // ensure that the timer advances
      if (currentTime >= new_state.tempoTic) {
        doResync(currentTime);
      }
    }
    handleTempo(currentTime);
  }
}
