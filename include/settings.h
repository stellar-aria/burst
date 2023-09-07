#pragma once
#include "sys/EEPROM.h"

enum class DistributionSign { NEGATIVE = 0, POSITIVE = 1, ZERO = 2 };

struct CalibrationData {
  uint16_t repetitions = 511;
  uint16_t distribution = 511;
  uint16_t divisions = 511;
  uint16_t probability = 511;

  void Validate() {
    Validate(distribution);
    Validate(repetitions);
    Validate(divisions);
    Validate(probability);
  }

 private:
  void Validate(uint16_t &value) {
    if (value == 0x0000 || value == 0xFFFF) {
      value = 511;
    }
  }
};

struct EepromMap {
  enum EEPROM_MAP : size_t {
    MASTER_CLOCK = 0,
    REPETITIONS = 4,
    DISABLE_FIRST_CLOCK = 5,
    CALIBRATION_DATA = 6,
    RETRIGGER = 14,
    PROBABILITY_AFFECTS_EOC = 15,
    FIRST_RUN = 16
  };
};

struct Settings {
  uint16_t master_clock = 120;
  uint8_t repetitions = 1;
  bool disable_first_clock = true;
  CalibrationData calibration_data;
  bool retrigger = false;
  bool probability_affects_eoc = false;
  bool first_run = false;

  void Load() {
    EEPROM.get(0, *this);
  }

  void Save() {
    EEPROM.put(0, *this);
  }

  void WriteCalibrationData() {
    EEPROM.put(EepromMap::CALIBRATION_DATA, this->calibration_data);
  }
};

struct State {
  //*** tempo and counter **

  /// the master clock is the result of averaged time of ping input or encoder
  /// button
  uint32_t masterClock = 0;

  /// the result of div/mult the master clock depending on the div/mult
  /// potentiometer/input
  uint32_t clockDivided = 0;

  /// the linear portions of time depending on clock divided and number of
  /// repetitions in the burst. if the distribution is linear this will
  /// give us the duration of every repetition.
  /// if it is not linear it will be used to calculate the distribution
  float timePortions = 0;

  // Divisions
  //// value of the time division or the time multiplier
  int16_t divisions = 1;

  // Repetitions
  uint8_t repetitions = 1;  /// number of repetitions
  uint8_t repetitionsEncoder = 1;

  // Random
  int16_t randomPot = 0;

  /// Distribution
  float distribution = 0;
  DistributionSign distributionSign = DistributionSign::POSITIVE;

  uint32_t tempoTic = 0;  // the PREVIOUS tempo tick
  uint32_t tempoTimer = 0;

  State() = default;
};