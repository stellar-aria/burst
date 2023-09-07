#pragma once

// THIS IS FROM WIRING.C
// LGPL license

namespace wiring {

#include <avr/interrupt.h>
// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
constexpr auto MICROSECONDS_PER_TIMER0_OVERFLOW = (64 * 256) / (F_CPU / 1000000L);

// the whole number of milliseconds per timer0 overflow
constexpr auto MILLIS_INC = MICROSECONDS_PER_TIMER0_OVERFLOW / 1000;

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
constexpr auto FRACT_INC = (MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3;
constexpr auto FRACT_MAX = 1000 >> 3;

volatile uint32_t timer0_overflow_count = 0;
volatile uint32_t timer0_millis = 0;
static uint8_t timer0_fract = 0;

ISR(TIMER0_OVF_vect) {
  uint32_t m = timer0_millis;
  uint8_t f = timer0_fract;

  m += MILLIS_INC;
  f += FRACT_INC;
  if (f >= FRACT_MAX) {
    f -= FRACT_MAX;
    m += 1;
  }

  timer0_fract = f;
  timer0_millis = m;
  uint32_t t0oc = timer0_overflow_count;
  timer0_overflow_count = t0oc + 1;
}

uint32_t millis() {
  uint8_t oldSREG = SREG;

  cli();

  uint32_t m = timer0_millis;
  SREG = oldSREG;

  return m;
}

void init() {
  // this needs to be called first in main() or some functions won't
  // work there
  sei();

  // on the ATmega168, timer 0 is also used for fast hardware pwm
  // (using phase-correct PWM would mean that timer 0 overflowed half as often
  // resulting in different millis() behavior on the ATmega8 and ATmega168)
  TCCR0A = TCCR0A | _BV(WGM01) | _BV(WGM00);

  // set timer 0 prescale factor to 64
  // this combination is for the standard 168/328/1280/2560
  TCCR0B = TCCR0B | _BV(CS01) | _BV(CS00);

  // enable timer 0 overflow interrupt
  TIMSK0 = TIMSK0 | _BV(TOIE0);

  // timers 1 and 2 are used for phase-correct hardware pwm
  // this is better for motors as it ensures an even waveform
  // note, however, that fast pwm mode can achieve a frequency of up
  // 8 MHz (with a 16 MHz clock) at 50% duty cycle

  // set timer 1 prescale factor to 64
  TCCR1B = _BV(CS11) | _BV(CS10);

  // put timer 1 in 8-bit phase correct pwm mode
  TCCR1A = TCCR1A | _BV(WGM10);

  // set timer 2 prescale factor to 64
  TCCR2B = TCCR2B | _BV(CS22);

  // configure timer 2 for phase correct pwm (8-bit)
  TCCR2A = TCCR2A | _BV(WGM20);

  // set a2d prescaler so we are inside the desired 50-200 KHz range
  // and enable a2d conversions
  ADCSRA = _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADEN);

  // the bootloader connects pins 0 and 1 to the USART; disconnect them
  // here so they can be used as normal digital i/o; they will be
  // reconnected in Serial.begin()
  UCSR0B = 0;
}
}  // namespace wiring