#pragma once
#include "../sys/pin.h"

namespace uno {
// PORTD
constexpr Pin D0 = Pin(Port::D, 0);
constexpr Pin D1 = Pin(Port::D, 1);
constexpr Pin D2 = Pin(Port::D, 2);
constexpr Pin D3 = Pin(Port::D, 3);
constexpr Pin D4 = Pin(Port::D, 4);
constexpr Pin D5 = Pin(Port::D, 5);
constexpr Pin D6 = Pin(Port::D, 6);
constexpr Pin D7 = Pin(Port::D, 7);

// PORTB
constexpr Pin D8 = Pin(Port::B, 0);
constexpr Pin D9 = Pin(Port::B, 1);
constexpr Pin D10 = Pin(Port::B, 2);
constexpr Pin D11 = Pin(Port::B, 3);
constexpr Pin D12 = Pin(Port::B, 4);
constexpr Pin D13 = Pin(Port::B, 5);

// PORTC
constexpr Pin D14 = Pin(Port::C, 0);
constexpr Pin D15 = Pin(Port::C, 1);
constexpr Pin D16 = Pin(Port::C, 2);
constexpr Pin D17 = Pin(Port::C, 3);
constexpr Pin D18 = Pin(Port::C, 4);
constexpr Pin D19 = Pin(Port::C, 5);
constexpr Pin D20 = Pin(Port::C, 6);

// Analog
constexpr Pin A0 = Pin(Port::C, 0);
constexpr Pin A1 = Pin(Port::C, 1);
constexpr Pin A2 = Pin(Port::C, 2);
constexpr Pin A3 = Pin(Port::C, 3);
constexpr Pin A4 = Pin(Port::C, 4);
constexpr Pin A5 = Pin(Port::C, 5);

namespace spi {
constexpr Pin SS = D10;
constexpr Pin MOSI = D11;
constexpr Pin MISO = D12;
constexpr Pin SCK = D13;
}  // namespace spi

constexpr Pin SS = spi::SS;
constexpr Pin MOSI = spi::MOSI;
constexpr Pin MISO = spi::MISO;
constexpr Pin SCK = spi::SCK;

namespace twi {
constexpr Pin SDA = D18;
constexpr Pin SCL = D19;
}  // namespace twi

constexpr Pin SDA = twi::SDA;
constexpr Pin SCL = twi::SCL;

constexpr Pin LED_BUILTIN = D13;

}  // namespace uno
