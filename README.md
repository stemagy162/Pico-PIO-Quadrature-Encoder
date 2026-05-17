# Pico PIO Quadrature Encoder (Arduino)

A high-performance, zero-CPU-overhead quadrature encoder library for the Raspberry Pi Pico (RP2040/RP2350) using the hardware PIO (Programmable I/O) subsystem. 

This library wraps complex PIO register operations into a dead-simple Arduino class with built-in safety guards, making it perfect for both rock-solid industrial robotics and educational use.

## Features

- **Zero CPU Overhead:** Pulse counting is completely offloaded to the Pico's dedicated PIO hardware. Never miss a single step, even at ultra-high RPMs.
- **Dead Simple API:** Initialize and read your quadrature encoder with just a few lines of standard Arduino code.
- **Built-in Safety Guards:** Automatically detects uninitialized Serial communications or incorrect pin configurations to prevent silent failures during debugging.

## Quick Start

### Hardware Setup
Connect your rotary encoder's Phase A and Phase B to any two **physically adjacent** GPIO pins on your Pico (e.g., GPIO 1 and GPIO 2).

### Software Example

```cpp
#include <Arduino.h>

#include "hardware/pio.h"
#include "quadrature.pio.h"
#include <encoder.hpp>

Encoder encoder;
static const uint8_t pinA = 1;
static const uint8_t pinB = pinA + 1;


void setup() {
    Serial.begin(115200); // Start Serial before Encoder::setup()
    delay(4000);
    Serial.println("Setup...");

    /* encoder */
    encoder.setup(pinA, pinB, pio0); // 3rd argument is pio0 or pio1
}

void loop() {
    Serial.println( encoder.getValue() );
    encoder.update();
    delay(10);
}
```

## Reference

### Encoder PIO
Repository : [jamon / pi-pico-pio-quadrature-encoder](https://github.com/jamon/pi-pico-pio-quadrature-encoder.git)

Copyright (c) 2022 Jamon Terrell <github@jamonterrell.com>

Copyright (c) 2023 Arda Alıcı     <ardayaozel@hotmail.com>

License: [MIT](https://en.wikipedia.org/wiki/MIT_License)