/*
 * 
 * Copyright 2026 Stemagy <stemagy.tech@gmail.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR
 * A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
 * AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * 
*/

#pragma once

#include <iostream>
#include <exception>
#include "hardware/pio.h"
#include "quadrature.pio.h"

/*
### Overview:
Internal data structure to store GPIO configurations for the encoder.
The assigned pins must be physically adjacent on the board (e.g., A and B phases next to each other).
### type
```cpp
struct Pin { 
    uint8_t A;
    uint8_t B;
};

```

*/
struct Pin {
uint8_t A = UINT8_MAX;
uint8_t B = UINT8_MAX;
};

/*

### Overview:

Internal data structure to track encoder steps and delta calculations.
Handles overflow wrapping smoothly by comparing current and previous raw counts.

### type

```cpp
struct Value {
    uint32_t last;
    uint32_t cur;
    int32_t diff;
};

```

*/
struct Value {
uint32_t last = 0;
uint32_t cur = 0;
int32_t diff = 0;
};

/*

### Overview:

High-performance rotary encoder reader class utilizing RP2040's PIO hardware.
It completely offloads the pulse counting process from the CPU, ensuring zero missed steps.

### Example

```cpp
Encoder encoder;
void setup() {
    // Pass Pin A, Pin B, and PIO instance directly into setup()
    encoder.setup(16, 17, pio0);
}
void loop() {
    encoder.update();
    int angle = encoder.getValue();
    delay(10);
}

```

*/
class Encoder {
    public:

        /*
        ### Overview:
        Initialize the PIO hardware, claim an unused state machine, and load the quadrature program.
        Includes safety guards for unconfigured pins and uninitialized serial communication.
        ### Arguments
        - `pinA` (uint8_t): GPIO number for Phase A.
        - `pinB` (uint8_t): GPIO number for Phase B (Must be adjacent to Pin A).
        - `pio` (PIO): The PIO instance pointer (`pio0` or `pio1`).
        */
        void setup(uint8_t pinA, uint8_t pinB, PIO pio) {
            pin.A = pinA;
            pin.B = pinB;
            this->pio = pio;

            // 1. Serial initialization guard (pico only)
            #if defined(ARDUINO_RASPBERRY_PI_PICO)
                if (!Serial) {
                    Serial.begin(115200);
                    delay(100); 
                    Serial.println("[ERROR] Serial.begin() was not called before Encoder::setup()!");
                    while (true) delay(1000);
                }
            #endif

            // 2. Configuration guard
            if (pin.A == UINT8_MAX || pin.B == UINT8_MAX) {  
                Serial.println("[ERROR] Encoder pins are not set correctly. Please check the arguments of setup().");
                while(true) delay(1000); // freeze micro controller
            }

            // 3. Claim hardware resources and initialize PIO program
            this->sm = pio_claim_unused_sm(this->pio, true);
            this->offset = pio_add_program(this->pio, &quadratureA_program);

            quadratureA_program_init(this->pio, this->sm, this->offset, pin.A, pin.B);
        }

        /*
        ### Overview:
        Fetch the current hardware register value from the PIO FIFO and update the accumulated angle.
        This must be called periodically in your main loop.
        */
        void update() {
            value.last = value.cur;
            // Force the PIO to push X register to FIFO
            pio_sm_exec_wait_blocking(this->pio, this->sm, pio_encode_in(pio_x, 32));
            // Read from FIFO
            value.cur = pio_sm_get_blocking(this->pio, this->sm);
            // Calculate delta with overflow handling
            value.diff = (int32_t)(value.cur - value.last);
        }

        /*
        ### Overview:
        Get the accumulated relative position (angle / step count) of the encoder.
        ### Return
        - `int`: Total relative steps from the starting position.
        */
        int getValue() {
            return (int32_t)value.cur;
        }

        void resetValue() {
            pio_sm_exec(this->pio, this->sm, pio_encode_set(pio_x, 0));
            value.cur = 0;
            value.last = 0;
            value.diff = 0;
        }

    private:
        Pin pin;
        PIO pio;
        uint sm;
        uint offset;
        Value value;
        int relative_angle = 0;
};