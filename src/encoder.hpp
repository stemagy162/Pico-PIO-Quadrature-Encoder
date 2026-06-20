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
struct AbsoluteValue {
    uint32_t last = 0;
    uint32_t cur = 0;
    int32_t diff = 0;
};

struct RelativeValue {
    int32_t last = 0;
    int32_t cur = 0;
    // RelativeValue::diff = AbsoluteValue.diff, diff field is omitted.
};

struct MicroSecond {
    unsigned long last = 0;
    unsigned long cur = 0;
    double dt;
};

struct Rotation {
    int32_t one_rotation_value; // the value when rotated 360 degrees
    int32_t last_count;
    int32_t last_value;
    int32_t cur_count;
    int32_t cur_value;
    int32_t diff_count = 0;
    int32_t diff_value = 0;
    double speed = 0.0f;
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
        void setup(uint8_t pinA, uint8_t pinB, PIO pio, uint32_t one_rotation_value = INT32_MAX) {
            if (!ARDUINO_RASPBERRY_PI_PICO) errorMessage("You cannot use PIO in the other board but Raspberry Pi Pico!");
            if (!Serial) {
                Serial.begin(115200);
                errorMessage("Serial.begin() was not called before Encoder::setup()!");
            }
            if (abs(pinA - pinB) != 1) errorMessage("Pin A and Pin B must be adjacent! e.g. setup(3, 4, pio0)");

            this->pin.A = pinA;
            this->pin.B = pinB;
            this->pio = pio;
            this->rotation.one_rotation_value = one_rotation_value;

            time.cur = micros();
            time.dt = time.cur - time.last;

            this->sm = pio_claim_unused_sm(this->pio, true);
            this->offset = pio_add_program(this->pio, &quadratureA_program);

            // Todo: implement determination which quadratureA or quadrature B is initialized
            quadratureA_program_init(this->pio, this->sm, this->offset, pin.A, pin.B);
        }

        void update() {
            absolute_value.last = absolute_value.cur;
            rotation.last_value = rotation.cur_value;
            rotation.last_count = rotation.cur_count;
            time.last = time.cur;

            pio_sm_exec_wait_blocking(this->pio, this->sm, pio_encode_in(pio_x, 32));
            absolute_value.cur = pio_sm_get_blocking(this->pio, this->sm);
            absolute_value.diff = (int32_t)(absolute_value.cur - absolute_value.last);

            relative_value.cur += absolute_value.diff;

            if (rotation.one_rotation_value != INT32_MAX) {
                rotation.cur_value += absolute_value.diff;
                rotation.cur_count += rotation.cur_value / rotation.one_rotation_value;
                rotation.cur_value %= rotation.one_rotation_value;
            }

            time.cur = micros();
            time.dt = (time.cur - time.last) / 1000000.0f;
            if (time.dt > 0.0f) {
                rotation.speed = ((double)absolute_value.diff / (double)rotation.one_rotation_value) / time.dt;
            } else {
                // pass
            }
        }

        /*
        ### Overview:
        Get the accumulated relative position (angle / step count) of the encoder.
        Under this process, fetching the current hardware register value from the PIO FIFO and update the accumulated angle.
        ### Return
        - `int`: Total relative steps from the starting position.
        */
        const int32_t getValue() {
            return relative_value.cur;
        }

        const int32_t getValueDisplacement() {
            return absolute_value.diff;
        }

        const int32_t getRotationCount() {
            if (rotation.one_rotation_value == INT32_MAX) errorMessage("Set Rotation Value in setup() to run getRotationCount()! e.g. setup(3, 4, pio0, 120)");
            return rotation.cur_count;
        }

        const int32_t getValueInRotation() {
            return rotation.cur_value;
        }

        const double getRotationSpeed() {
            return rotation.speed;
        }

        // void resetValue() {
        //     pio_sm_exec(this->pio, this->sm, pio_encode_set(pio_x, 0));
        //     absolute_value.cur = 0;
        //     absolute_value.last = 0;
        //     absolute_value.diff = 0;
        // }

    private:
        Pin pin;
        PIO pio;
        uint sm;
        uint offset;
        AbsoluteValue absolute_value;
        RelativeValue relative_value;
        Rotation rotation;
        
        MicroSecond time;

        void errorMessage(String message) {
            Serial.println("[ERROR] " + message);
            while (true) delay((ulong)LONG_MAX); // halt
        }
    
};
