#include <Arduino.h>

#include "hardware/pio.h"
#include "quadrature.pio.h"
#include <encoder.hpp>

Encoder encoder;
static const uint8_t pinA = 2;
static const uint8_t pinB = pinA + 1;

static const uint32_t ENCODER_PPR = 14; 

void setup() {
    Serial.begin(115200);
    delay(4000);
    Serial.println("Setup...");

    /* encoder */
    encoder.setup(pinA, pinB, pio0, ENCODER_PPR);
}

void loop() {
    encoder.update();
    
    int32_t total_val = encoder.getValue();
    int32_t diff_val  = encoder.getValueDisplacement();
    int32_t rot_count = encoder.getRotationCount();
    int32_t rot_val   = encoder.getValueInRotation();
    double  rot_speed = encoder.getRotationSpeed();

    Serial.print("Total: ");     Serial.print(total_val);
    Serial.print("\tDiff: ");     Serial.print(diff_val);
    Serial.print("\tRotCnt: ");   Serial.print(rot_count);
    Serial.print("\tInRot: ");    Serial.print(rot_val);
    Serial.print("\tSpeed[rps]: "); Serial.println(rot_speed, 2);
    
    delay(1000);
}