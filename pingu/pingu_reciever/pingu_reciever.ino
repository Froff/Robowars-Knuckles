#include <SPI.h>
#include "RF24.h"
#include "codesheet.h"
#include <stdint.h>

// Pins
#define COM_BEAK_NOOT_PIN A2
#define COM_BEAK_VOLUP_PIN A1
#define COM_BEAK_VOLDOWN_PIN A0
#define LEFT_WHEEL_PWM 6
#define RIGHT_WHEEL_PWM 5

// Constants
byte writeAdress[6] = "Singu";
byte readAdress[6] = "Pingu";

// Function declarations
void respond(const int32_t & message_in);
bool handleMessage(const int32_t & message_in);

// Global variables
RF24 radio(9,10);

// Main
void setup() {
    // Serial setup
    Serial.begin(115200);
    Serial.println(F("Finished Serial setup"));

    // Radio setup
    radio.begin();
    radio.setPALevel(RF24_PA_HIGH);
    radio.openWritingPipe(writeAdress);
    radio.openReadingPipe(1, readAdress);
    radio.startListening();
    Serial.println(F("Finished radio setup"));

    // Pin setup
    pinMode(COM_BEAK_NOOT_PIN, OUTPUT); digitalWrite(COM_BEAK_NOOT_PIN, HIGH);
    pinMode(COM_BEAK_VOLUP_PIN, OUTPUT); digitalWrite(COM_BEAK_VOLUP_PIN, HIGH);
    pinMode(COM_BEAK_VOLDOWN_PIN, OUTPUT); digitalWrite(COM_BEAK_VOLDOWN_PIN, HIGH);
    pinMode(LEFT_WHEEL_PWM, OUTPUT); analogWrite(LEFT_WHEEL_PWM, 0);
    pinMode(RIGHT_WHEEL_PWM, OUTPUT); analogWrite(RIGHT_WHEEL_PWM, 0);
}

void loop() {
    int32_t message_in = 0;
    bool message_recieved = false;

    while (radio.available()) {
        radio.read( &message_in, sizeof(message_in) );
        message_recieved = true;
    }

    if (message_recieved) {
        respond(message_in);
        if (handleMessage(message_in)) {
            Serial.println(F("Signal processed."));
        }
    }
}

void respond(const int32_t & message_in) {
    radio.stopListening();
    radio.write( &message_in, sizeof(message_in) );
    radio.startListening();

    if (message_in) {
        Serial.print(F("Sent response "));
        Serial.println(message_in, HEX);
    }
}

// Handles the incoming message. Returns true iff signal was recognized
bool handleMessage(const int32_t & message_in) {
    #define CALLBACK_ARRAY_MAX_SIZE 16
    void (*callbacks[CALLBACK_ARRAY_MAX_SIZE])(void);
    int callbackArraySize = 0;
    bool signalDidsomething = false;

    // Set pins
    if (message_in & CODE_BEAK_NOOT_BITMASK) {
        Serial.println(F("Doin' the noot noot!"));
        signalDidsomething = true;
        digitalWrite(COM_BEAK_NOOT_PIN, LOW);
        callbacks[callbackArraySize] = []() -> void {digitalWrite(COM_BEAK_NOOT_PIN, HIGH); Serial.println(F("Reset the noootynoot!"));};
        callbackArraySize++;
    }
    if (message_in & CODE_BEAK_VOLUP_BITMASK) {
        Serial.println(F("VOLUME UPPP!"));
        signalDidsomething = true;
        digitalWrite(COM_BEAK_VOLUP_PIN, LOW);
        callbacks[callbackArraySize] = []() -> void {digitalWrite(COM_BEAK_VOLUP_PIN, HIGH);};
        callbackArraySize++;
    }
    if (message_in & CODE_BEAK_VOLDOWN_BITMASK) {
        Serial.println(F("Volume down down down down..."));
        signalDidsomething = true;
        digitalWrite(COM_BEAK_VOLDOWN_PIN, LOW);
        callbacks[callbackArraySize] = []() -> void {digitalWrite(COM_BEAK_VOLDOWN_PIN, HIGH);};
        callbackArraySize++;
    }

    int left_wheel_input = (message_in & WHEEL_BITMASK_UNSHIFTED << WHEEL_LEFT_BITSHIFT) >> WHEEL_LEFT_BITSHIFT;
    int left_wheel_direction = left_wheel_input & 0b1000 ? -1 : 1;
    int left_wheel_output = (left_wheel_output & (WHEEL_BITMASK_UNSHIFTED >> 1)) * left_wheel_direction;
    analogWrite(LEFT_WHEEL_PWM, left_wheel_input * 16 + 119);

    int right_wheel_input = (message_in & WHEEL_BITMASK_UNSHIFTED << WHEEL_RIGHT_BITSHIFT) >> WHEEL_RIGHT_BITSHIFT;
    int right_wheel_direction = right_wheel_input & 0b1000 ? -1 : 1;
    int right_wheel_output = (right_wheel_output & (WHEEL_BITMASK_UNSHIFTED >> 1)) * right_wheel_direction;
    analogWrite(RIGHT_WHEEL_PWM, right_wheel_input * 16 + 117);

    if (left_wheel_input != 0 || right_wheel_input != 0) {
        Serial.print(F("Left wheel: "));
        Serial.println(left_wheel_input, BIN);
        Serial.print(F("Right wheel: "));
        Serial.println(right_wheel_input, BIN);
    }

    delayMicroseconds(5000);
    // Clean-up after delay
    for (int i = 0; i < callbackArraySize; i++) {
        callbacks[i]();
    }

    return signalDidsomething;
}
