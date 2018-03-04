#include <SPI.h>
#include "RF24.h"
#include "codesheet.h"
#include <stdint.h>

// Pins
#define COM_BEAK_NOOT_PIN A2
#define COM_BEAK_VOLUP_PIN A1
#define COM_BEAK_VOLDOWN_PIN A0
#define ARM_PWM_UP 3
#define ARM_PWM_DOWN 4
#define RIGHT_WHEEL_PWM_FORW 5
#define LEFT_WHEEL_PWM_FORW 6
#define RIGHT_WHEEL_PWM_BACK 7
#define LEFT_WHEEL_PWM_BACK 8


// Constants
byte writeAdress[6] = "Singu";
byte readAdress[6] = "Pingu";
const uint32_t ack = 1;

// Global variables
RF24 radio(9,10);
uint32_t lastmessage = 0;
bool message_in_queue = false;


// Main
void setup() {
    // Serial setup
    Serial.begin(115200);
    Serial.println(F("Finished Serial setup"));

    // Radio setup
    radio.begin();
    radio.setPALevel(RF24_PA_HIGH);
    radio.enableAckPayload();
    radio.enableDynamicPayloads();
    radio.openWritingPipe(writeAdress);
    radio.openReadingPipe(1, readAdress);
    radio.startListening();
    radio.writeAckPayload(1, &ack, sizeof(uint32_t));
    Serial.println(F("Finished radio setup"));

    // Pin setup
    pinMode(COM_BEAK_NOOT_PIN, OUTPUT); digitalWrite(COM_BEAK_NOOT_PIN, HIGH);
    pinMode(COM_BEAK_VOLUP_PIN, OUTPUT); digitalWrite(COM_BEAK_VOLUP_PIN, HIGH);
    pinMode(COM_BEAK_VOLDOWN_PIN, OUTPUT); digitalWrite(COM_BEAK_VOLDOWN_PIN, HIGH);
    pinMode(LEFT_WHEEL_PWM_FORW, OUTPUT); digitalWrite(LEFT_WHEEL_PWM_FORW, LOW);
    pinMode(LEFT_WHEEL_PWM_BACK, OUTPUT); digitalWrite(LEFT_WHEEL_PWM_BACK, LOW);
    pinMode(RIGHT_WHEEL_PWM_FORW, OUTPUT); digitalWrite(RIGHT_WHEEL_PWM_FORW, LOW);
    pinMode(RIGHT_WHEEL_PWM_BACK, OUTPUT); digitalWrite(RIGHT_WHEEL_PWM_BACK, LOW);
    pinMode(ARM_PWM_UP, OUTPUT); digitalWrite(ARM_PWM_UP, LOW);
    pinMode(ARM_PWM_DOWN, OUTPUT); digitalWrite(ARM_PWM_DOWN, LOW);

    // radio interrupt
    pinMode(2, INPUT_PULLUP);
    delay(50);
    attachInterrupt(0, catchMessage, LOW);

    Serial.println(F("Finished setup"));
}

void loop() {
    if (message_in_queue) {
        message_in_queue = false;
        handleMessage(lastmessage);
    }
}

void catchMessage() {
    Serial.println(F("got message!"));
    bool tx,fail,rx;
    radio.whatHappened(tx, fail, rx);
    radio.read(&lastmessage, sizeof(lastmessage));
    radio.writeAckPayload(1, &ack, sizeof(ack));
    message_in_queue = true;
}

// Handles the incoming message. Returns true iff signal was recognized
bool handleMessage(int32_t message_in) {
    // Set pins
    if (message_in & CODE_BEAK_NOOT_BITMASK) {
        Serial.println(F("Doin' the noot noot!"));
        digitalWrite(COM_BEAK_NOOT_PIN, LOW);
    } else digitalWrite(COM_BEAK_NOOT_PIN, HIGH);

    if (message_in & CODE_ARM_UP) {
        Serial.println(F("moving arm up"));
        digitalWrite(ARM_PWM_UP, HIGH);
        digitalWrite(ARM_PWM_DOWN, LOW);
    } else digitalWrite(ARM_PWM_UP, LOW);

    if (message_in & CODE_ARM_DOWN) {
        Serial.println(F("moving arm down"));
        digitalWrite(ARM_PWM_DOWN, HIGH);
        digitalWrite(ARM_PWM_UP, LOW);
    } else digitalWrite(ARM_PWM_DOWN, LOW);

        Serial.print("ARM-UP high? ");
        Serial.println(digitalRead(ARM_PWM_UP) == HIGH);
            Serial.print("ARM-DOWN high? ");
            Serial.println(digitalRead(ARM_PWM_DOWN) == HIGH);

    uint32_t left = (message_in >> WHEEL_LEFT_BITSHIFT);
    if (left & WHEEL_BITMASK_UNSHIFTED) {
        if (left & 0b1000) {
            Serial.println(F("moving left wheel forwards"));
            digitalWrite(LEFT_WHEEL_PWM_BACK, HIGH);
        } else {
            Serial.println(F("moving left wheel back"));
            digitalWrite(LEFT_WHEEL_PWM_FORW, HIGH);
        }
    } else {
        digitalWrite(LEFT_WHEEL_PWM_FORW, LOW);
        digitalWrite(LEFT_WHEEL_PWM_BACK, LOW);
    }

    uint32_t right = (message_in >> WHEEL_RIGHT_BITSHIFT);
    if (right & WHEEL_BITMASK_UNSHIFTED) {
        if (right & 0b1000){
            Serial.println(F("moving right wheel forwards"));
            digitalWrite(RIGHT_WHEEL_PWM_BACK, HIGH);
        } else {
            Serial.println(F("moving right wheel back"));
            digitalWrite(RIGHT_WHEEL_PWM_FORW, HIGH);
        }
    } else {
        digitalWrite(RIGHT_WHEEL_PWM_FORW, LOW);
        digitalWrite(RIGHT_WHEEL_PWM_BACK, LOW);
    }
    return true;
}
