#include <SPI.h>
#include "RF24.h"
#include "codesheet.h"
#include "math.h"

// Pins ************************************************************************
#define BUTTON_PIN 8
#define X_AXIS_PIN A7
#define Y_AXIS_PIN A6
#define LED_PIN 2

// Constants *******************************************************************
#define CALIBRATION_TIME_MILLIS 2000 // Time for calibrating inputs (like joystick) at the start
#define TIMEOUT_MICROSECENDS 200000
#define JOYSTICK_RESEND_PERIOD_MILLIS 1000
#define JOYSTICK_DEADZONE 0.05

byte writeAdress[6] = "Pingu";
byte readAdress[6] = "Singu";

// Class declarations **********************************************************
class Joystick {
public:
    double getX() {double r = joystickSlerp(x, minx, maxx); return abs(r) < JOYSTICK_DEADZONE ? 0 : r;}
    double getY() {double r = joystickSlerp(y, miny, maxy); return abs(r) < JOYSTICK_DEADZONE ? 0 : r;}

    Joystick (int xPin, int yPin) {this->xPin = xPin; this->yPin = yPin;}

    void readPositions() {
        x = analogRead(xPin);
        y = analogRead(yPin);
        calibrate();
    }

private:
    int xPin, yPin;

    int x = 512;
    int y = 512;

    int minx = 1024;
    int maxx = 0;
    int miny = 1024;
    int maxy = 0;

    void calibrate() {
        minx = min(minx, x);
        maxx = max(maxx, x);
        miny = min(miny, y);
        maxy = max(maxy, y);
    }

    // Slerps a numbers x between min and max to a number between -1 and 1
    double joystickSlerp (int t, int min, int max) {
        return (double(t) - 0.5* (double(max) - double(min))) / (double(max) - double(min));
    }
};

// Globals *********************************************************************
Joystick joy (X_AXIS_PIN, Y_AXIS_PIN);
RF24 radio(9,10);

// Function declarations *******************************************************
void sendSignal(const int32_t & value);
void composeAndSendMessage();

// Main ************************************************************************
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
    pinMode(BUTTON_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);

    // Calibrate joystick
    long start_time = millis();
    while (millis() - start_time < CALIBRATION_TIME_MILLIS) {
        joy.readPositions();
    }
}

void loop() {
    delay(1);
    composeAndSendMessage();
    Serial.println("ENDOFFRAME")
}

void composeAndSendMessage () {
    int32_t message = composeMessage();
    sendSignal(message);
}

int32_t composeMessage () {
    int32_t message = 0;

    Serial.print(F("zero message: "));
    Serial.print(message);

    // Check button, and add result to message (0 false, 1 true)
    static bool button_down_old = false;
    bool button_down_new = digitalRead(BUTTON_PIN) == LOW; // Button is active low
    if (button_down_new && !button_down_old) {
        message |= CODE_BEAK_NOOT_BITMASK;
    }
    button_down_old = button_down_new;

    Serial.print(F("buttoned message: "));
    Serial.print(message);

    // Check joysticks
    joy.readPositions();
    double left_wheel_acc = 0.0;
    double right_wheel_acc = 0.0;
    if (joy.getX() != 0) {
        left_wheel_acc  += joy.getX();
        right_wheel_acc += joy.getX();
    }

    // Bound joystick outputs
    if (left_wheel_acc < -0.99) left_wheel_acc = -0.99;
    else if (left_wheel_acc > 0.99) left_wheel_acc = 0.99;
    if (right_wheel_acc < -0.99) right_wheel_acc = -0.99;
    else if (right_wheel_acc > 0.99) right_wheel_acc = 0.99;

    // Create wheel bits
    int32_t left_wheel_input  = (int32_t)(8 * left_wheel_acc) & WHEEL_BITMASK_UNSHIFTED;
    left_wheel_input &= left_wheel_acc < 0 ? 0b1000 : 0b0000;
    left_wheel_input <<= WHEEL_LEFT_BITSHIFT;

    int32_t right_wheel_input = (int32_t)(8 * right_wheel_acc) & WHEEL_BITMASK_UNSHIFTED;
    right_wheel_input &= right_wheel_acc < 0 ? 0b0000 : 0b1000;
    right_wheel_input <<= WHEEL_RIGHT_BITSHIFT;

    message |= left_wheel_input | right_wheel_input;

    Serial.print(F("full message: "));
    Serial.print(message);

    return message;
}

void sendSignal(const int32_t & message_out) {
    if (message_out != 0) {
        Serial.print(F("Sending message: "));
        Serial.println(message_out, HEX);
    }
    radio.stopListening();
    bool failed = !radio.write( &message_out, sizeof(message_out) );
    radio.startListening();
    if (failed) {
        Serial.println(F("Failed to send. Giving up..."));
        return;
    }
    int32_t started_waiting_at = micros();
    boolean timeout = false;
    // Wait for response
    while (!radio.available()) {
        if (micros() - started_waiting_at > TIMEOUT_MICROSECENDS) {
            timeout = true;
            break;
        }
    }
    if (timeout) {
        Serial.println(F("Response timed out."));
        return;
    }
    int32_t message_in;
    radio.read(&message_in, sizeof(message_in));
    if (message_in == message_out) {
        if (message_out != 0) {
            Serial.println(F("Command send and recieved successfully"));
        }
    } else {
        Serial.println(F("Message/response mismatch. Something went wrong"));
    }
}
