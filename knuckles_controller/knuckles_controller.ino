#include <SPI.h>
#include "RF24.h"
#include "codesheet.h"
#include "math.h"
#include <LiquidCrystal.h>

// Pins ************************************************************************
#define BUTTON_PIN A5
#define X_AXIS_PIN A7
#define Y_AXIS_PIN A6
#define LCDRS 8
#define LCDEN 7
#define LCDD4 6
#define LCDD5 5
#define LCDD6 4
#define LCDD7 3
#define RADIOCE 9
#define RADIOCS 10
#define RADIO_INTERRUPT_PIN 2

#define ARM_DOWN_PIN A0
#define ARM_UP_PIN A1

// Constants *******************************************************************
#define JOYSTICK_DEADZONE 0.05

byte writeAdress[6] = "Pingu";
byte readAdress[6] = "Singu";

// Class declarations **********************************************************
class Joystick {
public:
    double getX() {double r = joystickSlerp(x, minx, maxx); return abs(r) < JOYSTICK_DEADZONE ? 0 : r;}
    double getY() {double r = joystickSlerp(y, miny, maxy); return abs(r) < JOYSTICK_DEADZONE ? 0 : r;}
    bool getSW() {return sw;}

    Joystick (int xPin, int yPin, int swPin) {
        this->xPin = xPin;
        this->yPin = yPin;
        this->swPin = swPin;
        pinMode(xPin, INPUT);
        pinMode(yPin, INPUT);
        pinMode(swPin, INPUT_PULLUP);
    }

    void readPositions() {
        x = analogRead(xPin);
        y = analogRead(yPin);
        sw = digitalRead(swPin) == LOW;
        calibrate();
    }

private:
    int xPin, yPin, swPin;

    int x = 512;
    int y = 512;
    bool sw = false;

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
        return 2*(double(t) - 0.5* (double(max) - double(min))) / (double(max) - double(min));
    }
};

// Globals *********************************************************************
Joystick joy (X_AXIS_PIN, Y_AXIS_PIN, BUTTON_PIN);
RF24 radio(RADIOCE, RADIOCS);
LiquidCrystal lcd (LCDRS, LCDEN, LCDD4, LCDD5, LCDD6, LCDD7);
int lcd_draw_countdown = 0;
const int lcd_draw_countdown_start = 1;

// Function declarations *******************************************************
void sendSignal(const uint32_t & value);
uint32_t composeAndSendMessage();

// Main ************************************************************************
void setup() {
    // Serial setup
    Serial.begin(115200);

    // Radio setup
    radio.begin();
    radio.setPALevel(RF24_PA_HIGH);
    radio.enableAckPayload();                         // We will be using the Ack Payload feature, so please enable it
    radio.enableDynamicPayloads();                    // Ack payloads are dynamic payloads
    radio.openWritingPipe(writeAdress);
    radio.openReadingPipe(1, readAdress);

    // Pin setup
    pinMode(ARM_UP_PIN, INPUT_PULLUP);
    pinMode(ARM_DOWN_PIN, INPUT_PULLUP);

    // LCD setup
    lcd.begin(16,2);
    lcd.print("penor bol");

    // Settinginterrupts
    pinMode(RADIO_INTERRUPT_PIN, INPUT_PULLUP);
    delay(50);
    attachInterrupt(0, radioInterrupt, LOW);             // Attach interrupt handler to interrupt #0 (using pin 2) on BOTH the sender and receiver

    delay(50);
}

void loop() {
    joy.readPositions();
    uint32_t message= composeAndSendMessage();

    if (lcd_draw_countdown <= 0) {
        writeLCDjoysticks();
        lcd_draw_countdown = lcd_draw_countdown_start;
    }
    lcd_draw_countdown--;

    Serial.println(message, BIN);
    delay(100);
}

void writeLCDjoysticks () {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("joyx ");
    lcd.print(joy.getX());
    lcd.setCursor(0,1);
    lcd.print("joyy ");
    lcd.print(joy.getY());
    if (joy.getSW()) lcd.print("pOOsh");
}

uint32_t composeAndSendMessage () {
    uint32_t message = composeMessage();
    sendSignal(message);
    return message;
}

uint32_t composeMessage () {
    uint32_t message = 0;

    // Check buttons, and add result to message (0 false, 1 true)
    if (joy.getSW()) { message |= CODE_BEAK_NOOT_BITMASK; }
    if (digitalRead(ARM_UP_PIN) == LOW) { message |= CODE_ARM_UP; }
    if (digitalRead(ARM_DOWN_PIN) == LOW) { message |= CODE_ARM_DOWN; }

    // Check joysticks
    joy.readPositions();
    double left_wheel_acc = 0.0;
    double right_wheel_acc = 0.0;
    left_wheel_acc  += (joy.getY() + joy.getX())/2;
    right_wheel_acc += (joy.getY() - joy.getX())/2;

    // Bound joystick outputs
    if (left_wheel_acc < -0.99) left_wheel_acc = -0.99;
    else if (left_wheel_acc > 0.99) left_wheel_acc = 0.99;
    if (right_wheel_acc < -0.99) right_wheel_acc = -0.99;
    else if (right_wheel_acc > 0.99) right_wheel_acc = 0.99;

    // Create wheel bits
    uint32_t left_wheel_input  = ((uint32_t)floor(abs(8 * left_wheel_acc))) & WHEEL_BITMASK_UNSHIFTED;
    left_wheel_input |= left_wheel_acc < 0 ? 0b1000 : 0b0000;
    left_wheel_input <<= WHEEL_LEFT_BITSHIFT;

    uint32_t right_wheel_input = ((uint32_t)floor(abs(8 * right_wheel_acc))) & WHEEL_BITMASK_UNSHIFTED;
    right_wheel_input |= right_wheel_acc < 0 ? 0b1000 : 0b0000;
    right_wheel_input <<= WHEEL_RIGHT_BITSHIFT;

    message |= left_wheel_input | right_wheel_input;

    return message;
}

void sendSignal(const uint32_t & message_out) {
    radio.startWrite(&message_out, sizeof(uint32_t), 0);
}

void radioInterrupt (void) {
    Serial.println(F("hey"));
    bool tx, fail, rx; // tx: successful transmit. fail: failed transmit. rx: recieved message
    radio.whatHappened(tx, fail, rx); // What happened?

    if ( tx ) { Serial.println(F("sent!")); }

    if ( fail ) { Serial.println(F("FAIL!!")); }

    if ( rx || radio.available()) {
        uint32_t ack_message;
        radio.read(&ack_message, sizeof(ack_message));
        Serial.println(ack_message);
    }
}
