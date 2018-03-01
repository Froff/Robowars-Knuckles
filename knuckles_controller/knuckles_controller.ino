#include <SPI.h>
#include "RF24.h"
#include "codesheet.h"
#include "math.h"
#include <LiquidCrystal.h>

// Pins ************************************************************************
#define BUTTON_PIN A5
#define X_AXIS_PIN A7
#define Y_AXIS_PIN A6
#define LED_PIN 2
#define LCDRS 8
#define LCDEN 7
#define LCDD4 6
#define LCDD5 5
#define LCDD6 4
#define LCDD7 3
#define RADIOCE 9
#define RADIOCS 10
#define RADIO_INTERRUPT_PIN 2

// Constants *******************************************************************
#define CALIBRATION_TIME_MILLIS 2000 // Time for calibrating inputs (like joystick) at the start
#define TIMEOUT_MICROSECENDS 2000
#define JOYSTICK_RESEND_PERIOD_MILLIS 1000
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
        pinMode(swPin, INPUT);
    }

    void readPositions() {
        x = analogRead(xPin);
        y = analogRead(yPin);
        sw = digitalRead(swPin) == HIGH;
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
        return (double(t) - 0.5* (double(max) - double(min))) / (double(max) - double(min));
    }
};

// Globals *********************************************************************
Joystick joy (X_AXIS_PIN, Y_AXIS_PIN, BUTTON_PIN);
RF24 radio(RADIOCE, RADIOCS);
LiquidCrystal lcd (LCDRS, LCDEN, LCDD4, LCDD5, LCDD6, LCDD7);
int lcd_draw_countdown = 0;
const int lcd_draw_countdown_start = 5;

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
    Serial.println(F("Began the radio"));
    radio.setPALevel(RF24_PA_LOW);
    Serial.println(F("Set the PA"));
    radio.enableAckPayload();                         // We will be using the Ack Payload feature, so please enable it
    Serial.println(F("Enabled Ack Payload"));
    radio.enableDynamicPayloads();                    // Ack payloads are dynamic payloads
    Serial.println(F("Enabled dynamic payloads"));
    radio.openWritingPipe(writeAdress);
    Serial.println(F("Opened writing pipe"));
    radio.openReadingPipe(1, readAdress);
    Serial.println(F("Opened reading pipe"));
    radio.startListening();
    Serial.println(F("Started listening"));
    Serial.println(F("Finished radio setup"));

    // Pin setup
    pinMode(LED_PIN, OUTPUT);

    // Calibrate joystick
    long start_time = millis();
    while (millis() - start_time < CALIBRATION_TIME_MILLIS) {
        joy.readPositions();
    }

    // LCD setup
    lcd.begin(16,2);
    lcd.print("penor bol");

    // Settinginterrupts
    pinMode(RADIO_INTERRUPT_PIN, INPUT_PULLUP);
    delay(2000);
    attachInterrupt(0, radioInterrupt, LOW);             // Attach interrupt handler to interrupt #0 (using pin 2) on BOTH the sender and receiver
}

void loop() {
    Serial.println(F("Reading joystick"));
    joy.readPositions();
    composeAndSendMessage();

    if (lcd_draw_countdown <= 0) {
        writeLCD();
        lcd_draw_countdown = lcd_draw_countdown_start;
    }
    lcd_draw_countdown--;

    Serial.print(F("End of frame "));
    Serial.println(lcd_draw_countdown);
    delay(5);
}

void writeLCD () {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("joyx ");
    lcd.print(joy.getX());
    lcd.setCursor(0,1);
    lcd.print("joyy ");
    lcd.print(joy.getY());
    if (joy.getSW()) lcd.print("pOOsh");
}

void composeAndSendMessage () {
    int32_t message = composeMessage();
    sendSignal(message);
}

int32_t composeMessage () {
    int32_t message = 0;

    Serial.print(F("zero message: "));
    Serial.println(message);

    // Check button, and add result to message (0 false, 1 true)
    static bool button_down_old = false;
    bool button_down_new = joy.getSW(); // Button is active low
    if (button_down_new && !button_down_old) {
        message |= CODE_BEAK_NOOT_BITMASK;
    }
    button_down_old = button_down_new;

    Serial.print(F("buttoned message: "));
    Serial.println(message);

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
    Serial.println(message);

    return message;
}

void sendSignal(const int32_t & message_out) {
    if (message_out != 0) {
        Serial.print(F("Sending message: "));
        Serial.println(message_out, HEX);
    }

    radio.startWrite(&message_out, sizeof(int32_t), 0);
}

void radioInterrupt (void) {
    bool tx, fail, rx; // tx: successful transmit. fail: failed transmit. rx: recieved message
    radio.whatHappened(tx, fail, rx); // What happened?

    if ( tx ) { Serial.println(F("Send:OK")); }

    if ( fail ) { Serial.println(F("Send:Failed")); }

    if ( rx || radio.available()) {
        uint32_t ack_message;
        radio.read(&ack_message, sizeof(ack_message));
        Serial.print(F("Ack: "));
        Serial.println(ack_message);
    }
}
