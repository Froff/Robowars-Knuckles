#include <LiquidCrystal.h>

class Joystick{
private:
  uint8_t joyXPin, joyYPin, switchPin;
  uint16_t zero_valueX, zero_valueY, threshold;
public:
  Joystick(uint8_t joyXPin, uint8_t joyYPin, uint8_t switchPin, uint16_t threshold): joyXPin(joyXPin), joyYPin(joyYPin), switchPin(switchPin), threshold(threshold){}
  void begin(){
     pinMode( joyXPin, INPUT); pinMode( joyYPin, INPUT); pinMode( switchPin, INPUT);
    zero_valueX = analogRead( joyXPin); zero_valueY = analogRead(joyYPin);
  }
  int16_t getX(){
    int16_t val = analogRead(joyXPin) - zero_valueX;
    return abs(val) > threshold ? val : 0;
  }
  int16_t getY(){
    int16_t val = analogRead(joyYPin) - zero_valueY;
    return abs(val) > threshold ? val : 0;
  }
  bool getSwitch(){ return digitalRead(switchPin);}
};

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const uint8_t sw = 8, joyX = A6, joyY = A7;
Joystick shtikk( joyX, joyY, sw, 5);


void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  shtikk.begin();
  lcd.print("penor bol");
  delay(2000);
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  delay(100);
  lcd.clear();
  lcd.setCursor(0, 0);
  // print the number of seconds since reset:
  lcd.print("joyx ");
  lcd.print(shtikk.getX());
  lcd.setCursor(0,1);
  lcd.print("joyy ");
  lcd.print(shtikk.getY());
  if (shtikk.getSwitch()) lcd.print("pOOsh");
}
