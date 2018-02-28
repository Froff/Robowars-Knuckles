// MOTOR PRECOMPILER
#include <Servo.h>

// Construct servo object, along with a pointer used to select one of them at a time.
Servo left_m, right_m;
Servo* active_m;

// Switches what motor active_m points to
void switchWheel() {
  if(active_m == &left_m) active_m = &right_m;
  else active_m = &left_m;
}

void setup(){

  randomSeed(analogRead(A1)); //randomize the random seed with random noise

  // SETUP FOR MOTORS + MOTOR TIMER
  // Set motor pins
  left_m.attach(6);
  right_m.attach(5);
  active_m = &left_m;

  // Timer/Interrupt setup
  TIMSK2 |= 0b1; // Overflow Interrupt Enable
  TCCR2A |= 0b10;
  TCCR2B |= 0b101; // sets prescaler to 1024. Also starts timer
}

void loop(){}

ISR(TIMER2_OVF_vect) {
  const int counter_max = 256;
  static int OVF_counter = 0;
  OVF_counter ++;

  int m_speed = (OVF_counter <= counter_max / 3 ? 300 : 0);
  speed *= (active_m == &left_m ? -1 : 1); //wheels are reversed compared to eachother
  active_m -> writeMicroseconds(1500 + m_speed);

  if (OVF_counter == counter_max) {
    OVF_counter = 0;
    active_m -> writeMicroseconds(1500);
    switchWheel();
  }
}
