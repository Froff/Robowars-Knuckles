// Sound precompiler stuff
#include <SD.h>                 // need to include the SD library
#define SD_ChipSelectPin 4      //using digital pin 4 on arduino nano 328, can use other pins
#define DISABLE_SPEAKER2
#include <TMRpcm.h>           //  also need to include this library...
#include <SPI.h>

// Define pins
#define NOOT_PIN A0
#define VOLUME_UP_PIN A1
#define VOLUME_DOWN_PIN A2

// Construct sound fetching/playing object
TMRpcm tmrpcm;

// Setup sound files
const int noot_count = 5; //number of entries in noots
char* noots [] = { //the names of all the sound files (with file ending)
  "noot_00.wav",
  "noot_01.wav",
  "noot_02.wav",
  "noot_03.wav",
  "noot_04.wav"
};

// Plays a random sound file from noots
void playRandom() {
  Serial.println("Playing sound.");
  tmrpcm.play(noots[random(noot_count)]);
}

void setup() {
    randomSeed(analogRead(A7) * analogRead(A6)); //randomize the random seed with random noise
    Serial.begin(115200);
    Serial.println("checking for SD card...");

    // Set-up SD-card / speaker
    tmrpcm.speakerPin = 9;
    if (!SD.begin(SD_ChipSelectPin)) {
        Serial.println("SD fail");
        return;
    }
    Serial.println("SD success!");

    // Set pinmodes and pullups
    pinMode(NOOT_PIN, INPUT);
    digitalWrite(NOOT_PIN, HIGH);
    pinMode(VOLUME_UP_PIN, INPUT);
    digitalWrite(VOLUME_UP_PIN, HIGH);
    pinMode(VOLUME_DOWN_PIN, INPUT);
    digitalWrite(VOLUME_DOWN_PIN, HIGH);

    playRandom();
    tmrpcm.setVolume(6);
}

void loop() {
    static bool noot_old = false;
    static bool volume_up_old = false;
    static bool volume_down_old = false;

    if (wasPressedDown(NOOT_PIN, noot_old)) {
        playRandom();
    }
}

bool wasPressedDown(int pin, bool & old_value) {
    bool new_value = digitalRead(pin) == LOW;
    bool r = false;
    if (new_value && !old_value) {
        r = true;
    }
    old_value = new_value;
    return r;
}
