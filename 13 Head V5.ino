#include <Wire.h>
#include <LedControl.h>
#include <Servo.h>

LedControl lc = LedControl(12, 11, 10, 6);

byte Cat[8] = {B10001000, B11111000, B10101000, B01110001, B00100001, B01111001, B01111101, B10111110};
byte Apple[8] = {B00011000, B00001000, B01110110, B11111111, B11111111, B11111111, B01111010, B00110100};
byte Question[8] = {B00000000, B00111100, B01100110, B01100110, B01100000, B00111000, B00011000, B00011000};
byte Male[8] = {B00000000, B11110000, B11000000, B10100000, B10011100, B00010010, B00010010, B00001100};
byte Heart[8] = {B00000000, B01000100, B11101110, B11111110, B11111110, B11111110, B00111000, B00010000};
byte Warning[8] = {B00000000, B00011000, B00111100, B00111100, B00111100, B00011000, B00011000, B00000000};
byte Cock[8] = {B00011000, B00100100, B00100100, B00011000, B00011000, B00011000, B00111100, B00111100};
byte A[8] = {B00000000, B01111110, B01100110, B01100110, B01111110, B01100110, B01100110, B01111110};
byte B[8] = {B00000000, B01111100, B01100110, B01100110, B01111100, B01100110, B01100110, B01111100};
byte C[8] = {B00000000, B00111110, B01100110, B01100110, B01100110, B01100110, B01100110, B00111110};
byte D[8] = {B00000000, B01111100, B01100110, B01100110, B01100110, B01100110, B01100110, B01111100};
byte E[8] = {B00000000, B01111110, B01100000, B01100000, B01111100, B01100000, B01100000, B01111110};
byte F[8] = {B00000000, B01111110, B01100000, B01100000, B01111000, B01100000, B01100000, B01100000};
byte G[8] = {B00000000, B00111100, B01100110, B01100000, B01100000, B01101110, B01100110, B00111100};
byte H[8] = {B00000000, B01100110, B01100110, B01100110, B01111110, B01100110, B01100110, B01100110};
byte I[8] = {B00000000, B00111100, B00011000, B00011000, B00011000, B00011000, B00011000, B00111100};
byte J[8] = {B00000000, B00011110, B00001100, B00001100, B01101100, B01101100, B00001100, B00011000};
byte K[8] = {B00000000, B01100110, B01101100, B01111000, B01110000, B01111000, B01101100, B01100110};
byte L[8] = {B00000000, B01100000, B01100000, B01100000, B01100000, B01100000, B01100000, B01111110};
byte M[8] = {B00000000, B11000011, B11000011, B11000011, B11010101, B11010101, B11100111, B11000011};
byte N[8] = {B00000000, B11000110, B11000110, B11000110, B11000110, B11110110, B11100110, B11000110};
byte O[8] = {B00000000, B00111100, B01100110, B01100110, B01100110, B01100110, B01100110, B00111100};
byte P[8] = {B00000000, B01111100, B01100110, B01100110, B01111100, B01100000, B01100000, B01100000};
byte Q[8] = {B00000000, B00111100, B01100110, B01100110, B01100110, B01101110, B00111110, B00000011};
byte R[8] = {B00000000, B01111100, B01100110, B01100110, B01111100, B01111000, B01101100, B01100110};
byte S[8] = {B00000000, B00111110, B00000010, B00000010, B00111100, B01100000, B01100000, B00111100};
byte T[8] = {B00000000, B01111110, B00011000, B00011000, B00011000, B00011000, B00011000, B00011000};
byte U[8] = {B00000000, B01100110, B01100110, B01100110, B01100110, B01100110, B01100110, B00111100};
byte V[8] = {B00000000, B01100110, B01100110, B01100110, B00111100, B00111100, B00011000, B00000000};
byte W[8] = {B00000000, B11000110, B11000110, B11000011, B11000011, B11000011, B11111110, B01101100};
byte X[8] = {B00000000, B01100110, B01100110, B00111100, B00011000, B00111100, B01100110, B01100110};
byte Y[8] = {B00000000, B01100110, B01100110, B01100110, B00111100, B00011000, B00011000, B00011000};
byte Z[8] = {B00000000, B01111110, B00110000, B00011000, B00001100, B00000110, B00000110, B01111110};

const int addrL = 0;  // first LED matrix - Left robot eye
const int addrR = 1;  // second LED matrix - Right robot eye
const int addrM1 = 2; // Mouth 1
const int addrM2 = 3; // Mouth 2
const int addrM3 = 4; // Mouth 3
const int addrM4 = 5; // Mouth 4
//const int PIR = A3;
const int LED = 13;

// Smoke sensor variables
//int DigitalSmoke;
//int AnalogSmoke;

// Threshold for distinguishing between fart and fire
//const int threshold = 15;

// Servo variables
int Panpos = 90;
int Tiltpos = 10;
int Projectorpos = 35;
Servo Tilt;
Servo Pan;
Servo Projector;

void setup() {
  Wire.begin(13);
  Wire.onReceive(receiveEvent);

  pinMode(LED, OUTPUT);

  Pan.attach(6);
  Pan.write(Panpos);
  Tilt.attach(5);
  Tilt.write(Tiltpos);
  Projector.attach(7);
  Projector.write(Projectorpos);

  delay(1000);

  Startup();
  showNeutral();
}

void loop() {
  // No need for a loop as we are handling I2C communication through receiveEvent
}

void receiveEvent(int byteCount) {
  while (Wire.available()) {
    byte incomingByte = Wire.read();
    int lastincomingByte = incomingByte;

    switch (lastincomingByte) {
      case 'A': showAnger(); break;
      case 'B': showWarning(); break;//FLIP
      case 'C': showNeutral(); break;
      case 'D': showEvil(); break;
      case 'E': showSadness(); break; 
      case 'F': showSleep(); break; // FLIP
      case 'G': EmojiSmile(); break; // FLIP
      case 'H': showHeart(); break; // FLIP
      case 'I': showBlink(); break; // LOW
      case 'J': showHappy(); break;// fLIP
      case 'K': Tiltpos = 110; Tilt.write(Tiltpos); break; // Straight Neck
      case 'L': Tiltpos = 65; Tilt.write(Tiltpos); break; // Forward Tilt
      case 'M': Tiltpos = 130; Tilt.write(Tiltpos); break; // Rear Tilt
      case 'O': Panpos = 10; Pan.write(Panpos); break; // Left Pan
      case 'P': Panpos = 160; Pan.write(Panpos); break; // Right Pan
      case 'Q': Panpos = 100; Pan.write(Panpos); break; // Middle Pan
      case 'S': Projectorpos = 40; Projector.write(Projectorpos); break; // Projector Stowed (up)
      case 'T': Projectorpos = 115; Projector.write(Projectorpos); break; // Projector Deployed (Down)
      case 'V': static bool ledOn = false; ledOn = !ledOn; digitalWrite(LED, ledOn ? HIGH : LOW); break; // Toggle the LED state
    }
  }
}

void Startup() {
  lc.shutdown(addrL, false);
  lc.shutdown(addrR, false);
  lc.shutdown(addrM1, false);
  lc.shutdown(addrM2, false);
  lc.shutdown(addrM3, false);
  lc.shutdown(addrM4, false);

for (int i = 0; i <= 5; i++) {
    lc.setIntensity(i, 15); // Set the intensity to maximum (15) for the device at index i
}

  lc.clearDisplay(addrL);
  lc.clearDisplay(addrR);
  lc.clearDisplay(addrM1);
  lc.clearDisplay(addrM2);
  lc.clearDisplay(addrM3);
  lc.clearDisplay(addrM4);

  for (int row = 0; row < 8; row++) {
    lc.setRow(addrL, row, 255);
    lc.setRow(addrR, row, 255);
    lc.setRow(addrM1, row, 255);
    lc.setRow(addrM2, row, 255);
    lc.setRow(addrM3, row, 255);
    lc.setRow(addrM4, row, 255);
    delay(100);
  }

  lc.clearDisplay(addrL);
  lc.clearDisplay(addrR);
  lc.clearDisplay(addrM1);
  lc.clearDisplay(addrM2);
  lc.clearDisplay(addrM3);
  lc.clearDisplay(addrM4);
}

void showNeutral() {
  digitalWrite(LED, LOW);
  byte left[8] = {B00000000, B00111100, B01000010, B01011010, B01011010, B01000010, B00111100, B00000000};
  displayEmotion(left, left);
}
void showHappy() {
  byte left[8] = {B00000000, B00011000, B00100100, B01011010, B01011010, B01111110, B00000000, B00000000};
  byte right[8] = {B00000000, B00011000, B00100100, B01011010, B01011010, B01111110, B00000000, B00000000};
  displayEmotion(left, right);
}


void showEvil() {
  byte left[8] = {B00000000, B00000000, B01111110, B01011010, B01011010, B00100100, B00011000, B00000000};
  byte right[8] = {B00000000, B00000000, B01111110, B01011010, B01011010, B00100100, B00011000, B00000000};
  displayEmotion(left, right);
}

void showAnger() {
  byte left[8] = {B00000000, B00001100, B00010010, B00111010, B01011010, B01000010, B00111100, B00000000};
  byte right[8] = {B00000000, B00110000, B01001000, B01011100, B01011010, B01000010, B00111100, B00000000};
  displayEmotion(left, right);
}

void showBlink() {
  byte left[8] = {B00000000, B00000000, B00000000, B00000000, B11111111, B00000000, B00000000, B00000000};
  byte right[8] = {B00000000, B00000000, B00000000, B00000000, B11111111, B00000000, B00000000, B00000000};
  displayEmotion(left, right);
}


void showHeart() {
  byte left[8] = {B00000000, B01101100, B11111111, B11111111, B11111111, B01111110, B00111100, B00011000};
  byte right[8] = {B00000000, B01101100, B11111111, B11111111, B11111111, B01111110, B00111100, B00011000};
  displayEmotion(left, right);
}


void EmojiSmile() {
  byte left[8] = {B00000000, B01100110, B01100110, B00000000, B10000001, B01000010, B00111100, B00000000};
  byte right[8] = {B00000000, B01100110, B01100110, B00000000, B10000001, B01000010, B00111100, B00000000};
  displayEmotion(left, right);
}

void showSleep() {
  byte left[8] = {B00000000, B00000000, B00000000, B10000001, B01000010, B00111100, B00000000, B00000000};
  byte right[8] = {B00000000, B00000000, B00000000, B10000001, B01000010, B00111100, B00000000, B00000000};
  displayEmotion(left, right);
}


void displayEmotion(byte left[8], byte right[8]) {
  lc.clearDisplay(addrL);
  lc.clearDisplay(addrR);
  for (int row = 0; row < 8; row++) {
    lc.setRow(addrL, row, left[row]);
    lc.setRow(addrR, row, right[row]);
  }
}

void Alarm() {
  showWarning();
  digitalWrite(LED, HIGH);
}

void EmojiSad() {
  byte left[8] = {B00111100, B01000010, B10100101, B10011001, B10000001, B10100101, B01000010, B00111100};
  byte right[8] = {B00111100, B01000010, B10100101, B10011001, B10000001, B10100101, B01000010, B00111100};
  displayEmotion(left, right);
}

void clearMatrix() {
  lc.clearDisplay(addrL);
  lc.clearDisplay(addrR);
}

void showWarning() {
  byte left[8] = {B00011000, B00011000, B00011000, B00011000, B00011000, B00000000, B00011000, B00011000};
  byte right[8] = {B00011000, B00011000, B00011000, B00011000, B00011000, B00000000, B00011000, B00011000};
  displayEmotion(left, right);
}


void ClearAll() {
  lc.clearDisplay(addrM1);
  lc.clearDisplay(addrM2);
  lc.clearDisplay(addrM3);
  lc.clearDisplay(addrM4);
  lc.clearDisplay(addrR);
  lc.clearDisplay(addrL);
}

void ClearMouth() {
  lc.clearDisplay(addrM1);
  lc.clearDisplay(addrM2);
  lc.clearDisplay(addrM3);
  lc.clearDisplay(addrM4);
}

void ShowApple() {
  lc.clearDisplay(2);
  lc.clearDisplay(5);
  lc.clearDisplay(3);
  lc.clearDisplay(4);
  for (int i = 0; i < 8; i++) lc.setRow(addrM1, i, Apple[i]);
  for (int i = 0; i < 8; i++) lc.setRow(addrM4, i, Apple[i]);
}

void ShowCat() {
  lc.clearDisplay(2);
  lc.clearDisplay(5);
  lc.clearDisplay(3);
  lc.clearDisplay(4);
  for (int i = 0; i < 8; i++) lc.setRow(addrM1, i, Cat[i]);
  for (int i = 0; i < 8; i++) lc.setRow(addrM4, i, Cat[i]);
}

void showSadness() {
  byte left[8] = {B00000000, B00111100, B01000010, B01011010, B01011010, B00111010, B00010010, B00001100};
  byte right[8] = {B00000000, B00111100, B01000010, B01011010, B01011100, B01001000, B00110000, B00000000};
  displayEmotion(left, right);
}

