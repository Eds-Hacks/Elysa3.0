#include <Wire.h>

const int s0 = 2;
const int s1 = 3;
const int s2 = 4;
const int MuxOut = A0;

// Number of outputs on the multiplexer
const int numOutputs = 8;

// Boolean variables for each LED
bool ledState[numOutputs] = {false, false, false, false, false, false, false, false};





void setup() {
  // Set the select pins as outputs
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);

  // Set the MuxOut pin as output
  pinMode(MuxOut, OUTPUT);

  // Initialize I2C communication as slave
  Wire.begin(12);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(sendResponse);  // set function to be called when a request is received

}

void loop() {




  // Loop through each LED
  for (int i = 0; i < numOutputs; i++) {
    // Set the select pins based on the current LED
    digitalWrite(s0, i & 0x01);
    digitalWrite(s1, (i >> 1) & 0x01);
    digitalWrite(s2, (i >> 2) & 0x01);

    // Turn on or off the LED based on its corresponding Boolean state
    digitalWrite(MuxOut, ledState[i] ? HIGH : LOW);

    // Delay for one second if the LED is on, otherwise no delay
    delay(ledState[i] ? 1000 : 0);


  }



}

void toggleLED(int index, bool state) {
  // Check if the index is within the valid range of LEDs
  if (index >= 1 && index < numOutputs) {
    // Set the LED state
    ledState[index] = state;
  }
}

void receiveEvent(int numBytes) {
  while (Wire.available()) {
    // Read the incoming byte
    char incomingByte = Wire.read();

    // Process the selected LED based on the received byte
    switch (incomingByte) {
      case 'A':
        toggleLED(1, true);
        break;
      case 'B':
        toggleLED(2, true);
        break;
      case 'C':
        toggleLED(3, true);
        break;
      case 'D':
        toggleLED(4, true);
        break;
      case 'E':
        toggleLED(6, true);
        break;
      case 'F':
        toggleLED(7, true);
        break;
      case 'a':
        toggleLED(1, false);
        break;
      case 'b':
        toggleLED(2, false);
        break;
      case 'c':
        toggleLED(3, false);
        break;
      case 'd':
        toggleLED(4, false);
        break;
      case 'e':
        toggleLED(6, false);
        break;
      case 'f':
        toggleLED(7, false);
        break;
      default:
        // Invalid input, do nothing
        break;
    }
  }
}
void sendResponse() {
  // This function is called when a request is received from the master
  Wire.write("Alive");    // send the response back to the master
  Wire.write('\0');       // add null terminator
}