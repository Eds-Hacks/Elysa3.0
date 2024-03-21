#include <Wire.h>

/*

Left

'R': Turns on the red LED and turns off the blue and green LEDs.
'B': Turns on the blue LED and turns off the red and green LEDs.
'G': Turns on the green LED and turns off the red and blue LEDs.
'E': Turns on the enable pins for two motor drivers and turns off the red and blue LEDs, and turns on the green LED.
'D': Turns off the enable pins for two motor drivers and turns on the red LED, and turns off the blue and green LEDs.
'F': Extends the arm and turns on the blue LED while turning off the red and green LEDs.
'M': Retracts the arm and turns on the green LED while turning off the red and blue LEDs.
'S': Stops the arm movement and turns on the red LED while turning off the blue and green LEDs.
'X': Toggles an external LED connected to pin 13.
'V': Turns on an electromagnet by setting one digital pin to HIGH and the other to LOW, while turning on the blue LED and turning off the red and green LEDs. It also prints "Mag On" to the serial monitor.
'C': Turns off the electromagnet by setting one digital pin to LOW and the other to HIGH, while turning on the red LED and turning off the blue and green LEDs. It also prints "Mag Off" to the serial monitor.

*/

int LED = 13; // pin number for LED
int Red = 12; // pin number for red LED
int Blue = A2 ; // pin number for blue LED
int Green = A1; // pin number for green LED
int Enable1 = 9; // pin number for motor driver enable 1
int Enable2 = 10; // pin number for motor driver enable 2
int MotorA = 7; // pin number for motor driver input A
int MotorB = 8; // pin number for motor driver input B
int MagA = 5; // pin number for electromagnet A
int MagB = 6; // pin number for electromagnet B
byte incomingByte = 0; // variable to store incoming bytes from serial
bool ledOn = false; // Bool to change led state.

void setup() {
  Serial.begin(9600); // start serial communication
  pinMode(Red, OUTPUT); // set pin mode for red LED as output
  pinMode(Blue, OUTPUT); // set pin mode for blue LED as output
  pinMode(Green, OUTPUT); // set pin mode for green LED as output
  pinMode(LED, OUTPUT); // set pin mode for built-in LED as output
  pinMode(Enable1, OUTPUT); // set pin mode for motor driver enable 1 as output
  pinMode(Enable2, OUTPUT); // set pin mode for motor driver enable 2 as output
  pinMode(MotorA, OUTPUT); // set pin mode for motor driver input A as output
  pinMode(MotorB, OUTPUT); // set pin mode for motor driver input B as output
  pinMode(MagA, OUTPUT); // set pin mode for electromagnet A as output
  pinMode(MagB, OUTPUT); // set pin mode for electromagnet B as output

  digitalWrite(Red,HIGH); // turn on red LED
  delay(1000); // wait for 1 second
  digitalWrite(Blue,HIGH); // turn on blue LED
  digitalWrite(Red,LOW); // turn off red LED
  delay(1000); // wait for 1 second
  digitalWrite(Blue,LOW); // turn off blue LED
  digitalWrite(Green,HIGH); // turn on green LED
  delay(1000); // wait for 1 second
  digitalWrite(Green,LOW); // turn off green LED

  digitalWrite(Enable1,HIGH); // enable motor driver 1
  digitalWrite(Enable2,HIGH); // enable motor driver 2

  Wire.begin(10); // Initialize I2C communication with address 10
  Wire.onReceive(receiveEvent); // Set function to be called when receiving data
  // Other setup code
}
void loop() {
  serial(); // call the serial function
}

void serial(){
  if (Serial.available()) {
    incomingByte = Serial.read();

    switch (incomingByte) {
      case 'R': 
        digitalWrite(Red,HIGH);
        digitalWrite(Blue,LOW);
        digitalWrite(Green,LOW);
        break;
      case 'B': 
        digitalWrite(Red,LOW);
        digitalWrite(Blue,HIGH);
        digitalWrite(Green,LOW);
        break;
      case 'G':
        digitalWrite(Red,LOW);
        digitalWrite(Blue,LOW);
        digitalWrite(Green,HIGH);
        break;
      case 'E':
        digitalWrite(Enable1,HIGH);
        digitalWrite(Enable2,HIGH);
        digitalWrite(Red,LOW);
        digitalWrite(Blue,LOW);
        digitalWrite(Green,HIGH);
        break;
      case 'D':
        digitalWrite(Enable1,LOW);
        digitalWrite(Enable2,LOW);
        digitalWrite(Red,HIGH);
        digitalWrite(Blue,LOW);
        digitalWrite(Green,LOW);
        break;
      case 'F':
        digitalWrite(Red,LOW);
        digitalWrite(Green,LOW);
        digitalWrite(Blue,HIGH);
        Serial.println("Extending Arm");
        digitalWrite(MotorA,HIGH);
        digitalWrite(MotorB,LOW); 
        break;
      case 'M':
        digitalWrite(Red,LOW);
        digitalWrite(Green,HIGH);
        digitalWrite(Blue,LOW);
        Serial.println("Retracting Arm");
        digitalWrite(MotorA,LOW);
        digitalWrite(MotorB,HIGH); 
        break;
      case 'S':
        digitalWrite(Red,HIGH);
        digitalWrite(Green,LOW);
        digitalWrite(Blue,LOW);
        Serial.println("Stop");
        digitalWrite(MotorA,LOW);
        digitalWrite(MotorB,LOW); 
        break;
      case 'X': 
      ledOn = !ledOn;
       digitalWrite(LED, ledOn ? HIGH : LOW);
      break;
      case 'V': 
        digitalWrite(Red,LOW);
        digitalWrite(Green,LOW);
        digitalWrite(Blue,HIGH);
        digitalWrite(MagA,HIGH);
        digitalWrite(MagB,LOW);
        Serial.println("Mag On");
        break;
      case 'C': 
        digitalWrite(Red,HIGH);
        digitalWrite(Green,LOW);
        digitalWrite(Blue,LOW);
        digitalWrite(MagA,LOW);
        digitalWrite(MagB,HIGH);
        delay(100);
        digitalWrite(MagA,LOW);
        digitalWrite(MagB,LOW);
        Serial.println("Mag Off");
        break;
    }
  }
}

void receiveEvent(int byteCount) {
  while (Wire.available()) {
    incomingByte = Wire.read();
    switch (incomingByte) {
      case 'R': 
        digitalWrite(Red,HIGH);
        digitalWrite(Blue,LOW);
        digitalWrite(Green,LOW);
        break;
      case 'B': 
        digitalWrite(Red,LOW);
        digitalWrite(Blue,HIGH);
        digitalWrite(Green,LOW);
        break;
      case 'G':
        digitalWrite(Red,LOW);
        digitalWrite(Blue,LOW);
        digitalWrite(Green,HIGH);
        break;
      case 'E':
        digitalWrite(Enable1,HIGH);
        digitalWrite(Enable2,HIGH);
        digitalWrite(Red,LOW);
        digitalWrite(Blue,LOW);
        digitalWrite(Green,HIGH);
        break;
      case 'D':
        digitalWrite(Enable1,LOW);
        digitalWrite(Enable2,LOW);
        digitalWrite(Red,HIGH);
        digitalWrite(Blue,LOW);
        digitalWrite(Green,LOW);
        break;
      case 'F':
        digitalWrite(Red,LOW);
        digitalWrite(Green,LOW);
        digitalWrite(Blue,HIGH);
        Serial.println("Extending Arm");
        digitalWrite(MotorA,HIGH);
        digitalWrite(MotorB,LOW); 
        break;
      case 'M':
        digitalWrite(Red,LOW);
        digitalWrite(Green,HIGH);
        digitalWrite(Blue,LOW);
        Serial.println("Retracting Arm");
        digitalWrite(MotorA,LOW);
        digitalWrite(MotorB,HIGH); 
        break;
      case 'S':
        digitalWrite(Red,HIGH);
        digitalWrite(Green,LOW);
        digitalWrite(Blue,LOW);
        Serial.println("Stop");
        digitalWrite(MotorA,LOW);
        digitalWrite(MotorB,LOW); 
        break;
      case 'X': 
      ledOn = !ledOn;
       digitalWrite(LED, ledOn ? HIGH : LOW);
      break;
      case 'V': 
        digitalWrite(Red,LOW);
        digitalWrite(Green,LOW);
        digitalWrite(Blue,HIGH);
        digitalWrite(MagA,HIGH);
        digitalWrite(MagB,LOW);
        Serial.println("Mag On");
        break;
      case 'C': 
        digitalWrite(Red,HIGH);
        digitalWrite(Green,LOW);
        digitalWrite(Blue,LOW);
        digitalWrite(MagA,LOW);
        digitalWrite(MagB,HIGH);
        delay(100);
        digitalWrite(MagA,LOW);
        digitalWrite(MagB,LOW);
        Serial.println("Mag Off");
        break;
    }
  }
}

