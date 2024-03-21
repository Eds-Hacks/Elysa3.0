#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "HCPCA9685.h"
#include <Arduino.h> //do i need this? added for the parse of option 7


float SoftwareVersion = 12.8;

//uptime timer
unsigned long startTime;
// Define I2C slave address for the device/module (default is 0x40)
#define I2CAdd 0x40

const byte A = 0;   // Controls the left elbow
const byte B = 1;   // Controls the left wrist
const byte C = 2;   // Not connected
const byte D = 4;   // Controls the left claw
const byte E = 5;   // Controls the left magnet
const byte F = 6;   // Controls the left shoulder
const byte G = 7;   // Not Connected

// THESE WILL CHANGE WITH REWIRE

const byte H = 8;   // Not connected
const byte I = 9;   // Controls the right claw
const byte J = 10;  // Controls the right magnet
const byte K = 11;  // Not connected
const byte L = 12;  // Not connected
const byte M = 13;  // Not connected
const byte N = 14;  // Controls the right shoulder
const byte O = 15;  // Not Connected
const byte P = 16;  // Not connected

// missing wrist and elbow


int Scroll = 0; //LCD Scroll scroll
int Menu; //LCD manual scroll

//local sub menu 7
  char MegaByte = 'Z'; //Incoming option 7
  int MegaSubMenu; //imcoming option 7
  char Mega = 'Z'; 

// Define I2C addresses for various components
const int lcdAddress1 = 0x27;   // First LCD address
const int lcdAddress2 = 0x3F;   // Second LCD address

// Constants for board addresses
const byte LEFT_CHEST_ADDRESS = 0x0B;
const byte RIGHT_CHEST_ADDRESS = 0x0A;
const byte MUX_BOARD_ADDRESS = 0x0C;
const byte HEAD_ADDRESS = 0x0D;
const byte TRANSCEIVER_ADDRESS = 0x14;
const byte HCPCA9685_ADDRESS = 0x40;

//Threshold for proximity sensors
const int Proxythreshold = 300;

// Define variables to store the current state and input values
int currentState = 0;
char charReceived;
int numberReceived;


// Define buffer variables for each device
char headBuffer = 'N';
char transceiverBuffer = 'O';
char leftChestBuffer = 'R';
char rightChestBuffer = 'R';
char muxBoardBuffer = 'N';
char ServoBuffer;

char BufferSelection;

// For LCD refresh interval
unsigned long previousMillis = 0;
const long interval = 2500; // Interval in milliseconds (0.5 seconds)

// For serial control
unsigned long lastSerialTime = 1000;
int selectedBoard = 0; // Initialize selectedBoard

int selectedAddress = -1;

//H Bridge Motor Driver Setup
int LJA = 25; //Rear Jack 
int LJB = 24; //Rear Jack Down
int RJA = 23; //Front Jack Lift
int RJB = 22; //Front Jack Lower
int FrontJackEnable = 26;
int RearJackEnable = 27;


int speed; // raw pwm value for speed
int speedPercentage = 50; //set 50% speed max 

//Motor pins defined
#define RightRPWM 13
#define RightLPWM 12
#define RightREN 14
#define RightLEN 15
#define LeftRPWM 6
#define LeftLPWM 7
#define LeftREN 16
#define LeftLEN 17

//A10 = current sensor
/*
yellow analogue sensors for the front IR sensors
A,12,8,4,13,9,5,1,2
D 8

relays projector controls 8,9,10,11

motors
rhs
yellow pwm 23
green enable 12 13
sense nc

lhs
yellow pwm 5,4
green enable 6,7
sense nc  


rhs pwm 1,0
rhs enable 12,13

*/
bool JackState = 0;


// Define relay pins

//2,3,4,5
const int ProjectorPower = 2;//OK
const int RaspberryPi = 5;//OK
const int ControlBoards = 4;//I THINK 
const int ChestPower = 3;//ok



/*
const int ProjectorSpare = 8;
const int ProjectorStandby = 9;
const int ProjectorInput = 10;
const int ProjectorUp = 11;
*/
const int ProjectorSpare = 9;//OK
const int ProjectorStandby = 8;
const int ProjectorInput = 11;//OK
const int ProjectorUp = 10;



char ServoByte ;

int ServoPos = 0;

float ExtTemperature;
int potValue;
int CoretemperatureInt;
float Coretemperature;
int CorehumidityInt;
float Corehumidity;
int keyStatus = 1;
float ExtHumidity;

// Define LCD settings
LiquidCrystal_I2C lcd(lcdAddress2, 20, 4);  // I2C address 0x27, 20 columns, 4 rows
LiquidCrystal_I2C lcd1(lcdAddress1, 20, 4); // I2C address 0x3F, 20 columns, 4 rows

// Constants for compass and gyros
#define CMPS11_ADDRESS 0x60 // Address of CMPS11 shifted right one bit for Arduino wire library
#define ANGLE_8 1          // Register to read 8-bit angle from

unsigned char high_byte, low_byte, angle8;
int pitch, roll;
unsigned int angle16;

#define DHTTYPE DHT22
#define DHTPIN 33

// Temp Sensor Setup
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

/* Create an instance of the library */
HCPCA9685 HCPCA9685(I2CAdd);

// Declare the buffer at the global level
static char buffer[64];
static int bufferIndex = 0;

void setup() {
 Serial.begin(9600);  // Initialize serial communication
  Serial.println("Serial communication initialized");


  pinMode(RightRPWM, OUTPUT);
  pinMode(RightLPWM, OUTPUT);
  pinMode(RightLEN, OUTPUT);
  pinMode(RightREN, OUTPUT);
  pinMode(LeftRPWM, OUTPUT);
  pinMode(LeftLPWM, OUTPUT);
  pinMode(LeftLEN, OUTPUT);
  pinMode(LeftREN, OUTPUT);
  //stop the motors
  digitalWrite(RightREN, LOW);
  digitalWrite(RightLEN, LOW);
  digitalWrite(LeftREN, LOW);
  digitalWrite(LeftLEN, LOW);
  Serial.println("Motors disabled");


    // Record the start time
  startTime = millis();
  Serial.begin(9600); // Initialize serial communication
   Wire.begin();        // Initialize I2C communication

   
//Wire.setClock(20000); // Set the I2C clock frequency to 50 kHz


 



 
//Set pin states for jack
  pinMode(LJA, OUTPUT);       //Rear Jack Up
  pinMode(LJB, OUTPUT);       //Rear Jack Down
  pinMode(RJA, OUTPUT);       //Front Jack Lift
  pinMode(RJB, OUTPUT);       //Front Jack Lower

  // Set the initial relay states for the power boards after initializing the LCD
  pinMode(ProjectorPower, OUTPUT);
  pinMode(RaspberryPi, OUTPUT);
  pinMode(ControlBoards, OUTPUT);
  pinMode(ChestPower, OUTPUT);
  pinMode(ProjectorSpare, OUTPUT);
  pinMode(ProjectorStandby, OUTPUT);
  pinMode(ProjectorInput, OUTPUT);
  pinMode(ProjectorUp, OUTPUT);
  Serial.println("Relay pins configured");




  // Set all relay pins to the initial state LOW
  digitalWrite(ProjectorPower, LOW);
  digitalWrite(RaspberryPi, LOW);
  delay(1000);
  digitalWrite(ControlBoards, HIGH);
  delay(1000);
  digitalWrite(ChestPower, HIGH);
  delay(1000);
  digitalWrite(ProjectorSpare, LOW);
  digitalWrite(ProjectorStandby, LOW);
  digitalWrite(ProjectorInput, LOW);
  digitalWrite(ProjectorUp, LOW);
  Serial.println("Relay pins set to initial state");


  // Initialize both LCDs
  lcd.init();
  lcd.backlight();
  lcd.clear();
  Serial.println("First LCD initialized");

  lcd1.init();
  lcd1.backlight();
  lcd1.clear();
  Serial.println("Second LCD initialized");

  /* Initialize the library and set it to 'servo mode' */
  HCPCA9685.Init(SERVO_MODE);
  Serial.println("Servo library initialized");

  /* Wake the device up */
  HCPCA9685.Sleep(false);
  Serial.println("Device woke up");

  ZeroServos();


  dht.begin(); // Initialize temperature and humidity sensor
  Key();       // Call the Key function
  Serial.println("Temperature and humidity sensor initialized");

      Wire.beginTransmission(MUX_BOARD_ADDRESS);
      Wire.write('F');
      Wire.endTransmission();

      lcd.clear();
      lcd1.clear();
  
lcd.setCursor(5, 0);
lcd.print("Diagnostic");  
lcd.setCursor(4, 1);
lcd.print("In Progress");  

SystemStatus();
delay(3000);
//lcd.print("Subsystem Ready");
      lcd.clear();
      lcd1.clear();
  

pinMode(FrontJackEnable, OUTPUT);
pinMode(RearJackEnable, OUTPUT);
digitalWrite(FrontJackEnable, LOW);
digitalWrite(RearJackEnable, LOW);

  Wire.beginTransmission(HEAD_ADDRESS); 
  Wire.write('Z');  
  Wire.endTransmission();


}

void loop() {

  
serial();
ProxySensors();
Temp(); 
TransceiverReceive(); 
Gyro();
ManualMenu();

unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // It's time to do something

    // Call your functions here

    dataDisplay();


    //Serial.println("Functions called");

    previousMillis = currentMillis;
  }
 
}

 
void Gyro() {
  Wire.beginTransmission(CMPS11_ADDRESS); // Start communication with CMPS11
  Wire.write(ANGLE_8);                   // Send the register to start reading from
  Wire.endTransmission();

  // Request 5 bytes from the CMPS11
  Wire.requestFrom(CMPS11_ADDRESS, 5);

  while (Wire.available() < 5); // Wait for all bytes to be received

  angle8 = Wire.read(); // Read the 5 bytes
  high_byte = Wire.read();
  low_byte = Wire.read();
  roll = Wire.read();  // Swap pitch and roll assignments
  pitch = Wire.read(); // Swap pitch and roll assignments

  //Serial.println("Gyro data");

  angle16 = high_byte; // Calculate 16-bit angle
  angle16 <<= 8;
  angle16 += low_byte;

/*
  // Print gyro data
  lcd.setCursor(0, 1);
  lcd.print("Angle ");
  lcd.print(angle16 / 10, DEC);
  lcd.setCursor(0, 2);
  lcd.print("Pitch ");
  lcd.print(pitch, DEC);
  lcd.print("   Roll ");
  lcd.print(roll, DEC);
*/

//if lcd is not showing gryo 

if (Scroll != 1) {
  //Print gyro data to Serial
  Serial.print("Gyro Angle: ");
  Serial.print(angle16 / 10, DEC);
  Serial.print("  Pitch: ");
  Serial.print(pitch, DEC);
  Serial.print("  Roll: ");
  Serial.println(roll, DEC);
}




}
// Function to read temperature and humidity data
void Temp() {
  sensors_event_t event;

  // Get temperature event and print its value
  dht.temperature().getEvent(&event);
  ExtTemperature = event.temperature;
  Serial.println("Temperature data retrieved");

    // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  ExtHumidity = event.relative_humidity;
}

// Function to handle key data
void Key() {
  keyStatus = 1; // Initialize keyStatus to a non-zero value

  while (keyStatus != 0) {



lcd.setCursor(4, 1);
lcd.print("Security Key");
lcd.setCursor(6, 2);
lcd.print("Required");
lcd.setCursor(2, 0);
lcd.print("System Lockout");

display0();
    // Request data from the slave
    Wire.requestFrom(TRANSCEIVER_ADDRESS, 7);
    
    // Add a 1 millisecond delay between send and receive commands
    delay(1);

    if (Wire.available() >= 7) {
      byte data[7];

      for (int i = 0; i < 7; i++) {
        data[i] = Wire.read(); // Read data from the slave
      }
      Serial.println("Key data received");

      // Decode the data received from the slave
      potValue = word(data[0], data[1]);
      CoretemperatureInt = word(data[2], data[3]);
      Coretemperature = CoretemperatureInt / 10.0;
      CorehumidityInt = word(data[4], data[5]);
      Corehumidity = CorehumidityInt / 10.0;
      keyStatus = data[6];
      Serial.println("Key data decoded");



delay(1000);

    }
  }
}

// Function to display data on LCD
void dataDisplay() {


  //lcd.clear();
  lcd1.clear();
  Serial.println("LCDs cleared");

  switch (Scroll) {
    case 0:
      display0();
      Scroll++;
      break;
    case 1:
      display1();
      Scroll++;
      break;
    case 2:
      display2();
      Scroll++;
      break;
    case 3:
      SystemStatus();
      Scroll++;
      break;
    case 4:
      Scroll++;
      Uptime();
      break;
    case 5:
    display3();
      Scroll = 0;
      break;


  }
}


void display0(){


  lcd1.setCursor(5, 0);
  lcd1.print("Elysa 3.0 ");
  lcd1.setCursor(2, 1);
  lcd1.print("Software Version");
  lcd1.setCursor(8, 2);
  lcd1.print(SoftwareVersion);
 

}

void display1(){
  Gyro();
  lcd1.setCursor(6, 0);
   lcd1.print("Gyroscope");  
  lcd1.setCursor(6, 1);
  lcd1.print("Angle ");
  lcd1.print(angle16 / 10, DEC);
  lcd1.setCursor(6, 2);
  lcd1.print("Pitch ");
  lcd1.print(pitch, DEC);
  lcd1.setCursor(4, 3);
  lcd1.print("   Roll ");
  lcd1.print(roll, DEC);

  // Print gyro data to Serial
  Serial.print("Gyro Angle: ");
  Serial.print(angle16 / 10, DEC);
  Serial.print("  Pitch: ");
  Serial.print(pitch, DEC);
  Serial.print("  Roll: ");
  Serial.println(roll, DEC);


}

void display2(){
  // Display temperature data on an LCD
  lcd1.setCursor(0, 0);
  lcd1.print("Ext Temp ");
  lcd1.print(ExtTemperature);
  lcd1.print("C");
  lcd1.setCursor(0, 1);
  lcd1.print("Core Temp: ");
  lcd1.print(Coretemperature);
  lcd1.print("C");
  lcd1.setCursor(0, 3);
  lcd1.print("Core Humidity:");
  lcd1.print(Corehumidity);
  lcd1.print("%");
  lcd1.setCursor(0, 2);
  lcd1.print("Ext Humidity ");
  lcd1.print(ExtHumidity);
  lcd1.print("%");
  Serial.println("Data displayed on LCDs");

}

void display3(){
  lcd1.setCursor(0, 0);
  lcd1.print("Speed = ");
  lcd1.print(speed);
  lcd1.print("%");


}


void display10(){


  // Display temperature data on an LCD
  lcd1.setCursor(0, 0);
  lcd1.print("Ext Temp ");
  lcd1.print(ExtTemperature);
  lcd1.print("C");

  // Display the received values on the first LCD
  lcd.setCursor(0, 0);
  lcd.print("Pot: ");
  lcd.print(potValue);
  lcd1.setCursor(0, 1);
  lcd1.print("Core Temp: ");
  lcd1.print(Coretemperature);
  lcd1.print("C");
  lcd1.setCursor(0, 3);
  lcd1.print("Core Humidity:");
  lcd1.print(Corehumidity);
  lcd1.print("%");
  lcd.setCursor(0, 3);
  lcd.print("Key Status: ");
  lcd.print(keyStatus);

  // Display humidity data on an LCD
  lcd1.setCursor(0, 2);
  lcd1.print("Ext Humidity ");
  lcd1.print(ExtHumidity);
  lcd1.print("%");
  Serial.println("Data displayed on LCDs");
}

void TransceiverReceive() {
  Wire.requestFrom(TRANSCEIVER_ADDRESS, 7); // Request data from the slave

  // Add a 1 millisecond delay between send and receive commands
  delay(1);

  if (Wire.available() >= 7) {
    byte data[7];

    for (int i = 0; i < 7; i++) {
      data[i] = Wire.read(); // Read data from the slave
    }
    Serial.println("Data received from Transceiver");

    // Decode the data received from the slave
    potValue = word(data[0], data[1]);
    CoretemperatureInt = word(data[2], data[3]);
    Coretemperature = CoretemperatureInt / 10.0;
    CorehumidityInt = word(data[4], data[5]);
    Corehumidity = CorehumidityInt / 10.0;
    keyStatus = data[6];
    Serial.println("Data decoded");
  }
}

void ServiceMode(){

digitalWrite(LJA,HIGH); //set left jack to deploy
digitalWrite(LJB,LOW);
digitalWrite(RJA,HIGH); //set right jack to deploy
digitalWrite(RJB,LOW);


}

void DriveMode(){


digitalWrite(LJA,LOW); //set left jack to retract
digitalWrite(LJB,HIGH);

digitalWrite(RJA,LOW); //set right jack to retract
digitalWrite(RJB,HIGH);
// Function to handle key data


}

void serial() {
  if (Serial.available() > 0) {
    byte incomingByte = Serial.read();

    if (selectedBoard == 0) {
      // No board selected yet
      if (incomingByte >= '0' && incomingByte <= '9') {
        // Valid board selection
        selectedBoard = incomingByte - '0';
        Serial.print("Selected Board: ");
        Serial.println(selectedBoard);
      } else {
       Serial.println("No board selected. Enter a number between 1 and 8 to select a board.");
      }
    } else {
      // Board selected, send command to the selected board
      byte boardAddress;

      switch (selectedBoard) {
        
        case 1:
          boardAddress = LEFT_CHEST_ADDRESS;
          timeout();
          break;
        case 2:
          boardAddress = RIGHT_CHEST_ADDRESS;
          timeout();
          break;
        case 3:
          boardAddress = MUX_BOARD_ADDRESS;
          timeout();
          break;
        case 4:
          boardAddress = HEAD_ADDRESS;
          timeout();
          break;
        case 5:
          boardAddress = TRANSCEIVER_ADDRESS;
          timeout();
          break;
        case 6:
          //Servos
          ServoByte = incomingByte;
          ServoPos = Serial.parseInt();
          ServoSend();
          timeout(); 
          break;
case 7:
 


      MegaByte = incomingByte;
    //MegaSubMenu = Serial.parseInt();

      Mega = MegaByte;
 //  Print the parsed number
    Serial.print("megabyte ");
    Serial.print(MegaByte);
    Serial.print("    MegaSubMenu ");
    Serial.println(MegaSubMenu);
    LocalMega();

    break;
  case 8:



    break;
      case 10:
SystemStatus();

    break;
}
      sendCommandToBoard(boardAddress, incomingByte);
    }
  }
}

void sendCommandToBoard(byte boardAddress, byte command) {
  Wire.beginTransmission(boardAddress);
  Wire.write(command);
  Wire.endTransmission();
  selectedBoard = 0;  // Reset board selection
}

void ServoSend() {
  //Replaced switch case so now its mapped A-P vs interget.
  if (ServoByte >= 'A' && ServoByte <= 'P') {
    ServoByte = ServoByte - 'A';
  }
          Serial.print("Servo Selected: ");
          Serial.println(ServoByte);
          Serial.print(" Servo Position: ");
          Serial.print(ServoPos);
  delay(1);
  HCPCA9685.Servo(ServoByte, ServoPos);
  delay(1);
}

void timeout() {
  const unsigned long timeoutPeriod = 500;  // Timeout period in milliseconds (0.5 seconds)

  if (Serial.available() > 0) {
    byte incomingByte = Serial.read();
    // Reset the timer when a byte is received
    lastSerialTime = millis();

    // Rest of your code for handling incoming bytes...
    // (including the logic for selecting the board and sending commands)

  } else {
    // Check if the timeout period has elapsed
    if (millis() - lastSerialTime >= timeoutPeriod) {
      selectedBoard = 0;  // No board selected
      Serial.println("Timeout: No incoming byte for 0.5 seconds. Board selection reset.");
    }
  }
}


void shutdown() {
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("System Shutdown ");

  lcd1.clear();
  lcd1.setCursor(2, 0);
  lcd1.print("System Shutdown ");

  // Communication with MUX_BOARD_ADDRESS
  Wire.beginTransmission(MUX_BOARD_ADDRESS); 
  Wire.write('f');                   
  Wire.write('B');   
  Wire.endTransmission();

  // Communication with HEAD_ADDRESS
  Wire.beginTransmission(HEAD_ADDRESS); 
  Wire.write('X');  
               
Wire.endTransmission();


  Serial.println("Shutdown");

  ZeroServos();

  // Shut down relays 
  digitalWrite(ProjectorPower, LOW);
  digitalWrite(ProjectorSpare, LOW);
  digitalWrite(ProjectorStandby, LOW);
  digitalWrite(ProjectorInput, LOW);
  digitalWrite(ProjectorUp, LOW);
  delay(15000);

    lcd1.clear();
  lcd1.setCursor(2, 0);
  lcd1.print("It is now safe to");
  lcd1.setCursor(4, 1);
  lcd1.print("power off the");
  lcd1.setCursor(7, 2);
  lcd1.print("Robot");

  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("It is now safe to");
  lcd.setCursor(4, 1);
  lcd.print("power off the");
  lcd.setCursor(7, 2);
  lcd.print("Robot");

delay(100);
digitalWrite(ChestPower, LOW);
digitalWrite(RaspberryPi, HIGH);
digitalWrite(ControlBoards, LOW);

 
}

void ZeroServos() {
  HCPCA9685.Servo(0, 150);   // Servo control for byte 0 (RHS Elbow)
  HCPCA9685.Servo(5, 100);   // Servo control for byte 5 (RHSMag)
  HCPCA9685.Servo(14, 100);  // Servo control for byte 14 (RHS Mag)
   HCPCA9685.Servo(13, 150);  // Servo control for byte 13 (RHS Elbow)
  HCPCA9685.Servo(1, 250);   // Servo control for byte 1 (RHS Wrist)
  HCPCA9685.Servo(2, 100);   // Servo control for byte 2 (RHS NC)
  HCPCA9685.Servo(4, 100);   // Servo control for byte 4 (RHS Claw)
  HCPCA9685.Servo(6, 100);   // Servo control for byte 6 (RHS NC)
  HCPCA9685.Servo(7, 100);   // Servo control for byte 7 (RLHS NC)
  HCPCA9685.Servo(8, 100);   // Servo control for byte 8 (LHS NC)
  HCPCA9685.Servo(9, 100);   // Servo control for byte 9 (RHS WRrist)
  HCPCA9685.Servo(10, 100);  // Servo control for byte 10 (LHS Shoulder)
  HCPCA9685.Servo(11, 100);  // Servo control for byte 11 (RHS NC)
  HCPCA9685.Servo(12, 100);  // Servo control for byte 12 (RHS NC)
  HCPCA9685.Servo(15, 100);  // Servo control for byte 15 (RHS NC)
}

void clearManual(){
lcd.setCursor(0, 1); // Set cursor to the first column of the second row
lcd.print("                    "); // Print 20 spaces
lcd.setCursor(0, 2); // Set cursor to the first column of the third row
lcd.print("                    "); // Print 20 spaces

}

void ManualMenu() {
 // Display the received values on the first LCD
  lcd.setCursor(0, 0);
  lcd.print("Manual Menu: ");
  lcd.print(potValue);

  // Use a switch-case statement to perform different actions based on potValue
  switch (potValue) {
    case 0:
  clearManual();
  lcd.setCursor(0, 2);
  lcd.print("This is a test");
      break;
    case 1:
  lcd.setCursor(2, 2);
  lcd.print("1");
      break;
    case 2:
  lcd.setCursor(2, 2);
  lcd.print("2");
      break;
    case 3:
      Serial.print("Display 3: ");
      // Do something for potValue = 3
      break;
    case 4:
      Serial.print("Display 4: ");
      // Do something for potValue = 4
      break;
    case 5:
      Serial.print("Display 5: ");
      // Do something for potValue = 5
      break;
    case 6:
      Serial.print("Display 6: ");
      // Do something for potValue = 6
      break;
    case 7:
      Serial.print("Display 7: ");
      // Do something for potValue = 7
      break;
    case 8:
      Serial.print("Display 8: ");
      // Do something for potValue = 8
      break;
    case 9:
      Serial.print("Display 9: ");
      // Do something for potValue = 9
      break;
    default:
      // This will be executed if potValue is outside the range (0-9)
      break;
  }
}

void SystemStatus() {
  Wire.requestFrom(RIGHT_CHEST_ADDRESS, 17);  // Request 17 bytes from slave with address 10
  delay(100);

  // Check if data is available to read
  if (Wire.available() >= 17) {
    char response[18];  // Increase the array size to accommodate the null terminator

    // Read the response into the array
    for (int i = 0; i < 17; i++) {
      response[i] = Wire.read();
    }

    // Add null terminator to the received data
    response[17] = '\0';

    // Print the received response
    Serial.print("Received response Right Chest: ");
    Serial.println(response);
        lcd1.setCursor(0, 0);
    lcd1.print("Right Chest = ");  
     lcd1.print(response);  
  } else {
    Serial.println("No response received.");
    lcd1.setCursor(0, 0);
    lcd1.print("Right Chest = Dead");  

  }
//////////////////////////////////////////////
  Wire.requestFrom(LEFT_CHEST_ADDRESS, 17);  // Request 17 bytes from slave with address 10
  //delay(100);

  // Check if data is available to read
  if (Wire.available() >= 17) {
    char response[18];  // Increase the array size to accommodate the null terminator

    // Read the response into the array
    for (int i = 0; i < 17; i++) {
      response[i] = Wire.read();
    }

    // Add null terminator to the received data
    response[17] = '\0';

    // Print the received response
    Serial.print("Received response Left Chest: ");
    Serial.println(response);
        lcd1.setCursor(0, 1);
    lcd1.print("Left Chest  = ");  
     lcd1.print(response);  
  } else {
    Serial.println("No response received.");
    lcd1.setCursor(0, 1);
    lcd1.print("Left Chest  = Dead");  

  }
/////////////////////////////////////////////////

//HEAD_ADDRESS
 Wire.requestFrom(HEAD_ADDRESS, 17);  // Request 17 bytes from slave with address 10
  //delay(100);

  // Check if data is available to read
  if (Wire.available() >= 17) {
    char response[18];  // Increase the array size to accommodate the null terminator

    // Read the response into the array
    for (int i = 0; i < 17; i++) {
      response[i] = Wire.read();
    }

    // Add null terminator to the received data
    response[17] = '\0';

    // Print the received response
    Serial.print("Received response Left Chest: ");
    Serial.println(response);
        lcd1.setCursor(6, 2);
    lcd1.print("Head  = ");  
     lcd1.print(response);  
  } else {
    Serial.println("No response received.");
    lcd1.setCursor(6, 2);
    lcd1.print("Head  = Dead");  

  }
////////////////////////////////////////////
Wire.requestFrom(MUX_BOARD_ADDRESS, 17);  // Request 17 bytes from slave with address 10
  //delay(100);

  // Check if data is available to read
  if (Wire.available() >= 17) {
    char response[18];  // Increase the array size to accommodate the null terminator

    // Read the response into the array
    for (int i = 0; i < 17; i++) {
      response[i] = Wire.read();
    }

    // Add null terminator to the received data
    response[17] = '\0';

    // Print the received response
    Serial.print("Received response MUX: ");
    Serial.println(response);
        lcd1.setCursor(7, 3);
    lcd1.print("Mux  = ");  
     lcd1.print(response);  
  } else {
    Serial.println("MUX No response received.");
    lcd1.setCursor(7, 3);
    lcd1.print("Mux  = Dead");  

  }
////////////////////////////////////////////

}


void LocalMega(){
   Serial.print("Mega");

speed = map(speedPercentage, 0, 100, 0, 255);

switch (Mega) {
    case 'A':
  Wire.beginTransmission(MUX_BOARD_ADDRESS);                
  Wire.write('D');   
  Wire.endTransmission();
      ClearLine3();
      ServiceMode();
      Serial.println("Service Mode");
      lcd.setCursor(0, 3); // Set cursor to the beginning of the third row
      lcd.print("Service Mode");
      digitalWrite(FrontJackEnable, HIGH);
      digitalWrite(RearJackEnable, HIGH);
      Stop();
      Serial.println("ServiceMode");
  Wire.beginTransmission(MUX_BOARD_ADDRESS);                
  Wire.write('d');   
  Wire.endTransmission();      
      break;
    case 'B':
      ClearLine3();
      DriveMode();
      Serial.println("Drive Mode");
      lcd.setCursor(0, 3);
      lcd.print("Drive Mode");
      digitalWrite(FrontJackEnable, HIGH);
      digitalWrite(RearJackEnable, HIGH);
      Stop();
      Serial.println("DriveMode");
      break;
    case 'C':
      Wire.beginTransmission(MUX_BOARD_ADDRESS);                
      Wire.write('B');   
      Wire.endTransmission();
      ClearLine3();
      digitalWrite(FrontJackEnable, LOW);
      digitalWrite(RearJackEnable, LOW);
      Stop();
      Serial.println("Stop");
      lcd.setCursor(0, 3);
      lcd.print("Stop");
      break;
    case 'D':
      ClearLine3();
      PowerSaver();
      Stop();
      Serial.println("PowerSaver");
      break;
    case 'E':
      ClearLine3();
      shutdown();
      Stop();
      Serial.println("Shutdown");
      lcd.setCursor(0, 3);
      lcd.print("Shutdown");
      break;
    case 'F':
      ClearLine3();
      Stop();
      Serial.println("Stop");
      lcd.setCursor(0, 3);
      lcd.print("Stop");
      Wire.beginTransmission(MUX_BOARD_ADDRESS);                
      Wire.write('B');   
      Wire.endTransmission();
      break;
    case 'G':
      ClearLine3();
      Forward();
      Serial.println("Forward");
      lcd.setCursor(0, 3);
      lcd.print("Forward");
      Wire.beginTransmission(MUX_BOARD_ADDRESS);                
      Wire.write('b');   
      Wire.endTransmission();
      break;
    case 'H':
      ClearLine3();
      RightForward();
      Serial.println("RightForward");
      lcd.setCursor(0, 3);
      lcd.print("RightForward");
      Wire.beginTransmission(MUX_BOARD_ADDRESS);                
      Wire.write('b');   
      Wire.endTransmission();
      break;
    case 'I':
      ClearLine3();
      LeftForward();
      Serial.println("LeftForward");
      lcd.setCursor(0, 3);
      lcd.print("LeftForward");
      Wire.beginTransmission(MUX_BOARD_ADDRESS);                
      Wire.write('b');   
      Wire.endTransmission();
      break;
    case 'J':
      ClearLine3();
      Backward();
      Serial.println("Backward");
      lcd.setCursor(0, 3);
      lcd.print("Backward");
      Wire.beginTransmission(MUX_BOARD_ADDRESS);                
      Wire.write('b');   
      Wire.endTransmission();
      break;
    case 'K':
      ClearLine3();
      LeftBackward();
      Serial.println("LeftBackward");
      lcd.setCursor(0, 3);
      lcd.print("LeftBackward");
      Wire.beginTransmission(MUX_BOARD_ADDRESS);                
      Wire.write('b');   
      Wire.endTransmission();
      break;
    case 'L':
      ClearLine3();
      RightBackward();
      Serial.println("RightBackward");
      lcd.setCursor(0, 3);
      lcd.print("RightBackward");
      Wire.beginTransmission(MUX_BOARD_ADDRESS);                
      Wire.write('b');   
      Wire.endTransmission();
      break;
    case 'M':
      ClearLine3();
      Serial.println("ProjectorStandby ");
      lcd.setCursor(0, 3);
      lcd.print("ProjectorStandby ");
digitalWrite(ProjectorStandby, HIGH);
delay(300);
digitalWrite(ProjectorStandby, LOW);

      break;
    case 'N':
      ClearLine3();
      Serial.println("(ProjectorInput  ");
      lcd.setCursor(0, 3);
      lcd.print("Proj Standby ");
digitalWrite(ProjectorInput, HIGH);
delay(300);
digitalWrite(ProjectorInput, LOW);

      break;
    case 'O':
      ClearLine3();
      Serial.println("ProjectorUp  ");
      lcd.setCursor(0, 3);
      lcd.print("ProjectorUp  ");
digitalWrite(ProjectorUp, HIGH);
delay(300);
digitalWrite(ProjectorUp, LOW);
      break;
    case 'P':
      ClearLine3();
      Serial.println("Proj Pwr On,  ");
      lcd.setCursor(0, 3);
      lcd.print("(Proj Pwr on  ");
digitalWrite(ProjectorPower, HIGH);

      break;
    case 'Q':
      ClearLine3();
      Serial.println("Proj Pwr Off,  ");
      lcd.setCursor(0, 3);
      lcd.print("(Proj Pwr off  ");
digitalWrite(ProjectorInput, HIGH);
delay(300);
digitalWrite(ProjectorInput, LOW);
delay(500);
digitalWrite(ProjectorInput, HIGH);
delay(300);
digitalWrite(ProjectorInput, LOW);

 //digitalWrite(ProjectorPower, LOW);
      break;













    default:
      // Code to handle unexpected values of Mega
      Serial.println("Unknown Function");
      lcd.setCursor(0, 3);
      lcd.print("Unknown Function");
      break;
  }
}

/*
Case 'A': Enters Service Mode, enables front and rear jacks, and stops.
Case 'B': Enters Drive Mode, enables front and rear jacks, and stops.
Case 'C': Disables front and rear jacks, and stops.
Case 'D': Enters Power Saver mode and stops.
Case 'E': Shuts down and stops.
Case 'F': Stops.
Case 'G': Moves forward.
Case 'H': Moves right forward.
Case 'I': Moves left forward.
Case 'J': Moves backward.
Case 'K': Moves left backward.
Case 'L': Moves right backward.
Case 'M': Prints "Wake Started" to Serial, displays "Wake" on LCD, and invokes the Wake function.
Default Case: Handles unknown or unexpected values of 'Mega', prints "Unknown Function" to Serial, and displays the same on the LCD.F stop
G forward
H right forward
I left forward
J backward
K left backward
L Right BackwardServic
*/

void ClearLine3(){
   // Clear the local function screen
    lcd.setCursor(0, 3);
    lcd.print("                    ");
  }





void PowerSaver(){
 Serial.print("Power Saver Mode");
     

  // Communication with MUX_BOARD_ADDRESS
  Wire.beginTransmission(MUX_BOARD_ADDRESS); 
  Wire.write('f');                   
  Wire.write('A');   
  Wire.endTransmission(); 

  // Communication with HEAD_ADDRESS
  Wire.beginTransmission(HEAD_ADDRESS); 
  Wire.write(F);  
  Wire.endTransmission();

  lcd1.clear();
  lcd1.setCursor(4, 0);
  lcd1.print("Power Saver");
  lcd1.setCursor(2, 2);
  lcd1.print("Reboot required");
  lcd1.setCursor(5, 3);
  lcd1.print("after sleep");

  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("Power Saver");
  lcd.setCursor(2, 2);
  lcd.print("Reboot required");
  lcd.setCursor(5, 3);
  lcd.print("after sleep");


delay(100);
digitalWrite(ChestPower, LOW);



}

void Uptime(){
// Calculate the elapsed time in seconds
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - startTime) / 1000;

  // Calculate minutes and seconds
  unsigned long minutes = elapsedTime / 60;
  unsigned long seconds = elapsedTime % 60;

  // Calculate hours and days
  unsigned long hours = minutes / 60;
  unsigned long days = hours / 24;

  // Display the runtime
  Serial.print("System has been Running for: ");
  Serial.print(days);
  Serial.print(" days, ");
  Serial.print(hours % 24); // Display hours modulo 24 to avoid showing more than 24 hours
  Serial.print(" hours, ");
  Serial.print(minutes % 60); // Display minutes modulo 60 to avoid showing more than 60 minutes
  Serial.print(" minutes, ");
  Serial.print(seconds);
  Serial.println(" seconds");
  
  lcd1.setCursor(0, 0);
  lcd1.print("System Uptime:");
  lcd1.setCursor(0, 1);
  lcd1.print("Days: ");
  lcd1.print(days);
  lcd1.setCursor(0, 2);
  lcd1.print("Hrs: ");
  lcd1.print(hours % 24);
  lcd1.setCursor(0, 3);
  lcd1.print("Min: ");
  lcd1.print(minutes % 60);
  lcd1.print(" Sec: ");
  lcd1.print(elapsedTime % 60);

}
void ProxySensors() {



  // Initialize an array to store the previous values of each pin
  static int previousValues[16] = {0};

  // Define the user-friendly names for the pins
  int pinsOfInterest[] = {12, 8, 13, 9, A5};  // Added A5 for the GroundGuard sensor
  const char* pinNames[] = {"Right Side", "Front Right Bumper", "Left Front Bumper", "Left Side", "GroundGuard"};  // Added "GroundGuard" for the new sensor
  int numPinsOfInterest = sizeof(pinsOfInterest) / sizeof(pinsOfInterest[0]);

  // Variable to store the triggered pin index
  int triggeredPinIndex = -1;

  // Iterate through the user-friendly named pins
  for (int i = 0; i < numPinsOfInterest; i++) {
    int pin = pinsOfInterest[i];
    const char* pinName = pinNames[i];

    // Read the analog value from the current pin
    int value = analogRead(pin);

    // Check if the value has dropped under the threshold (200) for the bumper sensors
    if (i < 4 && value < 200) {
      // Set the triggered pin index
      triggeredPinIndex = i;

      // Print "Proximity Warning" followed by the pin name to Serial
      Serial.print("Proximity Warning - ");
      Serial.println(pinName);
    }
    // Check if the value has risen above the threshold (800) for the GroundGuard sensor
    else if (i == 4 && value > 800) {
      // Set the triggered pin index
      triggeredPinIndex = i;

      // Print "Proximity Warning" followed by the pin name to Serial
      Serial.print("Proximity Warning - ");
      Serial.println(pinName);
    }

    // Update the previous value for this pin
    previousValues[pin] = value;
  }

  // Send the byte 'B' or 'b' based on the triggered pin
  if (triggeredPinIndex != -1) {
    // Send 'B' if any of the bumper sensors are triggered
    Wire.beginTransmission(MUX_BOARD_ADDRESS);
    Wire.write('B');
    Wire.endTransmission();

    // Display warning on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Proximity Warning");
    lcd.setCursor(0, 1);

    // Print the specific sensor that is triggered using switch case
    switch (triggeredPinIndex) {
      case 4:  // Adjusted the case for the new sensor
        lcd.print("GroundGuard");
        Serial.println("Proximity Warning - GroundGuard");
        Stop();
        delay(1000);
        lcd.clear();
        Wire.beginTransmission(MUX_BOARD_ADDRESS);
        Wire.write('b');
        Wire.endTransmission();
        break;
      case 3:
        lcd.print("Right Side");
        Serial.println("Proximity Warning - Right Side");
        Stop();
        delay(1000);
        lcd.clear();
        Wire.beginTransmission(MUX_BOARD_ADDRESS);
        Wire.write('b');
        Wire.endTransmission();
        break;
      case 2:
        lcd.print("Front Right Bumper");
        Serial.println("Proximity Warning - Front Right Bumper");
        Stop();
        delay(1000);
        lcd.clear();
        Wire.beginTransmission(MUX_BOARD_ADDRESS);
        Wire.write('b');
        Wire.endTransmission();
        break;
      case 1:
        lcd.print("Left Front Bumper");
        Serial.println("Proximity Warning - Left Front Bumper");
        Stop();
        delay(1000);
        lcd.clear();
        Wire.beginTransmission(MUX_BOARD_ADDRESS);
        Wire.write('b');
        Wire.endTransmission();
        break;
      case 0:
        lcd.print("Left Side");
        Serial.println("Proximity Warning - Left Side");
        Stop();
        delay(1000);
        lcd.clear();
        Wire.beginTransmission(MUX_BOARD_ADDRESS);
        Wire.write('b');
        Wire.endTransmission();
        break;
    }
  } else {
    // No proximity warning, you can add code here if needed for this condition
  }
  /*
   if (digitalRead(21) == LOW) {
         // Display warning on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Proximity Warning");
    lcd.setCursor(0, 1);
    // RetroGuard is triggered
    lcd.print("RetroGuard");
    Serial.println("Proximity Warning - RetroGuard");
    Stop();
    delay(1000);
    
    Wire.beginTransmission(MUX_BOARD_ADDRESS);
    Wire.write('B');  // Assuming RetroGuard should also trigger 'B' for the MUX board, adjust if necessary
    Wire.endTransmission();
 }
 */
}



void Stop (){

  digitalWrite(RightREN, LOW);
  digitalWrite(RightLEN, LOW);
  digitalWrite(LeftREN, LOW);
  digitalWrite(LeftLEN, LOW);
  Serial.println("Motors disabled");


}
 void Forward(){

 
  digitalWrite(RightREN, HIGH);
  digitalWrite(RightLEN, HIGH);
 digitalWrite(LeftREN, HIGH);
  digitalWrite(LeftLEN, HIGH);
  
  analogWrite(RightRPWM,speed); 
  analogWrite(RightLPWM,0); 
    
  analogWrite(LeftLPWM,speed); 
  analogWrite(LeftRPWM,0); 
 }

 void Backward(){


  digitalWrite(RightREN, HIGH);
  digitalWrite(RightLEN, HIGH);
 digitalWrite(LeftREN, HIGH);
  digitalWrite(LeftLEN, HIGH);
  
  analogWrite(RightRPWM,0); 
  analogWrite(RightLPWM,speed); 
    
  analogWrite(LeftLPWM,0); 
  analogWrite(LeftRPWM,speed); 
 }

void LeftForward(){
  digitalWrite(RightREN, LOW);
  digitalWrite(RightLEN, LOW);
 digitalWrite(LeftREN, HIGH);
  digitalWrite(LeftLEN, HIGH);
    
  analogWrite(LeftLPWM,speed); 
  analogWrite(LeftRPWM,0); 

}
void RightForward(){
  digitalWrite(RightREN, HIGH);
  digitalWrite(RightLEN, HIGH);
 digitalWrite(LeftREN, LOW);
  digitalWrite(LeftLEN, LOW);
    
  analogWrite(RightLPWM,0); 
  analogWrite(RightRPWM,speed); 

}

void LeftBackward(){
  digitalWrite(RightREN, LOW);
  digitalWrite(RightLEN, LOW);
 digitalWrite(LeftREN, HIGH);
  digitalWrite(LeftLEN, HIGH);
    
  analogWrite(LeftLPWM,0); 
  analogWrite(LeftRPWM,speed); 

}
void RightBackward(){
  digitalWrite(RightREN, HIGH);
  digitalWrite(RightLEN, HIGH);
 digitalWrite(LeftREN, LOW);
  digitalWrite(LeftLEN, LOW);
    
  analogWrite(RightLPWM,speed); 
  analogWrite(RightRPWM,0); 

}

void Wake(){
  // Set all relay pins to the initial state LOW
  digitalWrite(ProjectorPower, LOW);
  digitalWrite(RaspberryPi, LOW);
  delay(1000);
  digitalWrite(ControlBoards, HIGH);
  delay(1000);
  digitalWrite(ChestPower, HIGH);
  delay(1000);

  Serial.println("System Wake Complete");
  // Communication with MUX_BOARD_ADDRESS
  Wire.beginTransmission(MUX_BOARD_ADDRESS); 
  lcd.clear();
  SystemStatus();
  Wire.write('F');                   
  Wire.write('a');   
  Wire.endTransmission(); 
delay(2000);

}

void ProjectorUP(){

  Wire.beginTransmission(HEAD_ADDRESS); 
  Wire.write('S');  
  Wire.endTransmission();


}
void ProjectorDown(){

  Wire.beginTransmission(HEAD_ADDRESS); 
  Wire.write('T');  
  Wire.endTransmission();
  
}



