#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// Define I2C address for this device
const int deviceAddress = 0x14;

// Define analog pins for potentiometer and key
int Pot = A0;
int Key = 2;

// Initialize variables for potentiometer, key status, temperature, and humidity
int PotValue = 0;
int keyStatus = 0;
float temperature = 0.0;
float relative_humidity = 0.0;

// DHT sensor setup
#define DHTPIN 3
#define DHTTYPE DHT11
DHT_Unified dht(DHTPIN, DHTTYPE);

void setup() {
  Wire.begin(deviceAddress); // Initialize I2C communication
  Wire.onRequest(requestEvent); // Register the event handler for I2C requests
  pinMode(Pot, INPUT_PULLUP);
  pinMode(Key, INPUT_PULLUP);
 // Serial.begin(9600); // Initialize serial communication
  dht.begin();
}

void loop() {
  // Read the analog value from the Pot pin
  PotValue = analogRead(Pot);

  // Map the PotValue to a range of 0 to 10
  PotValue = map(PotValue, 0, 1023, 0, 9);

  // Read temperature and humidity
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (!isnan(event.temperature)) {
    temperature = event.temperature;
  }
  dht.humidity().getEvent(&event);
  if (!isnan(event.relative_humidity)) {
    relative_humidity = event.relative_humidity;
  }

  // Read the key status
  keyStatus = digitalRead(Key);
/*
  // Print temperature, humidity, potentiometer value, and key status
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  Serial.print("Humidity: ");
  Serial.print(relative_humidity);
  Serial.println(" %");
  Serial.print("Potentiometer Value: ");
  Serial.println(PotValue);
  Serial.print("Key Status: ");
  Serial.println(keyStatus);
  */
}

// This function is called when the master requests data
void requestEvent() {
  // Create a byte array to hold the sensor data
  byte data[9];

  // Put the potentiometer values into the data array
  data[0] = highByte(PotValue);
  data[1] = lowByte(PotValue);

  // Encode temperature as an integer with one decimal point
  int tempInt = int(temperature * 10);
  data[2] = highByte(tempInt);
  data[3] = lowByte(tempInt);

  // Encode humidity as an integer with one decimal point
  int humidityInt = int(relative_humidity * 10);
  data[4] = highByte(humidityInt);
  data[5] = lowByte(humidityInt);

  // Put the key status into the data array
  data[6] = keyStatus;

  // Send the data to the master
  Wire.write(data, 7);
}
