#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// SIM800L Configuration
SoftwareSerial mySerial(3, 2); // RX, TX

// DHT11 Sensor Configuration
#define DHTPIN 4         // Pin connected to the DHT11 sensor
#define DHTTYPE DHT11    // Define the sensor type as DHT11
DHT dht(DHTPIN, DHTTYPE);

// Sound Sensor Configuration
const int soundSensorPin = A0;  // Analog pin for the sound sensor
const int sampleWindow = 50;    // Sampling window for sound in milliseconds

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column, 2 rows

// Thresholds
const float soundThreshold = 7.0;  // Sound level in dB
const float tempThreshold = 36.0;   // Temperature in °C

void setup() {
  // Initialize Serial Communication
  Serial.begin(9600);
  mySerial.begin(9600);  // SIM800L baud rate
  
  // Initialize DHT Sensor
  dht.begin();

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.print("Initializing...");
  delay(1000);
  lcd.clear();

  // Initialize SIM800L
  mySerial.println("AT");
  updateSerial();
  mySerial.println("AT+CMGF=1"); // Set SMS mode to text
  updateSerial();

  lcd.print("System Ready");
  delay(1000);
  lcd.clear();
}

void loop() {
  // ---- Sound Sensor Logic ----
  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned long startMillis = millis();

  // Collect sound data within the sample window
  while (millis() - startMillis < sampleWindow) {
    int sensorValue = analogRead(soundSensorPin);
    if (sensorValue > signalMax) signalMax = sensorValue;
    if (sensorValue < signalMin) signalMin = sensorValue;
  }

  unsigned int peakToPeak = signalMax - signalMin;
  double voltage = (peakToPeak * 5.0) / 1023.0;  // Convert to voltage (5V system)
  double dB = 20.0 * log10(voltage / 0.00631);   // Convert to approximate dB

  // ---- Temperature Sensor Logic (DHT11) ----
  float temperature = dht.readTemperature();  // Read temperature in °C

  // Check for DHT11 sensor errors
  if (isnan(temperature)) {
    lcd.clear();
    lcd.print("DHT11 Error!");
    delay(1000);
    return;
  }

  // ---- Output to LCD ----
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("Sound: ");
  lcd.print(dB);
  lcd.print(" dB");

  // ---- Alert Logic ----
  if (dB > soundThreshold || temperature > tempThreshold) {
    lcd.clear();
    lcd.print("ALERT!");
    lcd.setCursor(0, 1);
    lcd.print("Sending SMS...");
    sendSMS(temperature, dB);  // Send SMS alert
  }

  delay(2000);  // Wait 5 seconds before the next reading
}

void sendSMS(float temperature, double dB) {
  mySerial.println("AT+CMGS=\"your reciveing no\""); // Replace with your phone number
  updateSerial();

  String message = "ALERT! Temp: " + String(temperature) + "°C, Sound: " + String(dB) + "dB";
  mySerial.print(message);
  updateSerial();

  mySerial.write(26); // CTRL+Z to send the message
  updateSerial();

  lcd.clear();
  lcd.print("SMS Sent!");
  delay(3000);
}

void updateSerial() {
  delay(500);
  while (Serial.available()) {
    mySerial.write(Serial.read()); // Forward data to SIM800L
  }
  while (mySerial.available()) {
    Serial.write(mySerial.read()); // Forward data to Serial Monitor
  }
}
