#include <HX711_ADC.h> // HX711 for load cell
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // I2C LCD
#include <SoftwareSerial.h> // SIM800L
#include <Adafruit_Sensor.h>
#include <DHT.h>

// Load Cell Configuration
HX711_ADC LoadCell(6, 7);//dt6

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 16, 2);

// SIM800L Configuration
SoftwareSerial mySerial(3, 2); // RX, TX

// DHT11 Sensor Configuration
#define DHTPIN 4 // DHT11 sensor pin
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Sound Sensor Configuration
const int soundSensorPin = A0;
const int sampleWindow = 50;
unsigned int soundLevel;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  
  LoadCell.begin();
  LoadCell.start(2000);
  LoadCell.setCalFactor(45.5);
  
  lcd.init();
  lcd.backlight();
  dht.begin();
  
  lcd.setCursor(0, 0);
  lcd.print("Hive Monitor");
  delay(3000);
  lcd.clear();
}

void loop() {
  LoadCell.update();
  float rawweight = LoadCell.getData();
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
   float weight = rawweight-0; // general weight reduce 
  // Read sound level
  unsigned long startMillis = millis();
  unsigned int peakToPeak = 0;
  while (millis() - startMillis < sampleWindow) {
    int value = analogRead(soundSensorPin);
    if (value > peakToPeak) {
      peakToPeak = value;
    }
  }
  soundLevel = peakToPeak;

  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("W:");
  lcd.print(weight, 1);
  lcd.print("g");
  
  lcd.setCursor(9, 0);
  lcd.print("T:");
  lcd.print(temp, 1);
  lcd.print("C");
  
  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(humidity, 1);
  lcd.print("%");
  
  lcd.setCursor(9, 1);
  lcd.print("S:");
  lcd.print(soundLevel);

  Serial.print("Weight: "); Serial.print(weight, 1); Serial.println(" kg");
  Serial.print("Temperature: "); Serial.print(temp, 1); Serial.println(" C");
  Serial.print("Humidity: "); Serial.print(humidity, 1); Serial.println(" %");
  Serial.print("Sound Level: "); Serial.println(soundLevel);

  // Alert if necessary
  if (weight > 35000 || temp > 40.0 || humidity < 20.0 || soundLevel > 72) {
    sendAlert(weight, temp, humidity, soundLevel);
  }
  
  //delay(2000);
}

void sendAlert(float weight, float temp, float humidity, int soundLevel) {
  mySerial.println("AT+CMGF=1");
  delay(1000);
  mySerial.println("AT+CMGS=\"+917094387289\""); // Replace with actual number
  delay(1000);
  mySerial.print("Hive Alert! W:");
  mySerial.print(weight);
  mySerial.print("g T:");
  mySerial.print(temp);
  mySerial.print("C H:");
  mySerial.print(humidity);
  mySerial.print("% S:");
  mySerial.print(soundLevel);
  mySerial.println(".");
  delay(1000);
  mySerial.write(26);
}