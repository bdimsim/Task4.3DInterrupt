#include <DHT.h>
#include <BH1750.h>

// Must be placed before the header
#define USING_TIMER_TC3 true 
#define USING_TIMER_TC4 true
#define USING_TIMER_TCC true
#include <SAMDTimerInterrupt.h>

#define DHTTYPE DHT11

const uint8_t buttonPin = 2; // Pin for the button
const uint8_t yellowLedPin = 17;
const uint8_t blueLedPin = 20;
const uint8_t redLedPin = 21;
const uint8_t dhtPin = 14; // Pin for the DHT sensor

volatile uint8_t blueLedState = LOW;
volatile uint8_t redLedState = LOW;
volatile uint8_t yellowLedState = LOW;

BH1750 lightSensor;
DHT dht(dhtPin, DHTTYPE);
SAMDTimer ITimer0(TIMER_TC3);
SAMDTimer ITimer1(TIMER_TC4);
SAMDTimer ITimer2(TIMER_TCC);

void buttonRedLedToggle() {
  static unsigned long lastInterruptTime = 0;

  if (millis() - lastInterruptTime > 200) {
    redLedState = !redLedState; // Toggle the LED state
    digitalWrite(redLedPin, redLedState); // Update the LED

    lastInterruptTime = millis();
  }
}

void periodicBlueLedToggle() {
  blueLedState = !blueLedState; // Toggle the LED state
  digitalWrite(blueLedPin, blueLedState); // Set the LED to the new state
}

void periodicLightSensor() {
  float lux = lightSensor.readLightLevel();

  if (lux > 400) yellowLedState = HIGH;
  else yellowLedState = LOW;

  digitalWrite(yellowLedPin, yellowLedState);
}

void periodicDhtSensor() {
  float h = dht.readHumidity(); // Read humidity
  float t = dht.readTemperature(); // Read temperature as Celsius

  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%, Temperature: "));
  Serial.print(t);
  Serial.println(F(" °C"));
  
}

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // HIGH
  pinMode(dhtPin, INPUT_PULLUP); // HIGH

  pinMode(yellowLedPin, OUTPUT); // LOW
  pinMode(blueLedPin, OUTPUT); // LOW
  pinMode(redLedPin, OUTPUT); // LOW

  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonRedLedToggle, FALLING);
  ITimer0.attachInterruptInterval_MS(1000, periodicBlueLedToggle);
  ITimer1.attachInterruptInterval_MS(500, periodicLightSensor);
  ITimer2.attachInterruptInterval_MS(2000, periodicDhtSensor);

  Serial.begin(115200);
  Wire.begin(); // Enables I2C communication (SDA and SCL pins)
  lightSensor.begin(); // Begin measurement with default I2C address 0x23
  dht.begin();
}

void loop() {

}
