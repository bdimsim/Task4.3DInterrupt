#include <DHT.h>
#include <BH1750.h>

// Must be placed before the header
#define USING_TIMER_TC3 true 
#include <SAMD_ISR_Timer.h>
#include <SAMDTimerInterrupt.h>

#define DHTTYPE DHT11
#define HW_INTERVAL_MS 500L // Timer interval for the internal interrupt
#define TIMER0_INTERVAL_MS 1000L
#define TIMER1_INTERVAL_MS 500L
#define TIMER2_INTERVAL_MS 2000L

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

SAMD_ISR_Timer ISR_Timer;
SAMDTimer ITimer(TIMER_TC3);

void TimerHandler() {
  // This function is called every 500ms
  ISR_Timer.run(); // Call the timer interrupt handler
}

void buttonRedLedToggle() {
  static unsigned long lastInterruptTime = 0;

  if (millis() - lastInterruptTime > 200) {
    redLedState = !redLedState; // Toggle the LED state
    digitalWrite(redLedPin, redLedState); // Update the LED
    lastInterruptTime = millis();
  }
}

void toggleBlueLed() {
  blueLedState = !blueLedState; // Toggle the LED state
  digitalWrite(blueLedPin, blueLedState); // Update the LED
}

void updateLightSensorAndYellowLed() {
  float lux = lightSensor.readLightLevel();
  yellowLedState = lux > 400 ? HIGH : LOW; // Set yellow LED state based on light level
  digitalWrite(yellowLedPin, yellowLedState);
}

void readAndPrintDhtData() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

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

  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonRedLedToggle, FALLING); // Toggle red LED on button press (external interrupt)
  ITimer.attachInterruptInterval_MS(HW_INTERVAL_MS, TimerHandler); // Call TimerHandler every 500ms (internal interrupt)

  ISR_Timer.setInterval(TIMER0_INTERVAL_MS, toggleBlueLed); // Toggle blue LED every second
  ISR_Timer.setInterval(TIMER1_INTERVAL_MS, updateLightSensorAndYellowLed); // Read light sensor every 500ms and toggle yellow LED
  ISR_Timer.setInterval(TIMER2_INTERVAL_MS, readAndPrintDhtData); // Read DHT sensor every 2 seconds and print to serial

  Serial.begin(115200);
  Wire.begin(); // Enables I2C communication (SDA and SCL pins)
  lightSensor.begin(); // Begin measurement with default I2C address 0x23
  dht.begin();
}

void loop() {

}
