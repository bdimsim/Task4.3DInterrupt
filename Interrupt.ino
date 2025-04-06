#include <DHT.h>
#include <BH1750.h>

// Must be placed before the header
#define USING_TIMER_TC3 true 
#include <SAMD_ISR_Timer.h>
#include <SAMDTimerInterrupt.h>

#define DHTTYPE DHT11
#define HW_INTERVAL_MS 500L
#define TIMER0_INTERVAL_MS 1000L // Blue LED toggle every 1s
#define TIMER1_INTERVAL_MS 500L // Light sensor read every 0.5s
#define TIMER2_INTERVAL_MS 2000L // DHT read every 2s

const uint8_t buttonPin = 2; // Pin for the button
const uint8_t yellowLedPin = 17; // Pin for the yellow LED
const uint8_t blueLedPin = 20; // Pin for the blue LED
const uint8_t redLedPin = 21; // Pin for the red LED
const uint8_t dhtPin = 14; // Pin for the DHT sensor

// Volatile states of LEDs since they are modified in the ISR
volatile uint8_t blueLedState = LOW;
volatile uint8_t redLedState = LOW;
volatile uint8_t yellowLedState = LOW;

// Sensor objects
BH1750 lightSensor; // Light sensor
DHT dht(dhtPin, DHTTYPE); // DHT11

SAMD_ISR_Timer ISR_Timer; // ISR timer (1 hardware timer can service up to 16 ISR timers)
SAMDTimer ITimer(TIMER_TC3); // Use the hardward TC3 timer for the main timer

// Main timer handler called every 500ms
void TimerHandler() {
  // checks if any of the software timers have reached their scheduled 
  // interval and calls the associated callback function such as toggleBlueLed() 
  // that is scheduled to be called every 1000ms (and checked every 500ms)
  ISR_Timer.run();
}

void buttonRedLedToggle() {
  static unsigned long lastInterruptTime = 0; // Store the last interrupt time

  if (millis() - lastInterruptTime > 200) { // Mitigate button bouncing (rapid button activations due to vibration)
    redLedState = !redLedState; // Toggle the LED state
    digitalWrite(redLedPin, redLedState); // Update the LED
    lastInterruptTime = millis(); // Update the last interrupt time
  }
}

void toggleBlueLed() {
  blueLedState = !blueLedState; // Toggle the LED state
  digitalWrite(blueLedPin, blueLedState); // Update the LED
}

void updateLightSensorAndYellowLed() {
  float lux = lightSensor.readLightLevel(); // Read light level
  yellowLedState = lux > 400 ? HIGH : LOW; // Set yellow LED state based on light level
  digitalWrite(yellowLedPin, yellowLedState); // Update the LED
}

void readAndPrintDhtData() {
  float h = dht.readHumidity(); // Read humidity
  float t = dht.readTemperature(); // Read temperature

  if (isnan(h) || isnan(t)) { // Check if the readings are valid
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Print the readings (message) to the serial monitor
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%, Temperature: "));
  Serial.print(t);
  Serial.println(F(" °C"));
}

void setup() {
  // Set pin modes as input and output
  pinMode(buttonPin, INPUT_PULLUP); // default: HIGH
  pinMode(dhtPin, INPUT_PULLUP); // default: HIGH
  pinMode(yellowLedPin, OUTPUT); // default: LOW
  pinMode(blueLedPin, OUTPUT); // default: LOW
  pinMode(redLedPin, OUTPUT); // default: OW

  // Attach interrupts
  // External interrupt on button pin to toggle red LED
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonRedLedToggle, FALLING);
  // Internal interrupt on timer to call TimerHandler every 0.5s
  ITimer.attachInterruptInterval_MS(HW_INTERVAL_MS, TimerHandler);

  // Attach software timers to their respective functions
  ISR_Timer.setInterval(TIMER0_INTERVAL_MS, toggleBlueLed); // Toggle blue LED every 1000ms
  ISR_Timer.setInterval(TIMER1_INTERVAL_MS, updateLightSensorAndYellowLed); // Read light sensor every 500ms and toggle yellow LED
  ISR_Timer.setInterval(TIMER2_INTERVAL_MS, readAndPrintDhtData); // Read DHT sensor every 2 seconds and print a message to serial

  Serial.begin(115200);
  Wire.begin(); // Enables I2C communication (SDA and SCL pins)
  lightSensor.begin(); // Begin measurement with default I2C address 0x23
  dht.begin(); // Begin DHT sensor
}

void loop() {

}
