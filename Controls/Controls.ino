/**
* Innovative Global Solutions
* Infant Incubator Main System Code
*/


//TODO:
// Add Bound Screen Class
// Add polling function pass to Bound class
// add comments
// get consistent naming
// fix variable scoping

//Public libraries
#include <Arduino.h>
#include <Wire.h>
#include <LCD-I2C.h>
#include "Adafruit_SHT31.h"
#include "MAX30105.h"
#include "heartRate.h"
#include <stdint.h>
#include "DFRobot_STS3X.h"


//Custom libraries
#include "Bound.hpp"

//Pinout
#define HEAT_PIN 14
#define BUTTON_HUMID_BUTTON 11  // Digital pin for humidity screen button
#define HOME_SCREEN_BUTTON 8    // Digital pin for home screen
#define BUTTON_TEMP_BUTTON 12   // Digital pin for temperature screen
#define BUTTON_DOWN 36          // Digital pin for the down button
#define BUTTON_UP 33            // Digital pin for the up button
#define BUTTON_OK 6             // Digital pin for the ok button
#define BUTTON_BPM 9
#define BUTTON_EXTTEMP 28


#define CORE_TEMP_NUM_SAMPLES 10  // Number of samples for the rolling average


DFRobot_STS3X sts(&Wire, STS3X_I2C_ADDRESS_B);
float coreTempReadings[CORE_TEMP_NUM_SAMPLES];  // Array to store the last temperature readings
int currentIndex = 0;                           // Current index for storing the reading
bool isArrayFilled = false;                     // Flag to indicate when the array is fully populated

MAX30105 particleSensor;


LCD_I2C lcd = LCD_I2C(0x27, 20, 4);  // Default address of most PCF8574 modules, change according

const uint8_t RATE_SIZE = 4;         // Number of HR samples to average
uint8_t rates[RATE_SIZE];            // Array of heart rates
uint8_t rateSpot = 0;                // Array position tracker
long lastBeat = 0;                   // Time of the last beat
float BPM;                           // HR in beats per minute
float avgBPM;                        // Average BPM
long irValue;                        // Sensor's infrared value
const int PulseSensorPurplePin = 0;  // Analog pin for pulse sensor (if used in parallel)
int LED = LED_BUILTIN;               // Onboard Arduino LED
int Signal;                          // Raw signal data
int ogAvg = 515;
int threshholdAvg = ogAvg;  // Starting threshold average
int dataPts = 10;
int threshholdPts[10];  // Array for moving average
bool beat = false;
int iteratorLoc = 0;            // Moving window index
float upperThresholdAvg = 0.0;  // Average of values above the threshold
unsigned long peakTimes[8];     // Peak timestamps for BPM calculation
int peakIndex = 0;              // Peak array index
int peakWindowSize = 8;         // Window size for peak timestamps
float bpm = 0.0;                // Calculated BPM

Adafruit_SHT31 sht31 = Adafruit_SHT31();
const int buzzer = 7;  //buzzer to arduino pin 9

typedef enum{
  HOME_SCREEN,
  HUMIDITY_SCREEN,
  TEMPERATURE_SCREEN,
  BPM_SCREEN,
  EXTTEMP_SCREEN
} screen;

// const int HOME_SCREEN = 0;
// const int HUMIDITY_SCREEN = 1;
// const int TEMPERATURE_SCREEN = 2;
// const int BPM_SCREEN = 3;
// const int EXTTEMP_SCREEN = 4;
// below are the switch states for the bounds screen they help determine state of the button (high or low) to determine if screenswitching is necessary.
int humidSwitchState = 0;
int homeSwitchState = 0;
int tempSwitchState = 0;
int BPMSwitchState = 0;
int EXTSwitchState = 0;
int bpmSwitchState = 0;
int exbtempSwitchState = 0;
int i;  // Counter for min/max
int nextEncoderState[4] = { 2, 0, 3, 1 };
int prevEncoderState[4] = { 1, 3, 0, 2 };
int EncoderPinA = 0;
int EncoderPinB = 1;
int EncoderPos = 0;
float t;  //The read temperature
float h;  //The read humidity
int state, prevstate = 0, count = 0;

double softBoundArray[2] = { .33, .66 };
Bound TempBound = Bound(20, 30, softBoundArray);
Bound HumidBound = Bound(40, 60, softBoundArray);
Bound InfBPMBound = Bound(60, 100);
Bound InfExtTempBound = Bound(15, 40, softBoundArray);

float ok = 0;    //counter for ok button
screen currentScreen = HOME_SCREEN;  //0 is home screen, 1 is temperature screen, 2 is humidity
float lastScreenChange = 0;
float lastIncrement = 0;
// float ExternalBodyTemp;     // External body temperature
float rollingAvgTempC = 0.0;

void doEncoder() {
  state = (digitalRead(EncoderPinA) << 1) | digitalRead(EncoderPinB);
  if (state != prevstate) {
    if (state == nextEncoderState[prevstate]) {
      EncoderPos = 1;
    } else if (state == prevEncoderState[prevstate]) {
      EncoderPos = -1;
    }
    //Serial.println(EncoderPos, DEC);
    prevstate = state;
  }
  if (millis() - lastIncrement > 500) {
    if (currentScreen == TEMPERATURE_SCREEN) {
      if (ok == 1) {
        TempBound.lowerBound += EncoderPos;
      } else if (ok == 2) {
        TempBound.upperBound += EncoderPos;
      }
    } else if (currentScreen == HUMIDITY_SCREEN) {
      if (ok == 1) {
        HumidBound.lowerBound += EncoderPos;
      } else if (ok == 2) {
        HumidBound.upperBound += EncoderPos;
      }
    } else if (currentScreen == BPM_SCREEN) {
      if (ok == 1) {
        InfBPMBound.lowerBound += EncoderPos;
      } else if (ok == 2) {
        InfBPMBound.upperBound += EncoderPos;
      }
    } else if (currentScreen == EXTTEMP_SCREEN) {
      if (ok == 1) {
        InfExtTempBound.lowerBound += EncoderPos;
      } else if (ok == 2) {
        InfExtTempBound.upperBound += EncoderPos;
      }
    }
    lastIncrement = millis();
  }
}
void setup() {
  while (sts.begin() != true) {
    // Serial.println("Failed to init chip, please check if the chip connection is fine.");
    delay(1000);
  }
  // Serial.println("Begin ok!");
  sts.setFreq(sts.e10Hz);
  // Initialize the array with zeros
  for (int i = 0; i < CORE_TEMP_NUM_SAMPLES; i++) {
    coreTempReadings[i] = 0.0;
  }
  if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false)  //Use default I2C port, 400kHz speed
  {
    // Serial.println("MAX30105 was not found.");
    while (1);
  } 
  particleSensor.setup();                     //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);  //Turn off Red LED
  particleSensor.setPulseAmplitudeGreen(0);   //Turn off Green LED
  particleSensor.enableDIETEMPRDY();          //Enable the temp ready interrupt
    // Initialize threshold array for dynamic qualities
  for (int i = 0; i < dataPts; i++) {
    threshholdPts[i] = ogAvg;
  }
  // Initialize rates array to 0
  memset(rates, 0, sizeof(rates));
  pinMode(LED, OUTPUT);
  pinMode(BUTTON_HUMID_BUTTON, INPUT_PULLDOWN);  //establishes connection of button
  pinMode(HOME_SCREEN_BUTTON, INPUT_PULLDOWN);   //establishes connection of button
  pinMode(BUTTON_TEMP_BUTTON, INPUT_PULLDOWN);   //establishes connection of button
  pinMode(BUTTON_BPM, INPUT_PULLDOWN);
  pinMode(BUTTON_EXTTEMP, INPUT_PULLDOWN);
  pinMode(EncoderPinA, INPUT);
  pinMode(EncoderPinB, INPUT);
  pinMode(BUTTON_OK, INPUT);
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(26, OUTPUT);  // Set pin 9 as an output
  pinMode(24, OUTPUT);
  pinMode(29, OUTPUT);
  digitalWrite(6, HIGH);
  digitalWrite(EncoderPinA, HIGH);
  digitalWrite(EncoderPinB, HIGH);
  attachInterrupt(0, doEncoder, CHANGE);
  attachInterrupt(1, doEncoder, CHANGE);
  attachInterrupt(BUTTON_OK, incrementScreen, FALLING);
  lcd.begin(&Wire);
  lcd.display();
  lcd.backlight();
  if (!sht31.begin(0x44)) {  // Set to 0x45 for alternate i2c addr
    while (1) delay(1);
  }
  pinMode(buzzer, OUTPUT);
  pinMode(22, OUTPUT);
}

void incrementScreen() {
  if (millis() - lastScreenChange > 500) {
    ok++;
    lastScreenChange = millis();
  }
}
//Updates the BPM screen
void updateBPMScreen() {
  lcd.clear();
  lcd.setCursor(1, 0);  // adjust position
  lcd.print("BPM:");
  lcd.setCursor(10, 0);
  lcd.print(avgBPM);
  if (InfBPMBound.getStatus(avgBPM) < 0) {
    lcd.setCursor(0, 1);  // adjust position
    lcd.print("*MinBound:");
  }
  lcd.setCursor(1, 1);  // adjust position
  lcd.print("MinBound:");
  lcd.print(InfBPMBound.lowerBound);
  lcd.setCursor(1, 2);
  lcd.print("MaxBound:");
  if (InfBPMBound.getStatus(avgBPM) > 0) {
    lcd.setCursor(0, 2);  // adjust position
    lcd.print("*MaxBound:");
  }
  lcd.print(InfBPMBound.upperBound);
  lcd.display();
  deactivateWarning();

}
void updateEXTTempScreen() {
  lcd.clear();
  lcd.setCursor(1, 0);  // adjust position
  lcd.print("INF TEMP:");
  lcd.setCursor(10, 0);
  lcd.print(rollingAvgTempC);
  lcd.setCursor(15, 0);
  lcd.print(char(223));
  lcd.setCursor(16, 0);
  lcd.print("C");
  if (InfExtTempBound.getStatus(rollingAvgTempC) < 0) {
    lcd.setCursor(0, 1);  // adjust position
    lcd.print("*MinBound:");
  }
  lcd.setCursor(1, 1);  // adjust position
  lcd.print("MinBound:");
  lcd.print(InfExtTempBound.lowerBound);
  lcd.setCursor(1, 2);
  lcd.print("MaxBound:");
  if (InfExtTempBound.getStatus(rollingAvgTempC) > 0) {
    lcd.setCursor(0, 2);  // adjust position
    lcd.print("*MaxBound:");
  }
  lcd.print(InfExtTempBound.upperBound);
  lcd.display();
  deactivateWarning();
  // Serial.print("Ext Temp Bounds: ");
  // Serial.print(EXT[0]);
  // Serial.print(" - ");
  // Serial.println(EXT[1]);
}
//Updates the homescreen with latest data plus displaying it
void updateHomeScreen(int t, int h) {
  lcd.clear();
  lcd.setCursor(1, 0);  // adjust position
  lcd.print("TEMP:");
  if (TempBound.getStatus(t)) {
    lcd.setCursor(0, 0);  // adjust position
    lcd.print("*TEMP:");
  }
  lcd.setCursor(1, 1);  // adjust position
  lcd.print(t);
  lcd.setCursor(3, 1);
  lcd.print(char(223));
  lcd.setCursor(4, 1);
  lcd.print("C");
  lcd.setCursor(1, 2);  // adjust position
  lcd.print("HUMID:");
  if (HumidBound.getStatus(h)) {
    lcd.setCursor(0, 2);  // adjust position
    lcd.print("*HUMID:");
  }
  lcd.setCursor(1, 3);  // adjust position
  lcd.print(h);
  lcd.setCursor(3, 3);  // adjust position
  lcd.print("%");
  lcd.setCursor(10, 0);
  lcd.print("INF BPM:");
  if (InfBPMBound.getStatus(avgBPM)) {
    lcd.setCursor(9, 0);  // adjust position
    lcd.print("*INF BPM:");
  }
  lcd.setCursor(10, 1);
  lcd.print(avgBPM);
  lcd.setCursor(10, 2);
  lcd.print("INF TEMP:");
  if (InfExtTempBound.getStatus(rollingAvgTempC)) {
    lcd.setCursor(9, 2);  // adjust position
    lcd.print("*INF TEMP");
  }
  lcd.setCursor(15, 3);  // adjust position
  lcd.print(char(223));
  lcd.setCursor(16, 3);  // adjust position
  lcd.print("C");
  lcd.setCursor(10, 3);
  lcd.print(rollingAvgTempC);
  lcd.display();
}
//Updates the temperature screen plus displaying it
void updateTempScreen() {
  lcd.clear();
  lcd.setCursor(1, 0);  // adjust position
  lcd.print("TEMP:");
  lcd.setCursor(10, 0);
  lcd.print(t);
  lcd.setCursor(15, 0);
  lcd.print(char(223));
  lcd.setCursor(16, 0);
  lcd.print("C");
  lcd.setCursor(1, 1);  // adjust position
  lcd.print("MinBound:");
  if (TempBound.getStatus(t) < 0) {
    lcd.setCursor(0, 1);  // adjust position
    lcd.print("*MinBound:");
  }
  lcd.print(TempBound.lowerBound);
  lcd.setCursor(1, 2);
  lcd.print("MaxBound:");
  if (TempBound.getStatus(t) > 0) {
    lcd.setCursor(0, 2);  // adjust position
    lcd.print("*MaxBound:");
  }
  lcd.print(TempBound.upperBound);
  lcd.display();
  deactivateWarning();
}
//Updates the humidity screen plus displaying it
void updateHumidScreen() {
  lcd.clear();
  lcd.setCursor(1, 0);  // adjust position
  lcd.print("HUMID:");
  lcd.setCursor(10, 0);
  lcd.print(h);
  lcd.setCursor(15, 0);  // adjust position
  lcd.print("%");
  lcd.setCursor(1, 1);  // adjust position
  lcd.print("MinBound:");
  if (HumidBound.getStatus(h) < 0) {
    lcd.setCursor(0, 1);  // adjust position
    lcd.print("*MinBound:");
  }
  lcd.print(HumidBound.lowerBound);
  lcd.setCursor(1, 2);
  lcd.print("MaxBound:");
  if (HumidBound.getStatus(h) > 0) {
    lcd.setCursor(0, 2);  // adjust position
    lcd.print("*MaxBound:");
  }
  lcd.print(HumidBound.upperBound);
  lcd.display();
  deactivateWarning();
  // Serial.print("Humidity Bounds: ");
  // Serial.print(H[0]);
  // Serial.print(" - ");
  // Serial.println(H[1]);
}
//Warning triggers lights and
void activateWarning() {
  if ((millis() / 1000) % 2 == 0) {
    digitalWrite(22, HIGH);
    tone(buzzer, 1000);
  } else {
    digitalWrite(22, LOW);
    tone(buzzer, 500);
  }
}
void deactivateWarning() {
  digitalWrite(22, LOW);
  noTone(buzzer);
}

void loop() {
  // Read data from MAX30105
  irValue = particleSensor.getIR();
  // ExternalBodyTemp = particleSensor.readTemperature(); // Body temperature in Celsius
  // Check for a beat and calculate BPM
  if (checkForBeat(irValue) == true) {
    long deltaHeartBeat = millis() - lastBeat;
    lastBeat = millis();
    BPM = 60 / (deltaHeartBeat / 1000.0);
  }
  // Dynamic signal threshold logic
  Signal = irValue;  // Use IR value as the signal input for dynamic processing
  int oldValue = threshholdPts[iteratorLoc];
  double total = 0;
  for (int i = 0; i < dataPts; i++) {
    total += threshholdPts[i];
  }
  total = total - oldValue + Signal;
  double newThreshholdAvg = total / dataPts;
  threshholdPts[iteratorLoc] = Signal;
  iteratorLoc = (iteratorLoc + 1) % dataPts;
  threshholdAvg = newThreshholdAvg;
  // Calculate upperThresholdAvg
  int count = 0;
  double sumAboveThreshold = 0.0;
  for (int i = 0; i < dataPts; i++) {
    if (threshholdPts[i] > threshholdAvg) {
      sumAboveThreshold += threshholdPts[i];
      count++;
    }
  }
  upperThresholdAvg = (count > 0) ? sumAboveThreshold / count : 0.0;
  // BPM calculation using peaks
  if (Signal > upperThresholdAvg) {
    if (!beat) {
      peakTimes[peakIndex] = millis();
      peakIndex = (peakIndex + 1) % peakWindowSize;
      beat = true;
      digitalWrite(LED, HIGH);  // Turn on LED on beat
    }
  } else {
    if (beat) {
      digitalWrite(LED, LOW);  // Turn off LED
      beat = false;
    }
  }
  if (peakIndex > 1) {
    unsigned long totalInterval = 0;
    int peakCount = 0;
    for (int i = 0; i < peakWindowSize - 1; i++) {
      int currentIndex = (peakIndex - 1 - i + peakWindowSize) % peakWindowSize;
      int previousIndex = (peakIndex - 2 - i + peakWindowSize) % peakWindowSize;
      totalInterval += peakTimes[currentIndex] - peakTimes[previousIndex];
      peakCount++;
    }
    if (peakCount > 0) {
      float newBPM = 60000.0 / (totalInterval / peakCount);
      bpm = newBPM;  // Update bpm no matter the value
      // Update rates array with new bpm
      rates[rateSpot++] = (uint8_t)bpm;
      rateSpot %= RATE_SIZE;  // Wrap around index for circular buffer
      // Calculate average BPM from the last 10 values in rates[]
      avgBPM = 0;
      for (uint8_t i = 0; i < RATE_SIZE; i++) {
        avgBPM += rates[i];
      }
      avgBPM /= RATE_SIZE;
    }
  }
  //This reads data from the sensor
  t = sht31.readTemperature();
  h = sht31.readHumidity();
  // helps determine if the buttons are being pressed or not
  humidSwitchState = digitalRead(BUTTON_HUMID_BUTTON);
  homeSwitchState = digitalRead(HOME_SCREEN_BUTTON);
  tempSwitchState = digitalRead(BUTTON_TEMP_BUTTON);
  BPMSwitchState = digitalRead(BUTTON_BPM);
  EXTSwitchState = digitalRead(BUTTON_EXTTEMP);

  // Get the skin temperature and calculate estimated core temperature
  float skinTempC = 0;
  if ((millis() / 100) % 20 == 0) {
    skinTempC = sts.getTemperaturePeriodC();
    // delay(100);
    float coreTempC = skinTempC + 2.0;
    // Store the core temperature in the array
    coreTempReadings[currentIndex] = coreTempC;
    currentIndex++;
    // Reset index and mark array as filled once we reach the end
    if (currentIndex >= CORE_TEMP_NUM_SAMPLES) {
      currentIndex = 0;
      isArrayFilled = true;
    }
    // Calculate the rolling average
    float sum = 0.0;
    int count1 = isArrayFilled ? CORE_TEMP_NUM_SAMPLES : currentIndex;  // Use only filled elements
    for (int i = 0; i < count1; i++) {
      sum += coreTempReadings[i];
    }
    rollingAvgTempC = sum / count1;
  }
  float rollingAvgTempF = (rollingAvgTempC * 1.8) + 32;

  if (homeSwitchState != LOW || ok >= 3) {
    ok = 0;
    currentScreen = HOME_SCREEN;
    updateHomeScreen(t, h);
  } else if (tempSwitchState != LOW) {
    currentScreen = TEMPERATURE_SCREEN;
    ok = 1;
    updateTempScreen();
  } else if (humidSwitchState != LOW) {
    currentScreen = HUMIDITY_SCREEN;
    ok = 1;
    updateHumidScreen();
  } else if (EXTSwitchState != LOW) {
    currentScreen = EXTTEMP_SCREEN;
    ok = 1;
    updateEXTTempScreen();
  } else if (BPMSwitchState != LOW) {
    currentScreen = BPM_SCREEN;
    ok = 1;
    updateBPMScreen();
  }
  if (currentScreen == HOME_SCREEN && ((millis() / 500) % 10 == 0)) {
    updateHomeScreen(t, h);
    //Serial.print("Hello");
  } else if (currentScreen == TEMPERATURE_SCREEN && ((millis() / 100) % 10 == 0)) {
    updateTempScreen();
  } else if (currentScreen == HUMIDITY_SCREEN && ((millis() / 100) % 10 == 0)) {
    updateHumidScreen();
  } else if (currentScreen == EXTTEMP_SCREEN && ((millis() / 100) % 10 == 0)) {
    updateEXTTempScreen();
  } else if (currentScreen == BPM_SCREEN && ((millis() / 100) % 10 == 0)) {
    updateBPMScreen();
  }

  // Check if the internal and external temperature and humidity are outside the allowed bounds

  if (!TempBound.getStatus(t) && !HumidBound.getStatus(h) && !InfExtTempBound.getStatus(rollingAvgTempC) && !InfBPMBound.getStatus(avgBPM)) {
    deactivateWarning();
  } else {
    activateWarning();  //lets talk about this
    if (TempBound.getStatus(t) > 0 || InfExtTempBound.getStatus(rollingAvgTempC) > 0) {
      digitalWrite(24, HIGH);  //fan
    } else {
      digitalWrite(24, LOW);
    }
    if (TempBound.getStatus(t) < 0 || InfExtTempBound.getStatus(rollingAvgTempC) < 0) {
      digitalWrite(HEAT_PIN, 255);  //heater
    } else {
      digitalWrite(HEAT_PIN, 0);
    }
    if (HumidBound.getStatus(h) < 0) {
      digitalWrite(29, HIGH);  //humidifier
    } else {
      digitalWrite(29, LOW);
    }
  }
}
