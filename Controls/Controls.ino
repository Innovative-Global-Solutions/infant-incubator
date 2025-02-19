#include <Arduino.h>
#include <Wire.h>
#include <LCD-I2C.h>
#include "Adafruit_SHT31.h"
#include "MAX30105.h"
#include "heartRate.h"
#include <stdint.h>
#include "DFRobot_STS3X.h"
#define numSamples 10  // Number of samples for the rolling average
DFRobot_STS3X sts(&Wire, STS3X_I2C_ADDRESS_B);
float coreTempReadings[numSamples]; // Array to store the last temperature readings
int currentIndex = 0;  // Current index for storing the reading
bool isArrayFilled = false;  // Flag to indicate when the array is fully populated
#define HEAT_PIN 14
#define BUTTON_HUMID_BUTTON 11 // Digital pin for humidity screen button
#define HOME_SCREEN_BUTTON 8  // Digital pin for home screen

#define BUTTON_TEMP_BUTTON 12  // Digital pin for temperature screen
#define BUTTON_DOWN 36        // Digital pin for the down button
#define BUTTON_UP 33          // Digital pin for the up button
#define BUTTON_OK 6           // Digital pin for the ok button
#define BUTTON_BPM 9
#define BUTTON_EXTTEMP 28
MAX30105 particleSensor;
//#define BUTTON_OK 37 hello
LCD_I2C lcd = LCD_I2C(0x27, 20, 4); // Default address of most PCF8574 modules, change according
                           // 1.54" 200x200 Tricolor EPD with SSD1681 chipset
                           // ThinkInk_154_Tricolor_Z90 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

const uint8_t RATE_SIZE = 4;     // Number of HR samples to average
uint8_t rates[RATE_SIZE];         // Array of heart rates
uint8_t rateSpot = 0;             // Array position tracker
long lastBeat = 0;                // Time of the last beat
float BPM;                        // HR in beats per minute
float avgBPM;                     // Average BPM
long irValue;                     // Sensor's infrared value
//float ExternalBodyTemp;           // External body temperature
// Pulse sensor dynamic qualities
const int PulseSensorPurplePin = 0; // Analog pin for pulse sensor (if used in parallel)
int LED = LED_BUILTIN;             // Onboard Arduino LED
int Signal;                        // Raw signal data
int ogAvg = 515;
int threshholdAvg = ogAvg;         // Starting threshold average
int dataPts = 10;
int threshholdPts[10];             // Array for moving average
bool beat = false;
int iteratorLoc = 0;               // Moving window index
float upperThresholdAvg = 0.0;     // Average of values above the threshold
unsigned long peakTimes[8];        // Peak timestamps for BPM calculation
int peakIndex = 0;                 // Peak array index
int peakWindowSize = 8;            // Window size for peak timestamps
float bpm = 0.0;                   // Calculated BPM

bool enableHeater = false;
uint8_t loopCnt = 0;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
unsigned long timer = 0;
const int buzzer = 7; //buzzer to arduino pin 9
const int HOME_SCREEN = 0;
const int HUMIDITY_SCREEN = 1;
const int TEMPERATURE_SCREEN = 2;
const int BPM_SCREEN = 3;
const int EXTTEMP_SCREEN = 4;
// below are the switch states for the bounds screen they help determine state of the button (high or low) to determine if screenswitching is necessary.
int humidSwitchState = 0;
int homeSwitchState = 0;
int tempSwitchState = 0;
int BPMSwitchState = 0;
int EXTSwitchState = 0;
int bpmSwitchState=0;
int exbtempSwitchState=0;
int i; // Counter for min/max
int state, prevstate = 0, count = 0;
int nextEncoderState[4] = { 2, 0, 3, 1 };
int prevEncoderState[4] = { 1, 3, 0, 2 };
int EncoderPinA = 0;
int EncoderPinB = 1;
int EncoderPos = 0;
float t;                     //The read temperature
float h;                     //The read humidity
float T[2] = { 20.0, 30.0 }; //initial temp bounds
float H[2] = { 40.0, 60.0 }; //initial humid bounds
float AVGBPM[2] = { 60.0, 100.0 };
float EXT[2] = { 15.0, 40.0 };
float ok = 0;   //counter for ok button
int screen = 0; //0 is home screen, 1 is temperature screen, 2 is humidity
float lastScreenChange = 0;
float lastIncrement = 0;
// float ExternalBodyTemp;     // External body temperature
float rollingAvgTempC = 0.0;
bool oob = false;

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
    if (screen == TEMPERATURE_SCREEN) {
      if (ok == 1) {
        T[0] += EncoderPos;
      } else if (ok == 2) {
        T[1] += EncoderPos;
      }
    } else if (screen == HUMIDITY_SCREEN) {
      if (ok == 1) {
        H[0] += EncoderPos;
      } else if (ok == 2) {
        H[1] += EncoderPos;
      }
    } else if (screen == BPM_SCREEN) {
      if (ok == 1) {AVGBPM[0] += EncoderPos;
      } else if (ok == 2) {
        AVGBPM[1] += EncoderPos;
      }
    } else if (screen == EXTTEMP_SCREEN) {
      if (ok == 1) {
        EXT[0] += EncoderPos;
      } else if (ok == 2) {
        EXT[1] += EncoderPos;
      }
    }
    lastIncrement = millis();
  }
}
void setup() {
   while(sts.begin() != true){
       // Serial.println("Failed to init chip, please check if the chip connection is fine.");
        delay(1000);
    }
   // Serial.println("Begin ok!");
    sts.setFreq(sts.e10Hz);
    // Initialize the array with zeros
    for (int i = 0; i < numSamples; i++) {
        coreTempReadings[i] = 0.0;
    }
  if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) //Use default I2C port, 400kHz speed
  {
    // Serial.println("MAX30105 was not found.");
    while (1);
  }
  particleSensor.setup();                    //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);    //Turn off Red LED
  particleSensor.setPulseAmplitudeGreen(0);  //Turn off Green LED
  particleSensor.enableDIETEMPRDY();         //Enable the temp ready interrupt
    // Initialize threshold array for dynamic qualities
  for (int i = 0; i < dataPts; i++) {
    threshholdPts[i] = ogAvg;
  }
  // Initialize rates array to 0
  memset(rates, 0, sizeof(rates));
  pinMode(LED, OUTPUT);
  pinMode(BUTTON_HUMID_BUTTON, INPUT_PULLDOWN); //establishes connection of button
  pinMode(HOME_SCREEN_BUTTON, INPUT_PULLDOWN);  //establishes connection of button
  pinMode(BUTTON_TEMP_BUTTON, INPUT_PULLDOWN);  //establishes connection of button
  pinMode(BUTTON_BPM, INPUT_PULLDOWN);
  pinMode(BUTTON_EXTTEMP, INPUT_PULLDOWN);
  pinMode(EncoderPinA, INPUT);
  pinMode(EncoderPinB, INPUT);
  pinMode(BUTTON_OK, INPUT);
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(26, OUTPUT); // Set pin 9 as an output
  pinMode(24, OUTPUT);
  pinMode(29, OUTPUT);
  digitalWrite(6, HIGH);
  // pinMode(6, OUTPUT);
  // digitalWrite(6, LOW);
  digitalWrite(EncoderPinA, HIGH);
  digitalWrite(EncoderPinB, HIGH);
  attachInterrupt(0, doEncoder, CHANGE);
  attachInterrupt(1, doEncoder, CHANGE);
  attachInterrupt(BUTTON_OK, incrementScreen, FALLING);
  // while (!Serial) { // helps establish that sensor and display are working this information will show in the serial monitor
  //   delay(10);
    // Serial.print("No FOOL");
  // }
  // Serial.println("Adafruit EPD full update test in red/black/white");
  // display.begin(THINKINK_TRICOLOR); //setup display?
  lcd.begin(&Wire);
  lcd.display();
  lcd.backlight();
  // Serial.println("SHT31 test");
  if (!sht31.begin(0x44)) { // Set to 0x45 for alternate i2c addr
    // Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  // Serial.print("Heater Enabled State: ");
  // if (sht31.isHeaterEnabled())
    // Serial.println("ENABLED");
  // else
    // Serial.println("DISABLED");
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
  lcd.setCursor(1, 0); // adjust position
  lcd.print("BPM:");
    lcd.setCursor(10,0);
  lcd.print(avgBPM);
  if (avgBPM < AVGBPM[0]) {
    lcd.setCursor(0, 1); // adjust position
    lcd.print("*MinBound:");} 
  lcd.setCursor(1, 1); // adjust position
  lcd.print("MinBound:");
  lcd.print(AVGBPM[0]);
  lcd.setCursor(1, 2);
  lcd.print("MaxBound:");
  if (avgBPM > AVGBPM[1]) {
    lcd.setCursor(0, 2); // adjust position
    lcd.print("*MaxBound:");} 
  lcd.print(AVGBPM[1]);
  lcd.display();
  deactivateWarning();
  // Serial.print("BPM Bounds: ");
  // Serial.print(AVGBPM[0]);
  // Serial.print(" - ");
  // Serial.println(AVGBPM[1]);
}
void updateEXTTempScreen() {
  lcd.clear();
   lcd.setCursor(1, 0); // adjust position
  lcd.print("INF TEMP:");
   lcd.setCursor(10,0);
  lcd.print(rollingAvgTempC);
   lcd.setCursor(15,0);
    lcd.print(char(223));
    lcd.setCursor(16,0);
    lcd.print("C");
    if (rollingAvgTempC < EXT[0]) {
    lcd.setCursor(0, 1); // adjust position
    lcd.print("*MinBound:");} 
  lcd.setCursor(1, 1); // adjust position
  lcd.print("MinBound:");
  lcd.print(EXT[0]);
  lcd.setCursor(1, 2);
  lcd.print("MaxBound:");
  if (rollingAvgTempC > EXT[1]) {
    lcd.setCursor(0, 2); // adjust position
    lcd.print("*MaxBound:");} 
  lcd.print(EXT[1]);
  lcd.display();
  deactivateWarning();
  // Serial.print("Ext Temp Bounds: ");
  // Serial.print(EXT[0]);
  // Serial.print(" - ");
  // Serial.println(EXT[1]);
}
//Updates the homescreen with latest data plus displaying it
void updateHomeScreen(int t, int h ) {
lcd.clear();
lcd.setCursor(1, 0); // adjust position
  lcd.print("TEMP:");
   if (t < T[0] || t > T[1]) {
    lcd.setCursor(0, 0); // adjust position
    lcd.print("*TEMP:");} 
  lcd.setCursor(1, 1); // adjust position
  lcd.print(t);
  lcd.setCursor(3,1);
  lcd.print(char(223));
  lcd.setCursor(4,1);
  lcd.print("C");
  lcd.setCursor(1, 2); // adjust position
  lcd.print("HUMID:");
   if (h < H[0] || h > H[1]) {
    lcd.setCursor(0, 2); // adjust position
    lcd.print("*HUMID:");} 
  lcd.setCursor(1, 3); // adjust position
  lcd.print(h);
  lcd.setCursor(3, 3); // adjust position
  lcd.print("%");
  lcd.setCursor(10,0);
  lcd.print("INF BPM:");
  if (avgBPM < AVGBPM[0] || avgBPM > AVGBPM[1]) {
    lcd.setCursor(9, 0); // adjust position
    lcd.print("*INF BPM:");} 
  lcd.setCursor(10,1);
  lcd.print(avgBPM);
  lcd.setCursor(10,2);
  lcd.print("INF TEMP:");
  if (rollingAvgTempC < EXT[0] || rollingAvgTempC > EXT[1]) {
    lcd.setCursor(9, 2); // adjust position
    lcd.print("*INF TEMP");} 
    lcd.setCursor(15, 3); // adjust position
    lcd.print(char(223));
    lcd.setCursor(16,3); // adjust position
    lcd.print("C");
  lcd.setCursor(10,3);
  lcd.print(rollingAvgTempC);
  lcd.display();
}
//Updates the temperature screen plus displaying it
void updateTempScreen() {
  lcd.clear();
  lcd.setCursor(1, 0); // adjust position
  lcd.print("TEMP:");
   lcd.setCursor(10,0);
  lcd.print(t);
  lcd.setCursor(15,0);
  lcd.print(char(223));
    lcd.setCursor(16,0);
  lcd.print("C");
  lcd.setCursor(1, 1); // adjust position
  lcd.print("MinBound:");
    if (t < T[0]) {
    lcd.setCursor(0, 1); // adjust position
    lcd.print("*MinBound:");} 
  lcd.print(T[0]);
  lcd.setCursor(1, 2);
  lcd.print("MaxBound:");
  if (t > T[1]) {
    lcd.setCursor(0, 2); // adjust position
    lcd.print("*MaxBound:");} 
  lcd.print(T[1]);
  lcd.display();
  deactivateWarning();
  // Serial.print("Temperature Bounds: ");
  // Serial.print(T[0]);
  // Serial.print(" - ");
  // Serial.println(T[1]);
}
//Updates the humidity screen plus displaying it
void updateHumidScreen() {
  lcd.clear();
  lcd.setCursor(1, 0); // adjust position
  lcd.print("HUMID:");
   lcd.setCursor(10,0);
  lcd.print(h);
  lcd.setCursor(15, 0); // adjust position
  lcd.print("%");
  lcd.setCursor(1, 1); // adjust position
  lcd.print("MinBound:");
      if (h < H[0]) {
    lcd.setCursor(0, 1); // adjust position
    lcd.print("*MinBound:");} 
  lcd.print(H[0]);
  lcd.setCursor(1, 2);
  lcd.print("MaxBound:");
   if (h > H[1]) {
    lcd.setCursor(0, 2); // adjust position
    lcd.print("*MaxBound:");} 
  lcd.print(H[1]);
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
void loop() 
{
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
      digitalWrite(LED, HIGH); // Turn on LED on beat
    }
  } else {
    if (beat) {
      digitalWrite(LED, LOW); // Turn off LED
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
      bpm = newBPM; // Update bpm no matter the value
      // Update rates array with new bpm
      rates[rateSpot++] = (uint8_t)bpm;
      rateSpot %= RATE_SIZE; // Wrap around index for circular buffer
      // Calculate average BPM from the last 10 values in rates[]
      avgBPM = 0;
      for (uint8_t i = 0; i < RATE_SIZE; i++) {
        avgBPM += rates[i];
      }
      avgBPM /= RATE_SIZE;
    }
  }
  // Print results
  // Serial.print(irValue);
  // Serial.print(", ");
  // Serial.print(upperThresholdAvg);
  // Serial.print(", ");
  // Serial.println(threshholdAvg);
  // Serial.print("Avg BPM Components: ");
  for (uint8_t i = 0; i < RATE_SIZE; i++) {
    // Serial.print(rates[i]);
    // Serial.print(i < (RATE_SIZE - 1) ? ", " : "\n");
  }
 // Serial.print("Avg BPM: ");
  // Serial.print(avgBPM);
  // Serial.print(", ");
  // Serial.println(bpm);
  // calculate HR value
 // ExternalBodyTemp = particleSensor.readTemperature(); // get body temp in C
  // Print results to terminal
  // Serial.print("ExternalBodyTemp [C] = ");
  // Serial.print(ExternalBodyTemp, 4);
  // Serial.print(", BPM=");
  // Serial.print(BPM);
  // Serial.print(", Avg BPM=");
  // Serial.print(avgBPM);
  //This reads data from the sensor
  t = sht31.readTemperature();
  h = sht31.readHumidity();
  // helps determine if the buttons are being pressed or not
  humidSwitchState = digitalRead(BUTTON_HUMID_BUTTON);
  homeSwitchState = digitalRead(HOME_SCREEN_BUTTON);
  tempSwitchState = digitalRead(BUTTON_TEMP_BUTTON);
  BPMSwitchState = digitalRead(BUTTON_BPM);
  EXTSwitchState = digitalRead(BUTTON_EXTTEMP);
  //Helps show on serial monitor that sensor is working properly
  if (!isnan(t)) { // check if 'is not a number'
    // Serial.print("Temp *C = ");
    // Serial.print(t);
    // Serial.print("\t\t");
  } else {
    Serial.println("Failed to read temperature");
  }
  if (!isnan(h)) { // check if 'is not a number'
   // Serial.print("Hum. % = ");
   // Serial.println(h);
  } else {
    //Serial.println("Failed to read humidity");
  }

  // delay(1000);
  // Toggle heater enabled state every 30 seconds
  // An ~3.0 degC temperature increase can be noted when heater is enabled
  loopCnt++;
  // This is the calibration timer
  timer = millis() / 1000;
  // This is the screen switching to the home screen
  if (homeSwitchState != LOW || ok >= 3) {
    ok = 0;
    screen = HOME_SCREEN;
    updateHomeScreen(t, h);
  } else if (tempSwitchState != LOW) {
    screen = TEMPERATURE_SCREEN;
    ok = 1;
    updateTempScreen();
  } else if (humidSwitchState != LOW) {
    screen = HUMIDITY_SCREEN;
    ok = 1;
    updateHumidScreen();
  } else if (EXTSwitchState != LOW) {
    screen = EXTTEMP_SCREEN;
    ok = 1;
    updateEXTTempScreen();
  } else if (BPMSwitchState != LOW) {
    screen = BPM_SCREEN;
    ok = 1;
    updateBPMScreen();
  }
  if (screen == HOME_SCREEN && ((millis() / 500) % 10 == 0)) {
    updateHomeScreen(t, h);
    //Serial.print("Hello");
  } else if (screen == TEMPERATURE_SCREEN && ((millis() / 100) % 10 == 0)) {
    updateTempScreen();
  } else if (screen == HUMIDITY_SCREEN && ((millis() / 100) % 10 == 0)) {
    updateHumidScreen();
  } else if (screen == EXTTEMP_SCREEN && ((millis() / 100) % 10 == 0)) {
    updateEXTTempScreen();
  } else if (screen == BPM_SCREEN && ((millis() / 100) % 10 == 0)) {
    updateBPMScreen();
  }

  // Check if the internal and external temperature and humidity are outside the allowed bounds

  if (t > T[0] && t < T[1] && h >H[0] && h<H[1] && rollingAvgTempC > EXT[0] && rollingAvgTempC < EXT[1] && avgBPM > AVGBPM[0] && avgBPM < AVGBPM[1])
  {
    deactivateWarning();
  }

  else
  {
    activateWarning();
    if (h<H[0] && oob == false)
    {
      digitalWrite(26,LOW); //Humidifier
      delay(1000);
      digitalWrite(26,HIGH);
      delay(1000);
      digitalWrite(26,LOW);
      delay(1000);
      digitalWrite(26,HIGH);
      oob = true;
    }
    else if (h>H[0] && oob == true)
    {
      digitalWrite(26,LOW);
      delay(1000);
      digitalWrite(26,HIGH);
      oob = false;
    }
    if (t>((T[1]-T[0])*.66+T[0]) || rollingAvgTempC > ((EXT[1]-EXT[0])*.66+EXT[0]))
    {
      digitalWrite(24, HIGH); //fan
    }
    else
    {
      digitalWrite(24,LOW);
    }
    if (t<((T[1]-T[0])*.33+T[0]) || rollingAvgTempC < ((EXT[1]-EXT[0])*.33+EXT[0]))
    {
      digitalWrite(HEAT_PIN, 255); //heater
    }
    else
    {
      digitalWrite(HEAT_PIN,0);
    }
    // if (h>((H[1]-H[0])*.66+H[0]))
    // {
    //   digitalWrite(27, 255); //humidifier
    // }
    // else
    // {
    //   digitalWrite(27,LOW);
    // }
    if (h<((H[1]-H[0])*.33+H[0]))
    {
      digitalWrite(29, HIGH); //humidifier
    }
    else
    {
      digitalWrite(29,LOW);
    }
   // Get the skin temperature and calculate estimated core temperature
    float skinTempC = 0;
    if ((millis()/100)%20 == 0){
      skinTempC = sts.getTemperaturePeriodC();
      // delay(100);
      float coreTempC = skinTempC + 2.0;
      // Store the core temperature in the array
      coreTempReadings[currentIndex] = coreTempC;
      currentIndex++;
      // Reset index and mark array as filled once we reach the end
      if (currentIndex >= numSamples) {
          currentIndex = 0;
          isArrayFilled = true;
      }
      // Calculate the rolling average
      float sum = 0.0;
      int count1 = isArrayFilled ? numSamples : currentIndex; // Use only filled elements
      for (int i = 0; i < count1; i++) {
          sum += coreTempReadings[i];
      }
      rollingAvgTempC = sum / count1;
    }
    float rollingAvgTempF = (rollingAvgTempC * 1.8) + 32;
      // Serial.println(avgBPM);

    // Print temperatures
  //  Serial.print("Skin Temperature: ");
  //  Serial.print(skinTempC);
  //  Serial.print(" ℃ /n ");
  //  Serial.print((skinTempC * 1.8) + 32);
  //  Serial.println(" ℉");
  //  Serial.print("Estimated Core Temperature: ");
  //  Serial.print(coreTempC);
  //  Serial.print(" ℃ / ");
  //  Serial.print((coreTempC * 1.8) + 32);
  //  Serial.println(" ℉");
    //Serial.print("Rolling Average Core Temperature: ");
    // Serial.println(rollingAvgTempC);
   // Serial.print(" ℃  ");
    // Serial.print(rollingAvgTempF);
    // Serial.println("");
    //Serial.println(" ℉");
    }// delay(500);
}

