#include <Arduino.h>
#include <Wire.h>
#include <LCD-I2C.h>
#include "Adafruit_SHT31.h"
#include "MAX30105.h"
#include "heartRate.h"
#include <stdint.h>
#define HEAT_PIN 15
#define BUTTON_HUMID_BUTTON 8 // Digital pin for humidity screen button
#define HOME_SCREEN_BUTTON 11  // Digital pin for home screen
#define BUTTON_TEMP_BUTTON 12  // Digital pin for temperature screen
#define BUTTON_DOWN 36        // Digital pin for the down button
#define BUTTON_UP 33          // Digital pin for the up button
#define BUTTON_OK 6           // Digital pin for the ok button
#define BUTTON_BPM 9
#define BUTTON_EXTTEMP 10
MAX30105 particleSensor;
//#define BUTTON_OK 37 hello
LCD_I2C lcd = LCD_I2C(0x27, 20, 4); // Default address of most PCF8574 modules, change according
                           // 1.54" 200x200 Tricolor EPD with SSD1681 chipset
                           // ThinkInk_154_Tricolor_Z90 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
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
const uint8_t RATE_SIZE = 20; // # of HR samples to average
uint8_t rates[RATE_SIZE];   // Array of heart rates
uint8_t rateSpot = 0;       // array position tracker for
long lastBeat = 0;          // Time at which last beat occured
long irValue;               // The sensor's infared value
long deltaHeartBeat;        // Time between heart bears
float BPM;                  // HR in beats per minute
int32_t avgBPM;
float t;                     //The read temperature
float h;                     //The read humidity
float T[2] = { 20.0, 30.0 }; //initial temp bounds
float H[2] = { 40.0, 60.0 }; //initial humid bounds
float AVGBPM[2] = { 60.0, 120.0 };
float EXT[2] = { 0.0, 200.0 };
float ok = 0;   //counter for ok button
int screen = 0; //0 is home screen, 1 is temperature screen, 2 is humidity
float lastScreenChange = 0;
float lastIncrement = 0;
float ExternalBodyTemp;     // External body temperature
void doEncoder() {
  state = (digitalRead(EncoderPinA) << 1) | digitalRead(EncoderPinB);
  if (state != prevstate) {
    if (state == nextEncoderState[prevstate]) {
      EncoderPos = 1;
    } else if (state == prevEncoderState[prevstate]) {
      EncoderPos = -1;
    }
    Serial.println(EncoderPos, DEC);
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
  Serial.begin(9600);
  if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found.");
    while (1);
  }
  particleSensor.setup();                    //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);    //Turn off Red LED
  particleSensor.setPulseAmplitudeGreen(0);  //Turn off Green LED
  particleSensor.enableDIETEMPRDY();         //Enable the temp ready interrupt
  pinMode(BUTTON_HUMID_BUTTON, INPUT_PULLDOWN); //establishes connection of button
  pinMode(HOME_SCREEN_BUTTON, INPUT_PULLDOWN);  //establishes connection of button
  pinMode(BUTTON_TEMP_BUTTON, INPUT_PULLDOWN);  //establishes connection of button
  pinMode(BUTTON_BPM, INPUT_PULLDOWN);
  pinMode(BUTTON_EXTTEMP, INPUT_PULLDOWN);
  pinMode(EncoderPinA, INPUT);
  pinMode(EncoderPinB, INPUT);
  pinMode(BUTTON_OK, INPUT);
  pinMode(24, OUTPUT);
  pinMode(HEAT_PIN, OUTPUT);
  digitalWrite(6, HIGH);
  // pinMode(6, OUTPUT);
  // digitalWrite(6, LOW);
  digitalWrite(EncoderPinA, HIGH);
  digitalWrite(EncoderPinB, HIGH);
  attachInterrupt(0, doEncoder, CHANGE);
  attachInterrupt(1, doEncoder, CHANGE);
  attachInterrupt(BUTTON_OK, incrementScreen, FALLING);
  while (!Serial) { // helps establish that sensor and display are working this information will show in the serial monitor
    delay(10);
    Serial.print("No FOOL");
  }
  // Serial.println("Adafruit EPD full update test in red/black/white");
  // display.begin(THINKINK_TRICOLOR); //setup display?
  lcd.begin(&Wire);
  lcd.display();
  lcd.backlight();
  Serial.println("SHT31 test");
  if (!sht31.begin(0x44)) { // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  Serial.print("Heater Enabled State: ");
  if (sht31.isHeaterEnabled())
    Serial.println("ENABLED");
  else
    Serial.println("DISABLED");
  pinMode(buzzer, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
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
  lcd.print(BPM);
  if (BPM < AVGBPM[0]) {
    lcd.setCursor(0, 1); // adjust position
    lcd.print("*MinBound:");} 
  lcd.setCursor(1, 1); // adjust position
  lcd.print("MinBound:");
  lcd.print(AVGBPM[0]);
  lcd.setCursor(1, 2);
  lcd.print("MaxBound:");
  if (BPM > AVGBPM[1]) {
    lcd.setCursor(0, 2); // adjust position
    lcd.print("*MaxBound:");} 
  lcd.print(AVGBPM[1]);
  lcd.display();
  deactivateWarning();
  Serial.print("BPM Bounds: ");
  Serial.print(AVGBPM[0]);
  Serial.print(" - ");
  Serial.println(AVGBPM[1]);
}
void updateEXTTempScreen() {
  lcd.clear();
   lcd.setCursor(1, 0); // adjust position
  lcd.print("Ext Temp:");
   lcd.setCursor(10,0);
  lcd.print(ExternalBodyTemp);
    if (ExternalBodyTemp < EXT[0]) {
    lcd.setCursor(0, 1); // adjust position
    lcd.print("*MinBound:");} 
  lcd.setCursor(1, 1); // adjust position
  lcd.print("MinBound:");
  lcd.print(EXT[0]);
  lcd.setCursor(1, 2);
  lcd.print("MaxBound:");
  if (ExternalBodyTemp > EXT[1]) {
    lcd.setCursor(0, 2); // adjust position
    lcd.print("*MaxBound:");} 
  lcd.print(EXT[1]);
  lcd.display();
  deactivateWarning();
  Serial.print("Ext Temp Bounds: ");
  Serial.print(EXT[0]);
  Serial.print(" - ");
  Serial.println(EXT[1]);
}
//Updates the homescreen with latest data plus displaying it
void updateHomeScreen(int t, int h) {
lcd.clear();
lcd.setCursor(1, 0); // adjust position
  lcd.print("Temp:");
   if (t < T[0] || t > T[1]) {
    lcd.setCursor(0, 0); // adjust position
    lcd.print("*Temp:");} 
  lcd.setCursor(1, 1); // adjust position
  lcd.print(t);
  lcd.setCursor(1, 2); // adjust position
  lcd.print("Humid%");
   if (h < H[0] || h > H[1]) {
    lcd.setCursor(0, 2); // adjust position
    lcd.print("*Humid%");} 
  lcd.setCursor(1, 3); // adjust position
  lcd.print(h);
  lcd.setCursor(9,0);
  lcd.print("INF BPM:");
  if (BPM < AVGBPM[0] || BPM > AVGBPM[1]) {
    lcd.setCursor(9, 0); // adjust position
    lcd.print("*INF BPM:");} 
  lcd.setCursor(10,1);
  lcd.print(BPM);
  lcd.setCursor(10,2);
  lcd.print("INF TEMP:");
  if (ExternalBodyTemp < EXT[0] || ExternalBodyTemp > EXT[1]) {
    lcd.setCursor(9, 2); // adjust position
    lcd.print("*INF TEMP");} 
  lcd.setCursor(10,3);
  lcd.print(ExternalBodyTemp);
  lcd.display();
}
//Updates the temperature screen plus displaying it
void updateTempScreen() {
  lcd.clear();
  lcd.setCursor(1, 0); // adjust position
  lcd.print("Temp:");
   lcd.setCursor(10,0);
  lcd.print(t);
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
  Serial.print("Temperature Bounds: ");
  Serial.print(T[0]);
  Serial.print(" - ");
  Serial.println(T[1]);
}
//Updates the humidity screen plus displaying it
void updateHumidScreen() {
  lcd.clear();
  lcd.setCursor(1, 0); // adjust position
  lcd.print("Humid:");
   lcd.setCursor(7,0);
  lcd.print(h);
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
  Serial.print("Humidity Bounds: ");
  Serial.print(H[0]);
  Serial.print(" - ");
  Serial.println(H[1]);
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
  // calculate HR value
  irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    deltaHeartBeat = millis() - lastBeat;
    lastBeat = millis();
    BPM = 60 / (deltaHeartBeat / 1000.0);
    if (BPM < 255 && BPM > 20)
    {
      rates[rateSpot++] = (uint8_t) BPM; //Store this reading in the array
      rateSpot %= RATE_SIZE;            //Wrap array position tracker
      //Take average of readings
      avgBPM = 0;
      for (uint8_t i = 0 ; i < RATE_SIZE ; i++)
      {
        avgBPM += rates[i];
      }
      avgBPM /= RATE_SIZE;
    }
  }
  ExternalBodyTemp = particleSensor.readTemperature(); // get body temp in C
  // Print results to terminal
  Serial.print("ExternalBodyTemp [C] = ");
  Serial.print(ExternalBodyTemp, 4);
  Serial.print(", BPM=");
  Serial.print(BPM);
  Serial.print(", Avg BPM=");
  Serial.print(avgBPM);
  if (irValue < 50000)
  {
    Serial.print(" No finger?"); // most likely no finger on sensor
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
  //Helps show on serial monitor that sensor is working properly
  if (!isnan(t)) { // check if 'is not a number'
    Serial.print("Temp *C = ");
    Serial.print(t);
    Serial.print("\t\t");
  } else {
    Serial.println("Failed to read temperature");
  }
  if (!isnan(h)) { // check if 'is not a number'
    Serial.print("Hum. % = ");
    Serial.println(h);
  } else {
    Serial.println("Failed to read humidity");
  }

  // delay(1000);
  // Toggle heater enabled state every 30 seconds
  // An ~3.0 degC temperature increase can be noted when heater is enabled
  if (loopCnt >= 30) {
    enableHeater = !enableHeater;
    sht31.heater(enableHeater);
    Serial.print("Heater Enabled State: ");
    if (sht31.isHeaterEnabled())
      Serial.println("ENABLED");
    else
      Serial.println("DISABLED");
    loopCnt = 0;
  }
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
  if (screen == HOME_SCREEN && ((millis() / 100) % 10 == 0)) {
    updateHomeScreen(t, h);
    Serial.print("Hello");
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
  if (t < T[0] || t > T[1]) {
    activateWarning();
    if (t>T[1]){
      digitalWrite(24, HIGH); //fan
    }
    else{
      digitalWrite(24,LOW);
    }
    if (t<T[0]){
      digitalWrite(HEAT_PIN, 255); //heater
    }
    else{
      digitalWrite(HEAT_PIN,0);
    }
  }
  else{
    digitalWrite(24,LOW);
    if(h < H[0] || h > H[1]) {
    activateWarning();
    }
    else if(ExternalBodyTemp < EXT[0] || ExternalBodyTemp > EXT[1]) {
      activateWarning();
    }
    else if(BPM < AVGBPM[0] || BPM > AVGBPM[1]) {
      activateWarning();
    } 
    else {
      deactivateWarning();
  }

  }
  
}