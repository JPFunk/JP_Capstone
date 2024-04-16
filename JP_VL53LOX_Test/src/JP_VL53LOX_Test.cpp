/* 
 * Project JP VL53LOX test
 * Author: JP Funk
 * Date: 04/15/2024 Monday Build of TOF Functionality w/ 3 LEDs
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_VL53L0X.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_BME280.h"
#include "Button.h"
// OLED
const int OLED_RESET=-1;
int rot;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int TOF;
int targetLoc1, targetLoc2, targetLoc3, targetLoc0, prevTargetLoc;
bool position;
// Button
//const int BUTTONPIN = D3;
void buttonisClicked();
bool changed;
bool ledOnOff;
bool ledOnOff2;
bool ledOnOff3;
//void ledOn (int waterPumpPin);
const int LEDPIN = D6; // LED pin  for TOF Location 1 Water Pump
const int PUMPIN = D6;
const int LEDPIN2 = D5; // LED pin for TOF Location 2
const int LEDPIN3 = D4; // LED pin for TOF Location 3

void pumpOn (int waterPumpPin);
// Millis Timer
const int sampleTime = 1000; //3000
unsigned int duration, startTime;
// Adafruit BME
Adafruit_BME280 bme;
const char degree = 0xf8;
const int delayTime =1000;
bool status;
int hexAddress, startime;
float tempC, pressPA, humidRH, tempF, inHG;

Adafruit_SSD1306 display(OLED_RESET);
// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// setup() runs once, when the device is first turned on
void setup() {
  waitFor(Serial.isConnected, 5000);
  Serial.begin(115200);

//intialize OLED display
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
display.display();
rot = 0;
display.setRotation(rot);
display.setTextSize(4);
display.setTextColor(WHITE);
display.setCursor(0,0);
display.clearDisplay();
delay(1000);
  
  Wire.begin();

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
// Button
//pinMode(BUTTONPIN, INPUT);
// LED
pinMode (LEDPIN, OUTPUT);
pinMode (LEDPIN2, OUTPUT);
pinMode (LEDPIN3, OUTPUT);
pinMode (PUMPIN, OUTPUT);
// Millis Startime
startTime = 0;
}
// loop() runs over and over again, as quickly as it can execute.
void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
if ((millis()-startTime) > sampleTime)
  if (targetLoc1 != prevTargetLoc){
  Serial.printf("%i\n", position);
   if (targetLoc1 == TRUE) {
      ledOnOff= !ledOnOff;
      } prevTargetLoc = targetLoc1;
      digitalWrite(LEDPIN, ledOnOff);
       Serial.printf("LED1%i\n", targetLoc1);
       startTime = millis();
    }
if ((millis()-startTime) > sampleTime)
  if (targetLoc2 != prevTargetLoc){
  Serial.printf("%i\n", position);
   if (targetLoc2 == TRUE){
      ledOnOff2= !ledOnOff2;
      } prevTargetLoc = targetLoc2;
      digitalWrite(LEDPIN2, ledOnOff2);
       Serial.printf("LED2 On%i\n",targetLoc2);
      startTime = millis();
    }

if ((millis()-startTime) > sampleTime)
  if (targetLoc3 != prevTargetLoc){
  Serial.printf("%i\n", position);
   if (targetLoc3 == TRUE){
      ledOnOff3= !ledOnOff3;
      } prevTargetLoc = targetLoc3;
      digitalWrite(LEDPIN3, ledOnOff3);
       Serial.printf("LED3 On%i\n", targetLoc3);
       startTime = millis();
    }

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    if(measure.RangeMilliMeter <= 75){
      targetLoc1 =TRUE;
      targetLoc2 =FALSE; targetLoc3 =FALSE; targetLoc0 =FALSE;
      Serial.printf("targetPos 1%i\n", targetLoc1);
    }
   else if(measure.RangeMilliMeter > 75 && measure.RangeMilliMeter <= 150){
      targetLoc2 =TRUE;
      targetLoc1 =FALSE; targetLoc3 =FALSE; targetLoc0 =FALSE;
      Serial.printf("targetPos 2%i\n", targetLoc2);
    }
   else if(measure.RangeMilliMeter > 150 && measure.RangeMilliMeter <= 225){
      targetLoc3 =TRUE;
      targetLoc1 =FALSE; targetLoc2 =FALSE; targetLoc0 =FALSE;
      Serial.printf("targetPos 3%i\n", targetLoc3);
    }
  } else {
    targetLoc0 =TRUE;
    targetLoc1 =FALSE; targetLoc2 =FALSE; targetLoc3 =FALSE;  
    Serial.printf("targetPos 0%i\n", targetLoc0);
  } delay(500);
}

void pumpOn (int waterPumpPin) { // Water Pump OnOff function with serial print values
  digitalWrite(waterPumpPin, HIGH);
  Serial.printf("Pump On%i\n", pumpOn);
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.printf(" Pump !On!", pumpOn);
  digitalWrite(waterPumpPin, LOW);
  display.display();
 // delay (3000);
}
  // TOF Measure Range Status code
  // if (measure.RangeStatus != 4) {  // phase failures have incorrect data
  //     display.clearDisplay();
  //     display.setCursor(0,0);
  //     display.setTextSize(3);
  //     display.print(measure.RangeMilliMeter);
  //     display.print("mm");
  //     display.display();
  //     Serial.println();
  //     delay(50);
  // } else {
  //   display.clearDisplay();
  //   display.setCursor(0,0);
  //   display.setTextSize(3);
  //   display.print("Out of Range");
  //   display.display();
  //   return;
  // }