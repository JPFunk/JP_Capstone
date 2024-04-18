/* 
 * Project JP VL53LOX test
 * Author: JP Funk
 * Date: 04/18/2024 Thursda Build of TOF Functionality w/ 3 LEDs MP3 Player integration
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
#include "IotClassroom_CNM.h"
#include "IoTTimer.h"
#include "DFRobotDFPlayerMini.h"

// DFRobotMP3Player
DFRobotDFPlayerMini myDFPlayer;
Button nextButton(D0);
unsigned int lastSong;
void printDetail(uint8_t type, int value);
// OLED
const int OLED_RESET=-1;
int rot;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int TOF;
int targetLoc1, targetLoc2, targetLoc3, targetLoc4, targetLoc0, prevTargetLoc1, prevTargetLoc2, prevTargetLoc3, prevTargetLoc4;
bool position;
unsigned int rangeTime = 500;
// Button
//const int BUTTONPIN = D3;
void buttonisClicked();
bool changed;
bool ledOnOff, ledOnOff2, ledOnOff3, ledOnOff4;
//void ledOn (int waterPumpPin);
const int LEDPIN = D6; // LED pin  for TOF Location 1 Water Pump
const int PUMPIN = D6;
const int LEDPIN2 = D5; // LED pin for TOF Location 2
const int LEDPIN3 = D4; // LED pin for TOF Location 3

void pumpOn (int waterPumpPin);
// Millis Timer
const int sampleTime = 500; //3000
unsigned int duration, startTime;
// Adafruit BME
Adafruit_BME280 bme;
const char degree = 0xf8;
const int delayTime =1000;
bool status;
int hexAddress, startime;
float tempC, pressPA, humidRH, tempF, inHG;
// Millis Delay Timer
// void delayRange();
// const int DELAY = 500;
// Timer timer(DELAY, delayRange);

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
  Serial1.begin(9600);
  delay(1000);
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
//DFRobotMP3Player
  if (!myDFPlayer.begin(Serial1)) {  //Use softwareSerial to communicate with mp3.
    Serial.printf("Unable to begin:\n");
    Serial.printf("1.Please recheck the connection!\n");
    Serial.printf("2.Please insert the SD card!\n");
    while(true);
  }
  Serial.printf("DFPlayer Mini online.\n");
  
  myDFPlayer.volume(30);  //Set volume value. From 0 to 30
  myDFPlayer.loop(1);  //Play the first mp3
  myDFPlayer.enableLoopAll();

// Millis Startime
startTime = millis();
//startTime = 0;
}
// loop() runs over and over again, as quickly as it can execute.
void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

if ((millis()-startTime) > sampleTime) // TOF Level 2 Range 0-50mm
  if (targetLoc1 != prevTargetLoc1){
  Serial.printf("%i\n", position);
  startTime = millis();
   if (targetLoc1 == TRUE) {
      ledOnOff= !ledOnOff;
      } prevTargetLoc1 = targetLoc1;
      digitalWrite(LEDPIN, ledOnOff);
      Serial.printf("LED1%i\n", targetLoc1); 
    }

if ((millis()-startTime) > sampleTime) // TOF Level 2 Range 5-115mm
  if (targetLoc2 != prevTargetLoc2){
  Serial.printf("%i\n", position);
   if (targetLoc2 == TRUE){
      ledOnOff2= !ledOnOff2;
      } prevTargetLoc2 = targetLoc2;
      digitalWrite(LEDPIN2, ledOnOff2);
      Serial.printf("Next Song\n");
      (ledOnOff2 = TRUE); 
       myDFPlayer.next();
      if (ledOnOff2 = FALSE ){
        myDFPlayer.stop();
        }
      Serial.printf("LED2 On%i\n",targetLoc2);
      startTime = millis();
    }

if ((millis()-startTime) > sampleTime) // TOF Level 3 Range 135-185mm
  if (targetLoc3 != prevTargetLoc3){
  Serial.printf("%i\n", position);
   if (targetLoc3 == TRUE){
      ledOnOff3= !ledOnOff3;
      } prevTargetLoc3= targetLoc3;
      digitalWrite(LEDPIN3, ledOnOff3);
      Serial.printf("LED3 On%i\n", targetLoc3);
       startTime = millis();
    }
if ((millis()-startTime) > sampleTime) // TOF Level 4 Range Volume Up/Down 200-280mm
  if (targetLoc4 != prevTargetLoc4){
  Serial.printf("%i\n", position);
   if (targetLoc4 == TRUE){
      ledOnOff4= !ledOnOff4;
      } prevTargetLoc4= targetLoc4;
      //digitalWrite(LEDPIN3, ledOnOff4); 
       myDFPlayer.volumeUp();
      if (ledOnOff4 = FALSE ){
        myDFPlayer.volumeDown();
        Serial.printf("Vol DN%i\n", ledOnOff4);
        }
      Serial.printf("MP3 Vol%i\n", targetLoc4);
       startTime = millis();
    }
  if (measure.RangeStatus != 5) {  // phase failures have incorrect data
    if(measure.RangeMilliMeter <= 50){ //70
      targetLoc1 = TRUE;
      targetLoc2 = FALSE;
      targetLoc3 = FALSE; 
      targetLoc0 = FALSE;
      targetLoc4 = FALSE;
     Serial.printf("targetPos 1%i\n", targetLoc1);
    }
   else if(measure.RangeMilliMeter > 60 && measure.RangeMilliMeter < 120){
      targetLoc2 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc3 = FALSE; 
      targetLoc0 = FALSE;
      targetLoc4 = FALSE;
     Serial.printf("targetPos 2%i\n", targetLoc2);
    }
   else if(measure.RangeMilliMeter > 130 && measure.RangeMilliMeter < 190){
      targetLoc3 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc2 = FALSE; 
      targetLoc0 = FALSE;
      targetLoc4 = FALSE;
     Serial.printf("targetPos 3%i\n", targetLoc3);
    }
    else if(measure.RangeMilliMeter > 200 && measure.RangeMilliMeter < 300){
      targetLoc4 = TRUE;
      targetLoc3 = FALSE; //
      targetLoc1 = FALSE; 
      targetLoc2 = FALSE; 
      targetLoc0 = FALSE;
    Serial.printf("targetPos 4%i\n", targetLoc4);
  }
  } else {
    targetLoc0 = TRUE;
    targetLoc1 = FALSE; 
    targetLoc2 = FALSE; 
    targetLoc3 = FALSE; 
    targetLoc4 = FALSE; 
    Serial.printf("targetPos 0%i\n", targetLoc0);
  }
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
void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }

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