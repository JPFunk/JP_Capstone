/* 
 * Project JP VL53LOX test
 * Author: JP Funk
 * Date: 04/19/2024 Friday Build of TOF Functionality w/ 3 LEDs MP3 Player Volume integration, Started Neopixels
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
#include "Neopixel.h"
#include "Colors.h"
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
int targetLoc0, targetLoc1, targetLoc2, targetLoc3, targetLoc4, targetLoc5; 
int prevTargetLoc1, prevTargetLoc2, prevTargetLoc3, prevTargetLoc4, prevTargetLoc5;
bool position;
unsigned int rangeTime = 1000;
int modeSeq, modeVol, preVol;
const int volumeTime = 100; //3000

//const int BUTTONPIN = D3;
void buttonisClicked();
bool changed;
bool ledOnOff, ledOnOff2, ledOnOff3, ledOnOff4, ledOnOff5;
//void ledOn (int waterPumpPin);
const int LEDPIN = D6; // LED pin  for TOF Location 1 Water Pump
const int PUMPIN = D6;
const int LEDPIN2 = D5; // LED pin for TOF Location 2
const int LEDPIN3 = D4; // LED pin for TOF Location 3

// Neopixel
const int PIXELCOUNT = 12; // Total number of NeoPixels
// const int maxRange = 82;
// const int minRange = 72;
Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B); // declare object
int pixelAddr ;
int range1, range2, range3, range4, range5;
void pixelFill(int start, int end, int color);
int colorCount, color;
int i;
void targetRange();
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
  
  Wire.begin(); // ToF start sequence

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
//Neopixel display
 pixel.begin ();
 pixel.setBrightness (64); // bri is a value 0 - 255
 pixel.show (); // initialize all off

 // NeoPixel Set Up----------------------------------------------------------------
pixel.begin ();
pixel.setBrightness (64); // bri is a value 0 - 255
pixel.clear ();
pixel.show (); // initialize all off
colorCount=0;
pixelAddr=0;
random (0,4); // LED assignment 0,1,2,3,4
rainbow[0,1,2,3,4];

//DFRobotMP3Player
  if (!myDFPlayer.begin(Serial1)) {  //Use softwareSerial to communicate with mp3.
    Serial.printf("Unable to begin:\n");
    Serial.printf("1.Please recheck the connection!\n");
    Serial.printf("2.Please insert the SD card!\n");
    while(true);
  }
  Serial.printf("DFPlayer Mini online.\n");
  
  myDFPlayer.volume(30);  //Set volume value. From 0 to 30
  myDFPlayer.loop(0);  //Play the first mp3
  myDFPlayer.enableLoopAll();
  myDFPlayer.volume(modeVol);
// Millis Startime
startTime = millis();
//startTime = 0;
}
// loop() runs over and over again, as quickly as it can execute.
void loop() {

VL53L0X_RangingMeasurementData_t measure;
lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
targetRange(); // Neopixel VOID function int for target ranges

if ((millis()-startTime) > sampleTime) // TOF Level 1 Water Pump Range 0-50mm
  if (targetLoc1 != prevTargetLoc1){
  Serial.printf("%i\n", position);
  startTime = millis();
   if (targetLoc1 == TRUE) {
    ledOnOff= !ledOnOff;
   }  prevTargetLoc1 = targetLoc1;
      digitalWrite(LEDPIN, ledOnOff);
      Serial.printf("LED1%i\n", targetLoc1);
      startTime = millis();
    }

if ((millis()-startTime) > sampleTime) // TOF Level 2 Neopixel Range 5-115mm
  if (targetLoc2 != prevTargetLoc2){
  Serial.printf("%i\n", position);
   if (targetLoc2 == TRUE){
      ledOnOff2= !ledOnOff2;
      } prevTargetLoc2 = targetLoc2;
        digitalWrite(LEDPIN2, ledOnOff2);
        Serial.printf("LED2 On%i\n",targetLoc2);
        startTime = millis();
    }

if ((millis()-startTime) > sampleTime) // TOF Level 3 MP3 Next Range 135-185mm
  if (targetLoc3 != prevTargetLoc3){
  Serial.printf("%i\n", position);
   if (targetLoc3 == TRUE){
      ledOnOff3= !ledOnOff3;
      } prevTargetLoc3= targetLoc3;
      digitalWrite(LEDPIN3, ledOnOff3);
      myDFPlayer.next();
      Serial.printf("Next Song\n");
      Serial.printf("LED3 On%i\n", targetLoc3);
      startTime = millis();
  } 

    if ((millis()-startTime) > sampleTime)  // TOF Level 3 MP3 Stop Range 135-185mm
      if (targetLoc3 != prevTargetLoc3){
       if (targetLoc3 == TRUE){
         ledOnOff3= !ledOnOff3;
        }  prevTargetLoc3= targetLoc3;
          digitalWrite(LEDPIN3, ledOnOff3);
          myDFPlayer.stop();
          Serial.printf("Stop\n");
          Serial.printf("LED3 On%i\n", targetLoc3);
          startTime = millis();
      }
    
    //TOF Volume Code
    if ((millis()-startTime) > volumeTime)  // TOF Level 4 Volume Up Range 200-240mm
      if (targetLoc4 != prevTargetLoc4) {
        myDFPlayer.volumeUp();
        myDFPlayer.readVolume();
        Serial.printf("MP3 Vol DN%i\n", targetLoc4);
        startTime = millis();
      }

    if ((millis()-startTime) > volumeTime)  // TOF Level 5 Volume Dn Range 260-300mm
      if (targetLoc5 != prevTargetLoc5) {
        myDFPlayer.volumeDown();
        myDFPlayer.readVolume();
        Serial.printf("MP3 Vol DN%i\n", targetLoc5);
        startTime = millis();
      }

if (measure.RangeStatus != 6) {  // phase failures have incorrect data
    if(measure.RangeMilliMeter <= 30){ //70
      targetLoc1 = TRUE;
      targetLoc2 = FALSE;
      targetLoc3 = FALSE; 
      targetLoc4 = FALSE;
      targetLoc5 = FALSE;
      targetLoc0 = FALSE;
     Serial.printf("targetPos 1%i\n", targetLoc1);
    }
   else if(measure.RangeMilliMeter > 50 && measure.RangeMilliMeter < 110){
      targetLoc2 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc3 = FALSE; 
      targetLoc4 = FALSE;
      targetLoc5 = FALSE;
      targetLoc0 = FALSE;
     Serial.printf("targetPos 2%i\n", targetLoc2);
    }
   else if(measure.RangeMilliMeter > 125 && measure.RangeMilliMeter < 180){
      targetLoc3 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc2 = FALSE; 
      targetLoc4 = FALSE;
      targetLoc5 = FALSE;
      targetLoc0 = FALSE;
     Serial.printf("targetPos 3%i\n", targetLoc3);
    }
    else if(measure.RangeMilliMeter > 200 && measure.RangeMilliMeter < 240){
      targetLoc4 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc2 = FALSE; 
      targetLoc3 = FALSE;
      targetLoc5 = FALSE;
      targetLoc0 = FALSE;
    Serial.printf("targetPos 4%i\n", targetLoc4);
  }
    else if(measure.RangeMilliMeter > 260 && measure.RangeMilliMeter < 300){
      targetLoc5 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc2 = FALSE;
      targetLoc3 = FALSE; 
      targetLoc4 = FALSE; 
      targetLoc0 = FALSE;
    Serial.printf("targetPos 5%i\n", targetLoc5);
  }
  } else {
    targetLoc0 = TRUE;
    targetLoc1 = FALSE; 
    targetLoc2 = FALSE; 
    targetLoc3 = FALSE; 
    targetLoc4 = FALSE;
    targetLoc5 = FALSE;
    Serial.printf("targetPos 0%i\n", targetLoc0);
  }
}

// VOID Functions 
void pixelFill(int start, int end, int color) {
 int i;
 for (i=start; i<=end; i++){
 pixel.setPixelColor (i, color); // hexadecimal color
 }
  pixel.show (); // nothing changes until show ()
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

void targetRange() { // Neopixel activation with TOF Ranges
   if(range1 = targetLoc1){ // Neopixel TOF location 1 Pump
    pixelFill (0,11,red);
      pixel.show();
  }
  if(range2 = targetLoc2){  // Neopixel TOF location 1 Neopixel Crystal Ball selection
    pixelFill (0,11,yellow);
      pixel.show();
  }
   if(range3 = targetLoc3){  // Neopixel TOF location 3  MP3 Tracks
    pixelFill (0,11,blue);
      pixel.show();
  }
  if(range4 = targetLoc4){  // Neopixel TOF location 4 MP3 Volume Up
    pixelFill (0,11,cyan);
      pixel.show();
  }
   if(range5 = targetLoc5){  // Neopixel TOF location 5 MP3 Volume Downn
    pixelFill (0,11,violet);
      pixel.show();
  }
  pixel.clear();
}