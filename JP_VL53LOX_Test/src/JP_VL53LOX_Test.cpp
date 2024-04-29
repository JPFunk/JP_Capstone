/* 
 * Project JP VL53LOX test
 * Author: JP Funk
 * Date: 04/26/2024 Friday Morning of TOF MP3 Player + Volume , Neopixels, water pump, NeoPixelRing
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


// DFRobotMP3 Player
DFRobotDFPlayerMini myDFPlayer;
Button nextButton(D0);
unsigned int lastSong;
bool startStop;
bool volOnOff;
int track;
const int volumeTime = 300; //3000

// TOF VL53LOX Sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int TOF;
bool targetLoc0, targetLoc1, targetLoc2, targetLoc3;
bool prevTargetLoc1, prevTargetLoc2, prevTargetLoc3;
bool position;
unsigned int rangeTime = 500;

//const int BUTTONPIN = D3;
void buttonisClicked();
bool changed, buttonState;

// Touch Sensor
Button PUMPBUTTON (D3);
const int buttonTime = 500; //3000
// LED Pins
const int LEDPIN = D6; // LED pin  for TOF Location 1 Water Pump
// const int LEDPIN2 = D5; // LED pin for TOF Location 2
// const int LEDPIN3 = D4; // LED pin for TOF Location 3
// bool ledOnOff, ledOnOff2, ledOnOff3;

// Neopixel
const int PIXELCOUNT = 4; // Total number of single NeoPixels
int i, pixelAddr ;
bool neoOnOff;
void pixelFill(int start, int end, int color);
void targetRange();
Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B); // declare object

// Water Pump
const int PUMPIN = D7;
float  pubValue;
int subValue;
// Millis Timer
const int sampleTime = 500; //3000
const int pixelTime = 20; //3000
unsigned int duration, startTime;
// Adafruit BME
Adafruit_BME280 bme;
const char degree = 0xf8;
const int delayTime =1000;
bool status;
int hexAddress, startime;
float tempC, pressPA, humidRH, tempF, inHG;

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

// NeoPixel Ring
// #if (PLATFORM_ID == 32)
// #define PIXEL_PIN SPI
// #else // #if (PLATFORM_ID == 32)
// #define PIXEL_PIN D15
// #endif
const int PIXEL_COUNT = 12;
//#define PIXEL_TYPE WS2812B

Adafruit_NeoPixel strip(PIXEL_COUNT, SPI, WS2812B);
void rainbowRing(uint8_t wait);
uint32_t Wheel(byte WheelPos);
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
//Pump Pin
pinMode (PUMPIN, OUTPUT);
// LED
pinMode (LEDPIN, OUTPUT);
// pinMode (LEDPIN2, OUTPUT);
// pinMode (LEDPIN3, OUTPUT);

 // NeoPixel Set Up----------------------------------------------------------------
pixel.begin ();
pixel.setBrightness (64); // bri is a value 0 - 255
pixel.show (); // initialize all off
// Neopixel Ring
strip.begin();
strip.show(); // Initialize all pixels to 'off'
//DFRobotMP3Player
  if (!myDFPlayer.begin(Serial1)) {  //Use softwareSerial to communicate with mp3.
    Serial.printf("Unable to begin:\n");
    Serial.printf("1.Please recheck the connection!\n");
    Serial.printf("2.Please insert the SD card!\n");
    while(true);
  }
  Serial.printf("DFPlayer Mini online.\n");
  myDFPlayer.volume(15);  //Set volume value. From 0 to 30
  myDFPlayer.loop(0);  //Play the first mp3
// Millis Startime
startTime = millis();
//startTime = 0;
}

void loop() {
 // Pump Button
if ((millis()-startTime) > buttonTime){ 
  if(PUMPBUTTON.isPressed()) {
    buttonState =!buttonState;
    if (buttonState) {
      digitalWrite (PUMPIN, HIGH);
      Serial.printf("Pump Button On\n");
      } 
      else {
      digitalWrite (PUMPIN, LOW);
      Serial.printf("Pump OFF \n");
      }
      startTime = millis();
  }
}
// TOF Ranging functions
VL53L0X_RangingMeasurementData_t measure;
lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
targetRange(); // Neopixel VOID function int for target ranges
if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    if(measure.RangeMilliMeter <= 80){ //70
      targetLoc1 = TRUE;
      targetLoc2 = FALSE;
      targetLoc3 = FALSE; 
      targetLoc0 = FALSE;
    // Serial.printf("targetPos 1%i\n", targetLoc1);
    }
    else if(measure.RangeMilliMeter > 100 && measure.RangeMilliMeter < 180){
      targetLoc2 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc3 = FALSE; 
      targetLoc0 = FALSE;
    // Serial.printf("targetPos 2%i\n", targetLoc2);
    }
    else if(measure.RangeMilliMeter > 200 && measure.RangeMilliMeter < 300){
      targetLoc3 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc2 = FALSE; 
      targetLoc0 = FALSE;
     //Serial.printf("targetPos 3%i\n", targetLoc3);
    }
      else {
      targetLoc0 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc2 = FALSE; 
      targetLoc3 = FALSE; 
      Serial.printf("targetPos 0%i\n", targetLoc0);
  }
}

// if ((millis()-startTime) > sampleTime){ // TOF Level 1 Neopixel Range 0-100mm
//   if (targetLoc1 != prevTargetLoc1){
//   Serial.printf("%i\n", position);
//     if (targetLoc1 == TRUE){
//       neoOnOff= !neoOnOff;
//       } prevTargetLoc1 = targetLoc1;
//         digitalWrite(LEDPIN, neoOnOff);
//         Serial.printf("Red LED On\n",targetLoc1);
//         startTime = millis();
//     }
//   }

if ((millis()-startTime) > sampleTime){ // TOF Level 1 Neopixel Range 0-100mm
  if (targetLoc1 != prevTargetLoc1){
    if (targetLoc1 == TRUE){
      neoOnOff= !neoOnOff;
       if (neoOnOff) {
        rainbowRing(20);
        Serial.printf("Neo Ring On%i\n",targetLoc1);
        } else {
        //rainbowRing(20);
        Serial.printf("Neo Ring Off%i\n",targetLoc1);
        strip.clear();
        strip.show();
        }
      }
       startTime = millis();
    }
    prevTargetLoc1 = targetLoc1;
}

if ((millis()-startTime) > sampleTime){  // TOF Level 2 MP3 Next Range 100-200mm
  if (targetLoc2 != prevTargetLoc2) {
    if (targetLoc2 == TRUE){
      Serial.printf("%i,%i\n", targetLoc2, prevTargetLoc2);
      startStop = !startStop;
      if (startStop){
        myDFPlayer.play(track%3+1);
      // myDFPlayer.loop(0);  
        Serial.printf("MP3 On%i\n",targetLoc2);
          } else {
          myDFPlayer.stop();
          Serial.printf("MP3 Off%i\n",targetLoc2);
          track++;
          }
        }
        startTime = millis();
      }
      prevTargetLoc2 = targetLoc2;
}
if ((millis()-startTime) > sampleTime){  // TOF Level 3 MP3 Volume Range 200-300mm
  if (targetLoc3 != prevTargetLoc3) {
    if (targetLoc3 == TRUE){
      Serial.printf("%i,%i\n", targetLoc3, prevTargetLoc3);
      volOnOff = !volOnOff;
      if (volOnOff){
        myDFPlayer.volume(30);
        Serial.printf("Volume On%i\n",targetLoc3);
        } else {
          myDFPlayer.volume(0);
          Serial.printf("Volume Off%i\n",targetLoc3);
        }  
      }
      startTime = millis(); 
    }
    prevTargetLoc3 = targetLoc3;
}
}
// VOID Functions Neopixel
void pixelFill(int start, int end, int color) {
 int i;
 for (i=start; i<=end; i++){
 pixel.setPixelColor (i, color); // hexadecimal color
 }
  pixel.show (); // nothing changes until show ()
}
 // Neopixel activation with TOF Ranges
void targetRange() {
if ((millis()-startTime) > sampleTime) // new code for pixel targetloc
if(targetLoc1){   // Neopixel TOF location 1  NeoPixel Ring OnOFF
  if (startStop){
  pixelFill (0,3,red);
  }
  else {
  pixelFill (0,3,green);
  }
  startTime = millis();  // new code for pixel targetloc
  pixel.show();
}

if(targetLoc2){   // Neopixel TOF location 2  MP3 Tracks OnOFF
  if (startStop){
  pixelFill (0,3,turquoise);
  }
  else {
  pixelFill (0,3,magenta);
  }
  startTime = millis();  // new code for pixel targetloc
  pixel.show();
}

if(targetLoc3){  // Neopixel TOF location 3  MP3 Volume OnOFF
  if (startStop){
  pixelFill (0,3,blue);
  }
  else {
  pixelFill (0,3,yellow);
  }
  startTime = millis();  // new code for pixel targetloc
  pixel.show();
}
pixel.clear();
}

// Rainbow Ring Neopixel Function for Crystal Ball
void rainbowRing(uint8_t pixelTime) {
  if ((millis()-startTime) > pixelTime){
  uint16_t k, j;
  for(j=0; j<256; j++) {
    for(k=0; k<strip.numPixels(); k++) {
      strip.setPixelColor(k, Wheel((k+j) & 255));
      }
      strip.show();
      startTime = millis();
    }
  }
}
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}