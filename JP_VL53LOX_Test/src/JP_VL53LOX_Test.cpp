/* 
 * Project JP VL53LOX test
 * Author: JP Funk
 * Date: 05/06/2024 MOnday- still working on TOF bugs
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_VL53L0X.h"
#include <SPI.h>
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
unsigned int rangeTime = 250;

const int BUTTONPIN = D3;
void buttonisClicked();
bool changed, buttonState;

// Touch Sensor
Button PUMPBUTTON (D3);
// LED Pins
const int LEDPIN = D6; // LED pin  for TOF Location 1 Water Pump

// Neopixel
const int PIXELCOUNT = 5; // Total number of single NeoPixels
int i, pixelAddr, colorCount;
bool neoOnOff;
void pixelFill(int start, int end, int color);
void targetRange();
//Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B); // declare object
Adafruit_NeoPixel pixel(PIXELCOUNT, D2, WS2812B); // declare object
// Water Pump
const int PUMPIN = D7;
float  pubValue;
int subValue;
// Millis Timer
const int buttonTime = 250; //3000
const int sampleTime = 250; //3000
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

const int PIXEL_COUNT = 12;
//#define PIXEL_TYPE WS2812B
//Adafruit_NeoPixel strip(PIXEL_COUNT, SPI, WS2812B); for Photon 2
Adafruit_NeoPixel strip(PIXEL_COUNT, D12, WS2812B);
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
for (colorCount = 0; colorCount <= 6; colorCount++) {
  // The core of your code will likely live here.
 pixel.setPixelColor(pixelAddr, rainbow[colorCount]);
 pixelAddr++;
 pixel.show (); // nothing changes until show (){
 delay(200);
 }
//DFRobotMP3Player
  if (!myDFPlayer.begin(Serial1)) {  //Use softwareSerial to communicate with mp3.
    Serial.printf("Unable to begin:\n");
    Serial.printf("1.Please recheck the connection!\n");
    Serial.printf("2.Please insert the SD card!\n");
    while(true);
  }
  Serial.printf("DFPlayer Mini online.\n");
  myDFPlayer.volume(15);  //Set volume value. From 0 to 30
  myDFPlayer.loop(1);  //Play the first mp3
// Millis Startime
startTime = millis();
//startTime = 0;
}

void loop() {
 // Pump Button
  if(PUMPBUTTON.isClicked()) {
    buttonState =!buttonState;
   }
   //if ((millis()-startTime) > buttonTime){
    if (buttonState) {
      digitalWrite (PUMPIN, HIGH);
      pixelFill(4,4, teal);
     // Serial.printf("Pump Button On\n");
      } 
      else {
      digitalWrite (PUMPIN, LOW);
      pixelFill(4,4, black);
     // Serial.printf("Pump OFF \n");
      }
   // startTime = millis();
   //} Button Millis Bracket

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
    else if(measure.RangeMilliMeter > 100 && measure.RangeMilliMeter < 180){ // else
      targetLoc2 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc3 = FALSE; 
      targetLoc0 = FALSE;
    // Serial.printf("targetPos 2%i\n", targetLoc2);
    }
    else if(measure.RangeMilliMeter > 200 && measure.RangeMilliMeter < 300){ // else
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
     // Serial.printf("targetPos 0%i\n", targetLoc0);
  }
}

  // TOF Level 1 Neopixel Range 0-100mm
  // if (targetLoc1 != prevTargetLoc1){
  // //((millis()-startTime) > rangeTime);
  //   if (targetLoc1 == TRUE){
  //     neoOnOff= !neoOnOff;
  //      if (neoOnOff) {
  //       rainbowRing(20);
  //        strip.show();
  //       Serial.printf("Neo Ring On%i\n",targetLoc1);
  //       } else {
  //       Serial.printf("Neo Ring Off%i\n",targetLoc1);
  //       strip.clear();
  //      // strip.show();
  //       }
  //     }
  //   // startTime = millis();
  //     prevTargetLoc1 = targetLoc1;
  //   }
      
      // TOF Level 1 Neopixel Range 0-100mm
  if (targetLoc1 ){ /// new code section Monday afgernoon woking with EJ
  //((millis()-startTime) > rangeTime);
    //if (targetLoc1 =!targetLoc1){
        neoOnOff= !neoOnOff;
        targetLoc1 = FALSE;
      }   
        if (neoOnOff) {
        rainbowRing(20);
        Serial.printf("Neo Ring On%i\n",targetLoc1);
        } else {
        Serial.printf("Neo Ring Off%i\n",targetLoc1);
        strip.clear();
        strip.show();
        }
  
  

 // TOF Level 2 MP3 Next Range 100-200mm
  if (targetLoc2 != prevTargetLoc2) {
   // ((millis()-startTime) > rangeTime);
    if (targetLoc2 == TRUE){
     // Serial.printf("%i,%i\n", targetLoc2, prevTargetLoc2);
      startStop = !startStop;
      if (startStop){
        myDFPlayer.play(track%3+1);
        Serial.printf("MP3 On%i\n",targetLoc2);
         // myDFPlayer.loop(1); 
          } else {
          myDFPlayer.stop();
          Serial.printf("MP3 Off%i\n",targetLoc2);
          track++;
          }
        }
     //  startTime = millis(); 
     prevTargetLoc2 = targetLoc2;
      }
      

   // TOF Level 3 MP3 Volume Range 200-300mm
  if (targetLoc3 != prevTargetLoc3) {
   // ((millis()-startTime) > rangeTime);
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
    // startTime = millis();
    prevTargetLoc3 = targetLoc3;
    }
    

} // End Void Loop

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
  if(targetLoc1){   // Neopixel TOF location 1  NeoPixel Ring OnOFF
    if (neoOnOff){
    pixelFill(0,3,green);
    }
    else {
    pixelFill(0,3,red);
    }
    pixel.show();
  }

  if(targetLoc2){   // Neopixel TOF location 2  MP3 Tracks OnOFF
    if (startStop){
    pixelFill(0,3,turquoise);
    }
    else {
    myDFPlayer.stop();
    }
    pixel.show();
  }

  if(targetLoc3){  // Neopixel TOF location 3  MP3 Volume OnOFF
    if (volOnOff){
    pixelFill(0,3,blue);
    }
    else {
    pixelFill(0,3,yellow);
    }
    pixel.show();
  }
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