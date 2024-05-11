/* 
 * Project JP_Oracle_Fountain
 * Author: JP Funk
 * Date: 05/10/2024 Friday update with VL53LOX_Test code functions
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_VL53L0X.h"
#include <SPI.h>
#include "credentials.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_BME280.h"
#include "DFRobotDFPlayerMini.h"
#include "IoTClassroom_CNM.h"
#include "IoTTimer.h"
#include "math.h"
#include "Button.h"
#include "neopixel.h"
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
bool targetLoc0, targetLoc1, targetLoc2, targetLoc3, targetLoc4;
bool prevTargetLoc1, prevTargetLoc2, prevTargetLoc3;
bool position;
unsigned int rangeTime = 250;
void targetButton1();
void targetButton2();
void targetButton3();

// Reset Button
int8_t RST;
const int RESETBUTTON (RST);
int resetBtn;
int lastInterval;
bool buttonState1;
void buttonisClicked();
bool changed, buttonState;

// Touch Sensor
Button PUMPBUTTON (D3);
void WATERPUMP ();
// const int buttonTime = 500; //3000 old needs to be removed once other code is working

// Neopixel
const int PIXELCOUNT = 5; // Total number of single NeoPixels
int i, pixelAddr, colorCount, hold;
bool neoOnOff, blackOnOff,  randomOnOff;
void pixelFill(int start, int end, int color);
void targetRange();
//Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B); // declare object
Adafruit_NeoPixel pixel(PIXELCOUNT, D2, WS2812B); // declare object

// Water Pump
const int PUMPIN = D7;
float  pubValue;
bool subValue;

// Millis Timer
const int buttonTime = 250; //3000
const int sampleTime = 250; //3000
const int pixelTime = 20; //3000
unsigned int duration, beginTime;

// Date and Time String
String DateTime, TimeOnly; // String variable for Date and Time
// Adafruit BME
Adafruit_BME280 bme;
const char degree = 0xf8;
const int delayTime =1000;
bool status;
int hexAddress, startime;
float tempC, pressPA, humidRH, tempF, inHG;

void watchdogHandler() {
// Do as little as possible in this function , preferably just a reset
System.reset (RESET_NO_WAIT);
}
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_Publish tempFeed = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME"/feeds/tempFeed");
Adafruit_MQTT_Publish pressFeed = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME"/feeds/pressFeed");
Adafruit_MQTT_Publish humidFeed = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME"/feeds/humidFeed");
Adafruit_MQTT_Subscribe pumpFeed = Adafruit_MQTT_Subscribe(&mqtt,AIO_USERNAME"/feeds/pumpFeed");
Adafruit_MQTT_Subscribe buttonFeed = Adafruit_MQTT_Subscribe(&mqtt,AIO_USERNAME"/feeds/buttononoff");  
/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();
// Let Device OS manage the connection to the Particle Cloud
void getConc ();
 
SYSTEM_MODE(SEMI_AUTOMATIC);

#if (PLATFORM_ID == 32)
//const int PIXEL_COUNT = 12;
#define PIXEL_PIN D12
#endif
#define PIXEL_COUNT 12
#define PIXEL_TYPE WS2812B
//#define PIXEL_TYPE WS2812B
//Adafruit_NeoPixel strip(PIXEL_COUNT, SPI, WS2812B); for Photon 2
Adafruit_NeoPixel strip(PIXEL_COUNT, D12, WS2812B);
void rainbowRing(uint8_t hold);
uint32_t Wheel(byte WheelPos);
IoTTimer neoTimer; // New Neo COde
IoTTimer locTimer; // New Neo COde
bool repeatCycle; // New Neo COde
SYSTEM_THREAD(ENABLED);

// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);
// setup() runs once, when the device is first turned on

void setup() {
  waitFor(Serial.isConnected, 5000);
  Serial.begin(115200);
  Serial1.begin(9600);
  delay(1000);

  Wire.begin(); // ToF start sequence
// New Neo Code--------------------------------------------------
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  repeatCycle = TRUE;
  neoTimer.startTimer(20);
//--------------------------------------------------- TOF New Code

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
 
 // Connect to Internet but not Particle Cloud
WiFi.on();
WiFi.connect();
  while(WiFi.connecting()) {
  Serial.printf(".");
  }
Serial.printf("\n\n");
// Setup MQTT subscription
mqtt.subscribe(&buttonFeed);
mqtt.subscribe(&pumpFeed);
 // Particle Time
Particle.connect;  
Time.zone (-7); // MST = -7, MDT = -6
Particle.syncTime (); // Sync time with Particle Cloud

// Button
pinMode(RESETBUTTON, INPUT);
//Pump Pin
pinMode (PUMPIN, OUTPUT);

 // NeoPixel Set Up----------------------------------------------------------------
pixel.begin ();
pixel.setBrightness (64); // bri is a value 0 - 255
pixel.show (); // initialize all off

// Neopixel Ring-----------------------------------------
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
beginTime = millis();

lastInterval = millis();
// initialize BME
status = bme.begin(0x76);
  if (status==false);{
  Serial.printf("BME280 at address 0x%02X failed to start\n", 0x76); // Adafruit_BME280 bme code
  }
startime = millis();
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  WATERPUMP();
 //targetButton1();
  targetButton2();
  targetButton3();
 locTimer.startTimer(10000);
 // TOF Ranging functions
VL53L0X_RangingMeasurementData_t measure;
lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
targetRange(); // Neopixel VOID function int for target ranges
//Serial.printf("measure data %i\n",measure.RangeMilliMeter);
if (measure.RangeStatus != 5) {  // phase failures have incorrect data
    if(measure.RangeMilliMeter <= 70){
      targetLoc1 = TRUE;
      targetLoc2 = FALSE;
      targetLoc3 = FALSE;
      targetLoc4 = FALSE;
      targetLoc0 = FALSE;
    // Serial.printf("targetPos 1%i\n", targetLoc1);
    }
    else if(measure.RangeMilliMeter > 70 && measure.RangeMilliMeter < 150){
      targetLoc2 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc3 = FALSE;
      targetLoc4 = FALSE;
      targetLoc0 = FALSE;
    // Serial.printf("targetPos 2%i\n", targetLoc2);
    }
    else if(measure.RangeMilliMeter > 150 && measure.RangeMilliMeter < 230){
      targetLoc3 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc2 = FALSE;
      targetLoc4 = FALSE;
      targetLoc0 = FALSE;
     //Serial.printf("targetPos 3%i\n", targetLoc3);
    }
      else if (measure.RangeMilliMeter > 230 && measure.RangeMilliMeter < 400){
      targetLoc4 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc2 = FALSE; 
      targetLoc3 = FALSE;
      targetLoc0 = FALSE;
     //Serial.printf("targetPos 0%i\n", targetLoc0);
  }  
      else if  (measure.RangeMilliMeter > 7000){//Reset
      locTimer.startTimer(10000);
      targetLoc0 = TRUE;
      targetLoc1 = FALSE; 
      targetLoc2 = FALSE; 
      targetLoc3 = FALSE;
      targetLoc4 = FALSE;
     //Serial.printf("targetPos 0%i\n", targetLoc0);
 }
} 

MQTT_connect();
MQTT_ping();
  DateTime =Time.timeStr(); // Current Date and Time from Particle Time class
  TimeOnly =DateTime.substring (11,19); // Extract the Time from the DateTime String

  // BME functions with OLED display
  tempC = bme.readTemperature();
  tempF = map (tempC,0.0,100.0,32.0,212.0);
  pressPA = bme.readPressure();
  inHG = pressPA / 3386.0;
  humidRH = bme.readHumidity();
  // Adafruit Dashboard Button for Reset
Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &buttonFeed) {
      resetBtn = atoi((char *)buttonFeed.lastread);
      Serial.printf("buttonFeed%i\n", resetBtn);
      }
    if (subscription == &pumpFeed) {
      subValue = atoi((char *)pumpFeed.lastread);
      Serial.printf("pumpFeed%i\n", subValue);
      }
    }

  // Adafruit MQTT Publish functions----------------------------------------------
  Adafruit_MQTT_Publish *publish;
  static unsigned int last;
  if ((millis()-last)>10000 ) 
  if(mqtt.Update()) {
    tempFeed.publish(tempF);
    pressFeed.publish(inHG);
    humidFeed.publish(humidRH);
    last = millis();
    }
    if (resetBtn) {
    System.reset(RESET_NO_WAIT);
  }

} //End Void Loop Functions

void WATERPUMP (){
   // Pump Button
  if(PUMPBUTTON.isClicked()) {
    buttonState =!buttonState;
    //subValue = 0;
  }
  if (buttonState || subValue){
    digitalWrite (PUMPIN, HIGH);
    pixelFill(4,4, teal);
    subValue = 0;
    Serial.printf("Pump Button On\n");
    } 
    else {
    digitalWrite (PUMPIN, LOW);
    pixelFill(4,4, black);
    Serial.printf("Pump OFF \n");
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

// TOF Level 2 MP3 Next old Range 100-200mm
void targetButton2(){
  if (targetLoc2 != prevTargetLoc2) {
     Serial.printf("targetLoc 2 changed%i\n",targetLoc2);
    if (targetLoc2 == TRUE){
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
      prevTargetLoc2 = targetLoc2; // previous location
   }
}

// TOF Level 3 MP3 Volume old Range 200-300mm
void targetButton3(){
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
        pixel.show();
      }
    prevTargetLoc3 = targetLoc3;
    }
}

 // Neopixel activation with TOF Ranges
void targetRange() {
  if(targetLoc1){   // Neopixel TOF location 1  NeoPixel Ring OnOFF
    if (neoOnOff){
      repeatCycle = TRUE;
      pixelFill(0,3,teal);
      rainbowRing(20);
    }
    else {
      pixelFill(0,3,orange);
      strip.clear();
      strip.show();
    }
  }

  if(targetLoc2){   // Neopixel TOF location 2  MP3 Tracks OnOFF
    if (startStop){
    pixelFill(0,3,turquoise);
    }
    else {
    pixelFill(0,3,magenta);
    // myDFPlayer.stop(); don't know how this got here?
    }
    pixel.show();
  }

  if(targetLoc3){  // Neopixel TOF location 3  MP3 Volume OnOFF
    if (volOnOff){
    pixelFill(0,3,blue);
    }
    else {
    pixelFill(0,3,lime);
    }
    pixel.show();
  }

  if(targetLoc4){  // Neopixel TOF location 0 Turn Off Black
    blackOnOff =!blackOnOff;
    if (blackOnOff){
    pixelFill(0,3, black);
    }
  }

  if(targetLoc0){  // Neopixel TOF reset
  rainbowRing(20);
  if (neoOnOff == FALSE){
    strip.show();
    strip.clear();
  }
   if(locTimer.isTimerReady()){
   // blackOnOff =!blackOnOff;
    //if (blackOnOff){
    pixelFill(0,3, black);
    }
  }
}

void rainbowRing(uint8_t hold) {
  uint16_t k;
  static uint16_t j;
    if(neoTimer.isTimerReady()){
      for(k=0; k<strip.numPixels(); k++) {
        strip.setPixelColor(k, Wheel((k+j) & 255));
      }
      strip.show();
      j++;
      neoTimer.startTimer(hold);
    }
    if(j == 256){
      j = 0;
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

// MQTT Adafruit.IO connections
void MQTT_connect() {
  int8_t ret;
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
    Serial.printf("Retrying MQTT connection in 5 seconds...\n");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}
bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;
  if ((millis()-last)>120000) {
    Serial.printf("Pinging MQTT \n");
    pingStatus = mqtt.ping();
    if(!pingStatus) {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
