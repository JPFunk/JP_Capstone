/* 
 * Project JP_Oracle_Fountain
 * Author: JP Funk
 * Date: 04/25/2024 Thursday update with Dashboard, reset button & water pump
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
#include "IoTClassroom_CNM.h"
#include "IoTTimer.h"
#include "math.h"
#include "Button.h"

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
const int buttonTime = 500; //3000

// Water Pump
const int PUMPIN = D7;
float  pubValue;
int subValue;
// Millis Timer
const int sampleTime = 500; //3000
unsigned int duration, startTime;
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
SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);
// setup() runs once, when the device is first turned on

void setup() {
Serial.begin(9600);
waitFor(Serial.isConnected,10000);
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
MQTT_connect();
MQTT_ping();
  DateTime =Time.timeStr(); // Current Date and Time from Particle Time class
  TimeOnly =DateTime.substring (11,19); // Extract the Time from the DateTime String
  Serial.printf("Date and time is %s\n",DateTime.c_str());
  Serial.printf("Time is %s\n",TimeOnly.c_str());
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
  
  // Adafruit MQTT Publish functions
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
  // Water Pump Button OnOff
  if ((millis()-startTime) > buttonTime) 
  if(PUMPBUTTON.isPressed()) {
    buttonState =!buttonState;
    if (buttonState || subValue ) {
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
