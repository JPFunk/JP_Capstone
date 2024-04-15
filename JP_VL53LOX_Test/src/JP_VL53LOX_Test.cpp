/* 
 * Project JP VL53LOX test
 * Author: JP Funk
 * Date: 04/12/2024
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_VL53L0X.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Button.h"
// OLED
const int OLED_RESET=-1;
int rot;
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int TOF;
int targetLoc1, targetLoc2, targetLoc3, targetLoc0;

//void ledOn (int waterPumpPin);
const int LEDPIN = D6; // sharing LED pin with Pump
const int PUMPIN = D6;
void pumpOn (int waterPumpPin);
// Button
//const int BUTTONPIN = D3;
void buttonisClicked();
bool position;
bool changed;
int prevTargetLoc;
bool ledOnOff;
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
  // init done
  //display.display();
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
pinMode (PUMPIN, OUTPUT);

}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    if(measure.RangeMilliMeter <= 100){
      targetLoc1 =1;
      Serial.printf("targetPos 1%i\n", targetLoc1);
    }
    else if(measure.RangeMilliMeter >100 && measure.RangeMilliMeter <=200){
      targetLoc2 =2;
      Serial.printf("targetPos 2%i\n", targetLoc2);
    }
    else if(measure.RangeMilliMeter <310){
      targetLoc3 =3;
      Serial.printf("targetPos 3%i\n", targetLoc3);
    }
  } else {
    targetLoc0 =0;
    Serial.printf("targetPos 0%i\n", targetLoc0);
  }    delay(500);

  position = targetLoc1;
  if (position != prevTargetLoc){
  Serial.printf("%i\n",position);
  prevTargetLoc = position;
  }
   if (position == TRUE){
  ledOnOff= !ledOnOff;
  prevTargetLoc = position;
   digitalWrite(LEDPIN, ledOnOff);
  Serial.printf("LED On\n");
   }
}
  // const int sampleTime = 30000;
  // unsigned int duration, startTime;
  // startTime = 0;
  // targetLoc1 = changed;
  // changed = digitalWrite(LEDPIN, LOW);

  //if ((millis()-startTime) > sampleTime) {
    // if (targetLoc1) {
    //    digitalWrite (LEDPIN, LOW);
    //   display.clearDisplay();
    //   display.setCursor(0,0);
    //   display.setTextSize(3);
    //   display.printf("LED ON%i\n", targetLoc1);
    //   display.display();
    //   targetLoc1 = 1;
    //   startTime = millis();
    //   if (targetLoc1){
    //    digitalWrite(LEDPIN, HIGH);
    //   display.clearDisplay();
    //   display.setCursor(0,0);
    //   display.setTextSize(3);
    //   display.printf("LED OFF%i\n", targetLoc1);
    //   display.display();
    //   targetLoc1 = 0;
    //   startTime = millis();
    //   }
    //  }


        // targetLoc1 = buttonState1;
        // buttonState1 = TRUE;
        // if (buttonState1!= prevButtonState1){
        //   if (buttonState1 == TRUE)
        //   display.printf("ON %i\n",targetLoc1);//OLED printf code Hue Brightness Encoder Potentiometer-----------------
        //   changed = !changed; 
        //   prevButtonState1= buttonState1;
        // digitalWrite (LEDPIN, changed); 
        //  }

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

  // targetLoc1 =(changed)?
  // digitalWrite (LEDPIN, HIGH);

void pumpOn (int waterPumpPin) { // Water Pump OnOff function with serial print values
  digitalWrite(waterPumpPin, HIGH);
  Serial.printf("Pump On%i\n", pumpOn);
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.printf(" Pump !On!", pumpOn);
  digitalWrite(waterPumpPin, LOW);
  display.display();
  delay (3000);
}
