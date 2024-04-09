/* 
 * Project JP VL53LOX test
 * Author: JP Funk
 * Date: 04/08/2024
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_VL53L0X.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// OLED
const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// #if (SSD1306_LCDHEIGHT != 32)
//  #error("Height incorrect, please fix Adafruit_SSD1306.h!");
// #endif

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

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done
  display.display();
  delay(1000);
  
  Wire.begin();

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }

  // text display big!
  display.setTextSize(4);
  display.setTextColor(WHITE);
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
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {

    VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      display.clearDisplay();
      display.setCursor(0,0);
      display.setTextSize(0);
      display.print(measure.RangeMilliMeter);
      display.print("mm");
      display.display();
      Serial.println();
      delay(50);
  } else {
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);
    display.print("Out of Range");
    display.display();
    return;
  }
  // Code Serial Print VL53LOX test
  // VL53L0X_RangingMeasurementData_t measure;
    
  // Serial.print("Reading a measurement... ");
  // lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  // if (measure.RangeStatus != 4) {  // phase failures have incorrect data
  //   Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  // } else {
  //   Serial.println(" out of range ");
  // }
    
  // delay(100);
}