/* 
 * Project JP_Reset
 * Author: JP Funk
 * Date: 04/13/2024
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// setup() runs once, when the device is first turned on
void setup() {
  // Put initialization like pinMode and begin functions here
  Serial.begin(9600);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  //capture the time the board has been running
  int time = millis()/1000;
  
  //print the time from last reset
  Serial.print("I was reset ");
  Serial.print(time);
  Serial.println(" seconds ago.");
  Serial.println("Will you stop Dave. I am afraid.");
  delay(1000);
}
