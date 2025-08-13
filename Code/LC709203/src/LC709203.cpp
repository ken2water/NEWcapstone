/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
// #include <Adafruit_Bus_IO.h>
#include "Adafruit_LC709203F.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

Adafruit_LC709203F voltGage;
float battPercent;
float battvolt;

// setup() runs once, when the device is first turned on
void setup() {

  if (!voltGage.begin()) {
    Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
  }
  Serial.printf("Found LC709203F\n");
  voltGage.setPackSize(LC709203F_APA_3000MAH);



  Serial.begin(9600);

  // Put initialization like pinMode and begin functions here
}

// loop() runs over and over again, as quickly as it can execute.
void loop() { 
  delay(100000);
  battvolt = voltGage.cellVoltage();
  battPercent = voltGage.cellPercent();
  Serial.printf("Battery Voltage %f\n Battery Percentage %f\n",battvolt,battPercent);
  // The core of your code will likely live here.

  // Example: Publish event to cloud every 10 seconds. Uncomment the next 3 lines to try it!
  // Log.info("Sending Hello World to the cloud!");
  // Particle.publish("Hello world!");
  // delay( 10 * 1000 ); // milliseconds and blocking - see docs for more info!
}
