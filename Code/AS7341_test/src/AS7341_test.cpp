/* 
 * Project testing AS7341
 * Author: Kenneth Kinderwater
 * Date: 8-5-25
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_AS7341.h"
#include "Adafruit_BusIO_Register.h"

Adafruit_AS7341 LightSensor;


// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'

// setup() runs once, when the device is first turned on
void setup() {
  Serial.begin(9600);
  delay(1000);

  if (!LightSensor.begin()){
    Serial.println("Could not find AS7341");
  }
  else
  Serial.printf("Light Sensor has checked in\n");


  LightSensor.setATIME(1);
  LightSensor.setASTEP(999);
  LightSensor.setGain(AS7341_GAIN_1X);

  // Put initialization like pinMode and begin functions here
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {

    uint16_t readings[12];
  float counts[12];

  if (!LightSensor.readAllChannels(readings)){
    Serial.println("Error reading all channels!");
    return;
  }
  else{
    Serial.printf("ready to read\n");
  }


for(uint8_t i = 0; i < 12; i++) {
    if(i == 4 || i == 5) continue;
    // we skip the first set of duplicate clear/NIR readings
    // (indices 4 and 5)
    counts[i] = LightSensor.toBasicCounts(readings[i]);
  }

  Serial.print("F1 415nm : ");
  Serial.println(counts[0]);
  Serial.print("F2 445nm : ");
  Serial.println(counts[1]);
  Serial.print("F3 480nm : ");
  Serial.println(counts[2]);
  Serial.print("F4 515nm : ");
  Serial.println(counts[3]);
  Serial.print("F5 555nm : ");
  // again, we skip the duplicates  
  Serial.println(counts[6]);
  Serial.print("F6 590nm : ");
  Serial.println(counts[7]);
  Serial.print("F7 630nm : ");
  Serial.println(counts[8]);
  Serial.print("F8 680nm : ");
  Serial.println(counts[9]);
  Serial.print("Clear    : ");
  Serial.println(counts[10]);
  Serial.print("NIR      : ");
  Serial.println(counts[11]);

  Serial.println();
  
  delay(500);




  // The core of your code will likely live here.

  // Example: Publish event to cloud every 10 seconds. Uncomment the next 3 lines to try it!
  // Log.info("Sending Hello World to the cloud!");
  // Particle.publish("Hello world!");
  // delay( 10 * 1000 ); // milliseconds and blocking - see docs for more info!
}
