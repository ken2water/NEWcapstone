// Example usage for Adafruit_LC709203F library by Limor Fried.

#include "Adafruit_LC709203F.h"

// Initialize objects from the lib
Adafruit_LC709203F adafruit_LC709203F;

void setup() {
    // Call functions on initialized library objects that require hardware
    adafruit_LC709203F.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    adafruit_LC709203F.process();
}
