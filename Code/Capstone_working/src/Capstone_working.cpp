/* 
 * Project Final Capstone
 * Author: Kenneth Kinderwater
 * Date: 8-4-2025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include <Adafruit_AS7341.h>
#include <Adafruit_LC709203F.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Spark.h"
#include <GPS_CNM.h>
#include <Credentials.h>
#include "Adafruit_GPS.h"
#include "JsonParserGeneratorRK.h"


// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

//MQTT Setup
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

//Publish and Subscribe 
Adafruit_MQTT_Publish circutTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/circutree.tempffeed");
Adafruit_MQTT_Publish circutWind = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/circutree.windspeed");
Adafruit_MQTT_Publish circutCond = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/circutree.condition");
Adafruit_MQTT_Publish circutLocation = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/circutree.location");

//delcare sensors 
Adafruit_AS7341 LightSensor;
Adafruit_LC709203F BatteryMonitor; 
Adafruit_LC709203F BattMon;
Adafruit_GPS GPS(&Wire);

//GPS confiugre 
const int TIMEZONE = -6;
const unsigned int UPDATE = 30000;
float lat, lon, alt, location;

int sat;
unsigned int lastGPS;


//WeatherWebhook confiure 
const char *EVENT_Name = "GetWeatherData";
float lat42,lon42;
float tempF,windSpeed;
String condition;
bool getNewWeather;

//Time Variables
int hours, minutes, lasthour, lastminute;


//Declare Functions 
void MQTT_connect();
bool MQTT_ping();
void getGPS(float *latitude, float *longitude, float *altitude, int *satellites);
void subscriptionHandler(const char *event,const char *data);
void currentLocation(float lat, float lon);


void setup() {
  //start sensors 
  Serial.begin(9600);


  //webhook
  String subscriptionName = String::format("%s/%s/", System.deviceID().c_str(), EVENT_Name);
  Particle.subscribe(subscriptionName, subscriptionHandler, MY_DEVICES);
  Serial.printf("subscribing to %s\n", subscriptionName.c_str());

  //GPS startup
  {
  GPS.begin(0x10);
  if (!GPS.begin(0x10)){
    Serial.printf("GPS Failed to check in\n");
    }
  else{
    Serial.printf("GPS has checked in\n");
    }
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);
  }



}
  
void loop() {
  //Connecting to Adafruit
  MQTT_connect();
  MQTT_ping();
  Time.zone(TIMEZONE);//setting time zone
{
  //GPS read 
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }   
  }
    if (millis() - lastGPS > UPDATE) {
    lastGPS = millis();
    getGPS(&lat,&lon,&alt,&sat);
    lat42 = lat;
    lon42 = lon;
    Particle.publish(EVENT_Name, String::format("{\"lat42\":%0.5f,\"lon42\":%0.5f}",lat42,lon42),PRIVATE);
    Serial.printf("\n=================================================================\n");
    Serial.printf("Lat: %0.6f, Lon: %0.6f, Alt: %0.6f, Satellites: %i\n",lat, lon, alt, sat);
    Serial.printf("=================================================================\n");
    Serial.printf("publishing data...\n");
  circutTemp.publish(tempF);
  circutWind.publish(windSpeed);
  circutCond.publish(condition);
  currentLocation(lat,lon);

  }
}

//calling webhook ADD TIME INTERVAL 
// Lat42 = lat;
// Lon42 = lon;
// Lat42 = 35.1;
// Lon42 = -106.6;
  // minutes = Time.minute();
  // hours = Time.hour();
  // if((minutes != lastminute)&&(minutes%2 == 0)&&(hours%0 ==0)) {
  //   Serial.printf("Requesting data\n");
// Serial.printf("publishing data...\n");
// circutTemp.publish(tempF);
// circutWind.publish(windSpeed);



}


void getGPS(float *latitude, float *longitude, float *altitude, int *satellites){
  int theHour;

  theHour = GPS.hour + TIMEZONE;
  if(theHour < 0) {
    theHour = theHour + 24;
  }
    
  Serial.printf("Time: %02i:%02i:%02i:%03i\n",theHour, GPS.minute, GPS.seconds, GPS.milliseconds);
  Serial.printf("Dates: %02i-%02i-20%02i\n", GPS.month, GPS.day, GPS.year);
  Serial.printf("Fix: %i, Quality: %i",(int)GPS.fix,(int)GPS.fixquality);
    if (GPS.fix) {
      *latitude = GPS.latitudeDegrees;
      *longitude = GPS.longitudeDegrees; 
      *altitude = GPS.altitude;
      *satellites = (int)GPS.satellites;
    }
}

void subscriptionHandler(const char *event, const char *data) {
  JSONValue outerObj = JSONValue::parseCopy(data);
  JSONObjectIterator iter(outerObj);
    Serial.printf("\nWeather at %2i:%02i\n",Time.hour(),Time.minute());
    while(iter.next()) {
        if (iter.name() == "lat") {
            Serial.printf("Latitude: %0.6f\n", iter.value().toDouble());
        }
        if (iter.name() == "lon") {
            Serial.printf("Longitude: %0.6f\n", iter.value().toDouble());
        }
        if (iter.name() == "temp") {
            tempF = iter.value().toDouble();
            Serial.printf("Temperature: %0.2f(F)\n", iter.value().toDouble());
        }
        if (iter.name() == "wind") {
            windSpeed = iter.value().toDouble();
            Serial.printf("Wind Speed: %0.2f (MPH)\n", iter.value().toDouble());
        }
        if (iter.name() == "conditions") {
            condition = iter.value().toString().data();
            Serial.printf("Conditions: %s\n", (const char *)iter.value().toString());
        }       
    }
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

void currentLocation(float lat, float lon){
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);

    jw.insertKeyValue("lat", lat);
    jw.insertKeyValue("lon", lon);
  }
  circutLocation.publish(jw.getBuffer());


}