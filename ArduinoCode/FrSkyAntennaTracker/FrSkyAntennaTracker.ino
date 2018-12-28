/*
 * 
 *                       FRSKY ANTENNA TRACKER
 *                       
 * Open source Arduino based FrSky SmartPort compatible RC Antenna Tracker. 
 * Uses GPS and Barometer SmartPort telemetry to calculate Azimuth 
 * and Elevation angles and apply them in a closed loop with a Magnetometer 
 * sensor for Azimuth and an Accelerometer sensor for Elevation. 
 * 
 * 
 * 
 * Aaron G.
 * Dec 2018
 */
 
#include "FrSkySportSensor.h"
#include "FrSkySportSensorXjt.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorVario.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportDecoder.h"
#include "SoftwareSerial.h"
#include <math.h>
#define TRIPOD_HEIGHT   1.5             //Height of antennas over 0m (ground).
#define EARTH_RADIUS    6372795         //m
#define PRECISE_PI      3.14159265359
#define UPDATE_DELAY      200  //ms
#define TIME_OUT          2000 //ms

FrSkySportSensorXjt xjtSensor;
FrSkySportSensorFcs fcsSensor;
FrSkySportSensorGps gpsSensor;
FrSkySportSensorRpm rpmSensor;
FrSkySportSensorVario varioSensor;
FrSkySportDecoder smartPortDecoder;

uint32_t currentTime,updateTime,lastReceived;
uint16_t decodeResult;
float altitudeUAV;
float distanceUAV;


float getAzimuth(){
}

float getElevation(){
}


float computeAzimuth(){
}

float computeElevationAngle(){
  return atan2((altitudeUAV-TRIPOD_HEIGHT),distanceUAV);
}


void applyAzimuth(){
}

void applyElevation(){
}


void computeDistance(float lat1, float long1, float lat2, float long2){
  // returns distance in meters between two positions, both specified 
  // as signed decimal-degrees latitude and longitude. Uses great-circle 
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1-long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  distanceUAV = delta * EARTH_RADIUS; 
}




void setup(){
  Serial.begin(57600);
  smartPortDecoder.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_3,&xjtSensor,&fcsSensor,&gpsSensor,&rpmSensor,&varioSensor);
}

void loop(){


  decodeResult = smartPortDecoder.decode();
  if(decodeResult != SENSOR_NO_DATA_ID) lastReceived = millis();
  
  currentTime = millis();
  if((currentTime > updateTime) && ((currentTime - lastReceived) < TIME_OUT)){
    updateTime = currentTime + UPDATE_DELAY;




    gpsSensor.getLat()
    gpsSensor.getLon()
    varioSensor.getAltitude()


    
    
  }
  
  
}
