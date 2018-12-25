#include <math.h>
#define TRIPOD_HEIGHT   1.5             //Height of antennas over 0m (ground).
#define EARTH_RADIUS    6372795         //m
#define PRECISE_PI      3.14159265359

float altitudeUAV;
float distanceUAV;


float getAzimutAngle(){
}

float getElevationAngle(){
  return atan2((altitudeUAV-TRIPOD_HEIGHT),distanceUAV);
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
}

void loop(){
  
}
