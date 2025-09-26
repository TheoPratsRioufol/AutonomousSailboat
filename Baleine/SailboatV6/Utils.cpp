#include "Utils.h"

boolean isALeftB(float angA, float angB) {
  return ((int(angA - angB + 360))%360 > 180);
}

float floatModulo(float a, float m) {
  return a-int(a/m)*m;
}

float angularDiff(float a, float b) {
  float epsilon = floatModulo(a - b + 360,360);
  if (epsilon > 180)
    epsilon = epsilon - 360;
  return -epsilon;
}

int distanceBetweenGPSPoints(GPSPoint ptA, GPSPoint ptB) {
  return TinyGPSPlus::distanceBetween(ptA.lat, ptA.lng, ptB.lat, ptB.lng);
}

float angleFromGPSPoints(GPSPoint ptA, GPSPoint ptB) {
    float dLon = (ptB.lng - ptA.lng);
    float y = sin(dLon) * cos(ptB.lat);
    float x = cos(ptA.lat) * sin(ptB.lat) - sin(ptA.lat) * cos(ptB.lat) * cos(dLon);
    
    float brng = atan2(y, x)/0.0174532925;
    
    return floatModulo((brng + 2.0*360.0 - 90.0), 360.0);
}

boolean isBetweenAandB(float ang, float a, float b) {
  if (a < b) {
    // cas classique
    return (ang > a) && (ang < b);
  } else {
    // on regarde de a jusqu'à 360 et de 0 jusqu'à b
    return ((ang > a) && (ang < 360)) || ((ang > 0) && (ang < b));
  }
}
