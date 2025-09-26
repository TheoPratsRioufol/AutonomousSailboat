
#include <TinyGPSPlus.h>

/* type de donné utile */

#ifndef GPSPOINT_STRUCT
#define GPSPOINT_STRUCT

struct GPSPoint {
  float lat;
  float lng;
};

struct Compass {
  int x;
  int y;
  int z;
};

#endif

/* Fonction utiles */

boolean isALeftB(float angA, float angB);

float floatModulo(float a, float m);

float angularDiff(float a, float b);

int distanceBetweenGPSPoints(GPSPoint ptA, GPSPoint ptB);

float angleFromGPSPoints(GPSPoint ptA, GPSPoint ptB);

boolean isBetweenAandB(float ang, float a, float b);
