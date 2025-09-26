
#include <Servo.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include <TinyGPSPlus.h>
#include "sSoftSerial.h"
#include "Utils.h"
#include "Arduino.h"
#include <AS5600.h>

/* Define pinout */
#define LED_PIN A1
#define BATV_PIN A3
#define KEY_PIN A0
#define DIR_PIN 7 // 7
#define SAIL_PIN 8 // 8
#define GPS_TX 3
#define GPS_RX 4

#define GPS_BAUD 9600

#define WIND_VANE_MIN_VALUE 0.0
#define WIND_VANE_MAX_VALUE 4095.0


extern Servo rudder, sail;
extern GPSPoint lastUpdatedGPSPoint;
extern float GPSspeed;

#ifndef ATTITUDE_STRUCT
#define ATTITUDE_STRUCT
struct Attitude {
  float roll;
  float pitch;
};
#endif

void initHardware();

void updateHardware();

float getWindVaneAngle();

Attitude getAttitude();

Compass getCompass();

float getVoltage();
