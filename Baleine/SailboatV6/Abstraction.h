
// To control the harware of the sailboat

#include "Arduino.h"
#include "Parameters.h"
#include "Utils.h"
#include "Hardware.h"

#ifndef COMM_ABS_HEADER
#define COMM_ABS_HEADER
#include "Communication.h"
#endif

/* Defines */

#define P_PID_HEARDING floatVariables[0]
#define REST_ANG_RUDDER intVariables[7]
#define AMP_RUDDER intVariables[8]
#define MAX_SLOPE_RUDDER floatVariables[1]
#define T_WIND_FILTER floatVariables[2]
#define ABSTRACTION_UPDATE_PERIOD intVariables[9]
#define COMPASS_NORTH_OFFSET intVariables[10]
#define MIN_SAIL_ANG intVariables[11]
#define MAX_SAIL_ANG intVariables[12]
#define OFFSET_WIND_VANE intVariables[21]
#define WIND_MANUAL intVariables[22]
#define WIND_MANUAL_VALUE intVariables[23]
#define UPDATE_SAIL_PERIOD intVariables[26]
#define UPDATE_SAIL_ANG_TREASHOLD intVariables[27]
#define ENABLE_SAFE_SAIL intVariables[30]

/* fonction d'initailaisation/reset */

void initAbstraction();

/* Capteurs */

float getHeading();
float getAbsoluteWindDir();
GPSPoint getGPSPoint();
float getHeadingTarget();

/* Actionneurs */

void setHeadingTarget(float target);
void setPIDHeadingEnable(boolean enable);
float setRudder(float ang); // 0 = neutre, négatif pour aller à gauche, positif à doite. Veille à ne pas saturer
float setSail(float ang);
void requestSetSail(float ang);
void ledOn(unsigned long Dt);

/* Pour faire fonctionner l'affaire */

void updateAbstraction();

/* Communication */

void shareAbstractionStatus();
