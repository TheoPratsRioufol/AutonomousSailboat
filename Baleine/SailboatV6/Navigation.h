
// For high abstraction navigation plan

/* Requested libs */

#include "Parameters.h"
#include "Abstraction.h"
#include "Utils.h"

#ifndef COMM_NAV_HEADER
#define COMM_NAV_HEADER
#include "Communication.h"
#endif

/* Defines */

#define DIRECT_TRAJECTORY true
#define INDIRECT_TRAJECTORY false
#define BABORD_AMURE true
#define TRIBORD_AMURE false

#define ALLURE_PRES intVariables[0]
#define MAX_DISTANCE_AMURE intVariables[1]
#define TREASHOLD_END_TURN intVariables[2]
#define TURN_AMPL intVariables[3]
#define TRESHOLD_UPWIND intVariables[4]
#define HYSTERESIS_TRAJECTORY intVariables[5]
#define NAVIGATION_UPDATE_PERIOD intVariables[6]
#define JIBE intVariables[20]
#define MIN_DISTANCE_AMURE intVariables[24]
#define RETRY_TURN_PERIOD intVariables[25]
#define AUTO_SAIL intVariables[28]

#define TURN_RIGHT true
#define TURN_LEFT false

/* Fonction publique de cette librairie */

void initNavigation();
void shareNavigationStatus();
void setTarget(GPSPoint point);
GPSPoint getTarget();
String getNavState();
void setTurnPolicy(boolean jibeSelected); 
void updateNavigation();
void finishManoeuvre();
float getSailPosition();
void changeAmure();
