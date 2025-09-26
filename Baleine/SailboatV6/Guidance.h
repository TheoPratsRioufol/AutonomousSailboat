
#include "Abstraction.h"
#include "Navigation.h"
#include <EEPROM.h>

#ifndef COMM_HEADER
#define COMM_HEADER
#include "Communication.h"
#endif

/* C'est le cerveau haut niveau du bateau. Gère les waypoint, le recovery */

#define NB_MAX_WAYPOINTS 5
#define WAYPOINT_RADIUS intVariables[14]
#define GUIDANCE_MODE intVariables[15]
#define TARGET_HEADING intVariables[16]
#define GUIDANCE_UPDATE_PERIOD intVariables[17]
#define RECOVERY_TIME_BEGIN intVariables[18]
#define CURRENT_WP_NUM intVariables[19]


#ifndef MODE_ENUM
#define MODE_ENUM
enum MODE {
  WAYPOINTS = 0,
  HEADING = 1,
  RECOVERY = 2,
  MANUAL = 3
};
#endif

void initGuidance();

void shareGuidanceStatus();

void updateGuidance();

void addWaypoint(byte num, byte next, GPSPoint pt);

MODE getMode();

int getCurrentWpNum();

void setRecoveryGPSPoint(GPSPoint pt);
