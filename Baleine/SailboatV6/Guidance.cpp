#include "Guidance.h"

GPSPoint waypoints[NB_MAX_WAYPOINTS], recoveryTarget;
byte waypointsNext[NB_MAX_WAYPOINTS];
byte waypointsLength;
unsigned long guidanceTimer;
boolean navigationEnable;

/* Fonction privées */
void updateWaypointMode();
void updateRecoveryMode();
void updateHeadingMode();
void updateManualMode();
String getStrOfMode(MODE md);

void initGuidance() {
  /* Init variables */
  waypointsLength = 0;
  guidanceTimer = millis();
  recoveryTarget = {0, 0};
  EEPROM.get(ADR_EEPROM_RECOVERY_GPSPOINT, recoveryTarget);
  /* Send to user the recovery target */
  if (LoRa.beginPacket() != 0) {
    LoRa2FloatData('G',recoveryTarget.lat,recoveryTarget.lng,6);
    LoRa.endPacket(true);
  }
  navigationEnable = true;
  /* Init define */
  GUIDANCE_UPDATE_PERIOD = 1000;
  WAYPOINT_RADIUS = 3;
  GUIDANCE_MODE = HEADING;
  TARGET_HEADING = 0;
  RECOVERY_TIME_BEGIN = 30;
  CURRENT_WP_NUM = 0;
}

void shareGuidanceStatus() {
  LoRaStr('m', getStrOfMode(getMode()));
  LoRaIntData('w', getCurrentWpNum());
  if (navigationEnable)
    LoRa2FloatData('t', getTarget().lat, getTarget().lng, 6);
}

void setRecoveryGPSPoint(GPSPoint pt) {
  recoveryTarget = pt;
}

void updateGuidance() {

  if (navigationEnable) {
    updateNavigation();
  }

  if (millis() < guidanceTimer) {
    return;
  }

  guidanceTimer = millis() + GUIDANCE_UPDATE_PERIOD;
  
  // depends on the mode
  switch(GUIDANCE_MODE) {
    case WAYPOINTS:
      updateWaypointMode();
    break;

    case HEADING:
      updateHeadingMode();
    break;

    case MANUAL:
      updateManualMode();
    break;

    default: // or recovery
      updateRecoveryMode();
    break;
  }
  // check if recovery
  if ((millis() - getLastComTime())/1000 > RECOVERY_TIME_BEGIN) {
    if (GUIDANCE_MODE != RECOVERY) {
      // reset navigation parameters for safety
      //initNavigation();
    }
    GUIDANCE_MODE = RECOVERY;
  }
}

void addWaypoint(byte num, byte next, GPSPoint pt) {
  waypoints[num] = pt;
  waypointsNext[num] = next;
  waypointsLength++;
}

void updateWaypointMode() {
  navigationEnable = true;
  if (waypointsLength > 0) {
    setTarget(waypoints[CURRENT_WP_NUM]);
    /* check if waypoint is pass */
    if (distanceBetweenGPSPoints(getGPSPoint(), waypoints[CURRENT_WP_NUM]) < WAYPOINT_RADIUS) {
      // goto next
      CURRENT_WP_NUM = constrain(waypointsNext[CURRENT_WP_NUM], 0, waypointsLength);
    }
  }
}

void updateRecoveryMode() {
  setTarget(recoveryTarget);
  navigationEnable = true;
}

void updateHeadingMode() {
  setPIDHeadingEnable(true);
  setHeadingTarget(TARGET_HEADING);
  navigationEnable = false;
}

void updateManualMode() {
  setPIDHeadingEnable(false);
  navigationEnable = false;
}

MODE getMode() {
  return GUIDANCE_MODE;
}

int getCurrentWpNum() {
  return CURRENT_WP_NUM;
}

String getStrOfMode(MODE md) {
  switch(md) {
    case WAYPOINTS:
      return F("WAYPOINT");
    case HEADING:
      return F("HEADING");
    case MANUAL:
      return F("MANUAL");
    default:
      return F("RECOVERY");
    
  }
}
