#include "Navigation.h"


unsigned long updateTimer, turnTimer;
boolean turnDirection, amure, turning, indirectCourseStarted;
int endTurnAngle;
GPSPoint target, amureStartingPoint;

/* Définitions des fonction privées : */

void updateTrajectory();
boolean directRoutePossible(boolean mode, float targetAngle);
void beginIndirectCourse();
void updateIndirectCourse();
boolean isTurnNeeded(float outputAngle);
void turn(float angle);
void updateTurn();
float getTargetHeading();
void viser(float angle);
float getAngleAllureAmure(float allure, boolean amure);
int getDistanceInAmure();
int resetDistanceAmure();
float getRudderAngleTurn(boolean turnDir);
boolean getAmure(int ang);

/* Fonction publiques */

void initNavigation() {
  /* Setup variables */
  amure = true;
  turnDirection = TURN_LEFT;
  endTurnAngle = 0;
  target = {0, 0};
  amureStartingPoint = {0, 0};
  turning = false;
  indirectCourseStarted = false;
  turnTimer = millis();
  updateTimer = millis();
  
  /* Setup Parameters */
  AUTO_SAIL = 1;
  MAX_DISTANCE_AMURE = 30;
  MIN_DISTANCE_AMURE = 4;
  ALLURE_PRES = 45;
  TREASHOLD_END_TURN = 35;
  TURN_AMPL = 25;
  TRESHOLD_UPWIND = 35;
  HYSTERESIS_TRAJECTORY = 10;
  RETRY_TURN_PERIOD = 16;
  JIBE = true;
  #ifdef SIMULATION
  NAVIGATION_UPDATE_PERIOD = 100/SIMU_TIME_FACTOR;
  #else
  NAVIGATION_UPDATE_PERIOD = 100;
  #endif
}

void shareNavigationStatus() {
  LoRaFloatData('o', angleFromGPSPoints(getGPSPoint(), getTarget()));
  LoRaStr('_', getNavState());
}

void setTarget(GPSPoint point) {
  target = point;
}

GPSPoint getTarget() {
  return target;
}

String getNavState() {
  if (turning) {
      if (turnDirection == TURN_LEFT)
         return F("TURN LEFT");
      else
         return F("TURN RIGHT");
  } else if (indirectCourseStarted)
      return F("INDIRECT");
  else
      return F("DIRECT");
}

void setTurnPolicy(boolean jibeSelected) {
  JIBE = jibeSelected;
}

void updateNavigation() {
  if (millis() < updateTimer) {
    return;
  }

  updateTimer = millis() + NAVIGATION_UPDATE_PERIOD;
  
  if (turning) { /* For turns between strait lines */
    updateTurn();
  } else { /* For strait lines between turns */
    /* Update sail position */
    if (AUTO_SAIL != 0) {
      requestSetSail(getSailPosition());
    }
    updateTrajectory();
  }
}


void updateTrajectory() {
    if (indirectCourseStarted) {
      updateIndirectCourse();
      /* Look if indirect course is still mandatory */
      if (directRoutePossible(INDIRECT_TRAJECTORY, getTargetHeading()) && (getDistanceInAmure() > MIN_DISTANCE_AMURE)) {
          indirectCourseStarted = false;
      } else {
          return;
      }
    }
  
    /* update direct course */
    if (!directRoutePossible(DIRECT_TRAJECTORY, getTargetHeading())) {
      beginIndirectCourse();
    } else {
      if (isTurnNeeded(getTargetHeading())) { /* Si on ne peut pas y aller en un bord, ça set à rien de tourner */
        turn(getTargetHeading());
        return;
      }
      viser(getTargetHeading());
    }
  
}

void changeAmure() {
  /* Available only for indirect course */
  if (!indirectCourseStarted)
    return;
  amure = !amure;
  resetDistanceAmure();
  turn(getAngleAllureAmure(ALLURE_PRES, amure));
}

boolean directRoutePossible(boolean mode, float targetAngle) {
  float epsilon = abs(angularDiff(getAbsoluteWindDir(), targetAngle));

  if (mode == DIRECT_TRAJECTORY) {
    return (epsilon > TRESHOLD_UPWIND - HYSTERESIS_TRAJECTORY);
  }

  if (mode == INDIRECT_TRAJECTORY) {
    return (epsilon > TRESHOLD_UPWIND + HYSTERESIS_TRAJECTORY);
  }
  return true;
}

void beginIndirectCourse() {
  indirectCourseStarted = true;
  amure = !getAmure(getHeading());
  resetDistanceAmure();
}

void updateIndirectCourse() {
    viser(getAngleAllureAmure(ALLURE_PRES, amure));

    if (isTurnNeeded(getAngleAllureAmure(ALLURE_PRES, amure))) { /* Si les vagues ont fait tourner, on change d'allure */
        amure = !amure;
        resetDistanceAmure();
        return;
      }

    if (getDistanceInAmure() > MAX_DISTANCE_AMURE) {
      amure = !amure;
      resetDistanceAmure();
      turn(getAngleAllureAmure(ALLURE_PRES, amure));
      return;
    }
}

boolean isTurnNeeded(float outputAngle) {
  /* Turn is mandatory if the smallest side between getHeading() and outputAngle (< 180) cross "face au vent" */
  float inputAngle = getHeading();
  
  if (isALeftB(inputAngle, outputAngle)) {
    /* le côté 180 est de input vers output*/
    return isBetweenAandB(getAbsoluteWindDir(), inputAngle, outputAngle);
  } else {
    /* le côté 180 est de output vers input */
    return isBetweenAandB(getAbsoluteWindDir(), outputAngle, inputAngle);
  }
}

void turn(float angle) {
  turning = true;
  endTurnAngle = angle;
  turnTimer = millis();
  setPIDHeadingEnable(false);
  /* Then, choose the rudder direction */
  if (JIBE) {
    // need to not cross the wind
    if (isALeftB(getAbsoluteWindDir(), getHeading())) {
      turnDirection = TURN_RIGHT;
    } else {
      turnDirection = TURN_LEFT;
    }
  } else {
    // fastest
    if (isALeftB(getHeading(), angle)) {
      turnDirection = TURN_RIGHT;
    } else {
      turnDirection = TURN_LEFT;
    }
  }
  setRudder(getRudderAngleTurn(turnDirection));
}

void updateTurn() {
  if (abs(angularDiff(getHeading(), endTurnAngle)) < TREASHOLD_END_TURN) {
    finishManoeuvre();
  }

  if ((millis() - turnTimer)/1000 > RETRY_TURN_PERIOD) {
    /* On est bloqué depuis 10 s, on essaye de tourner dans l'autre sens */
    turnTimer = millis();
    turnDirection = !turnDirection;
    setRudder(getRudderAngleTurn(turnDirection));
  }
}

void finishManoeuvre() {
  setPIDHeadingEnable(true);
  turning = false;
}

float getTargetHeading() {
  return angleFromGPSPoints(getGPSPoint(), target);
}

void viser(float angle) {
  setPIDHeadingEnable(true);
  setHeadingTarget(angle);
}

float getAngleAllureAmure(float allure, boolean amure) {
  float sign = 1 - 2*(amure == BABORD_AMURE);
  return floatModulo(getAbsoluteWindDir() + allure*sign + 360, 360);
}

int getDistanceInAmure() {
  return distanceBetweenGPSPoints(amureStartingPoint, getGPSPoint());
}

int resetDistanceAmure() {
  amureStartingPoint = getGPSPoint();
}

boolean getAmure(int ang) {
  int wind = getAbsoluteWindDir();
  int alpha = wind + 180;
  if ((wind < ang) && (ang < alpha)) {
    return BABORD_AMURE;
  }
  if ((alpha > 360) && (0 < ang) && (ang < alpha%360)) {
    return BABORD_AMURE;
  }
  return TRIBORD_AMURE;
}

float getRudderAngleTurn(boolean turnDir) {
  if (turnDir == TURN_RIGHT) {
    return (float) TURN_AMPL;
  }
  return (float) -TURN_AMPL;
}

float getSailPosition() {
  int relWind = floatModulo(getAbsoluteWindDir() - getHeading()+360, 360);
  /* Clip : */
  if (relWind > 180)
      relWind = - (360 - relWind);
  relWind = constrain(abs(relWind), 30, 180);
  relWind = map(relWind, 30, 180, MIN_SAIL_ANG, MAX_SAIL_ANG);
  return relWind;
}
