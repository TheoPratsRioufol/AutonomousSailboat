
#include "Abstraction.h"

//#define SIMULATION

#ifndef SIMULATION

unsigned long lastRudderWriteTime, updateTimerAbstraction, ledTimer, sailTimer;
float filterAbsWindDirSin, filterAbsWindDirCos, heading, headingTarget, filterAbsWindDir, rudderTarget, requestSailAng;
boolean PIDHeadingEnable;

/* fonction provées */
void updateRudder();

void initAbstraction() {
  initHardware();
  /* Initialisation des variables */
  lastRudderWriteTime = millis();
  updateTimerAbstraction = millis();
  ledTimer = millis();
  sailTimer = millis();
  filterAbsWindDirSin = 0;
  filterAbsWindDirCos = 1;
  filterAbsWindDir = 0;
  heading = 0;
  headingTarget = 0;
  PIDHeadingEnable = true;
  rudderTarget = 0;
  requestSailAng = 60;
  /* initialisation des paramètres */
  P_PID_HEARDING = 2.5;
  REST_ANG_RUDDER = 72;
  AMP_RUDDER = 25;
  MAX_SLOPE_RUDDER = 40; // limite de vitesse angulaire du servo de barre (degré par seconde)
  T_WIND_FILTER = 7; // temps caractéristique en seconde
  ABSTRACTION_UPDATE_PERIOD = 100;
  COMPASS_NORTH_OFFSET = (360-90) - 191;
  MIN_SAIL_ANG = 35;
  MAX_SAIL_ANG = 90;
  OFFSET_WIND_VANE = 32;
  WIND_MANUAL_VALUE = 0;
  WIND_MANUAL = 0;
  UPDATE_SAIL_PERIOD = 4000;
  UPDATE_SAIL_ANG_TREASHOLD = 5;
  ENABLE_SAFE_SAIL = 1;
}

void shareAbstractionStatus() {
  LoRaFloatData('A', getHeading());
  LoRa2FloatData('G', getGPSPoint().lat, getGPSPoint().lng, 6);
  LoRaFloatData('W', getAbsoluteWindDir());
  LoRaIntData('r', rudder.read());
  LoRaIntData('T', getHeadingTarget());
  LoRaFloatData('V', getVoltage());
  LoRaFloatData('S', GPSspeed);
}

/* Getters */

float getHeading() {
  return heading;
}

float getHeadingTarget() {
  return headingTarget;
}

float getAbsoluteWindDir() {
  return filterAbsWindDir;
}

GPSPoint getGPSPoint() {
  return lastUpdatedGPSPoint;
}

/* Updaters */

void updateHeading() {
  Attitude attitude = getAttitude();
  Compass comp = getCompass();
  float Xhorizontal = ((float) comp.x)*cos(attitude.roll) + ((float) comp.z)*sin(attitude.roll);
  float Yhorizontal = ((float) comp.y)*cos(attitude.pitch) + ((float) comp.z)*sin(attitude.pitch);
  heading = atan2(Xhorizontal, Yhorizontal)/0.0174532925;
  
  if  (heading < 0) heading+=360;
      heading=360-heading;
  heading = floatModulo(heading + COMPASS_NORTH_OFFSET + 360, 360);
}

void updateAbsoluteWindDirFilter(float dt) {
  // filtre passe bas de fréquence T_WIND_FILTER, dt en seconde
  float alpha = T_WIND_FILTER/dt;
  if (WIND_MANUAL == 0) {
    float windValue = floatModulo(OFFSET_WIND_VANE - getWindVaneAngle() + heading + 360, 360);
    // to hande 350 to 10 deg issue, sin and cos are used and then recombined
    filterAbsWindDirCos = (cos(windValue*0.0174532925) + alpha*filterAbsWindDirCos)/(1 + alpha); 
    filterAbsWindDirSin = (sin(windValue*0.0174532925) + alpha*filterAbsWindDirSin)/(1 + alpha); 
    filterAbsWindDir = floatModulo(atan2(filterAbsWindDirSin, filterAbsWindDirCos)/0.0174532925 + 360, 360);
  } else {
    filterAbsWindDir = WIND_MANUAL_VALUE;
  }
}

void updatePIDHeading() {
  float epsilon = angularDiff(getHeading(), headingTarget);
  setRudder(P_PID_HEARDING*epsilon);
}

void updateAbstraction() {
  updateHardware();
  
  if (millis() > ledTimer) {
    digitalWrite(LED_PIN, LOW);
  }
  
  if (millis() < updateTimerAbstraction) {
    return;
  }

  updateTimerAbstraction = millis() + ABSTRACTION_UPDATE_PERIOD;

  //if ((ENABLE_SAFE_SAIL == 1) && ((abs(getAttitude().pitch/0.0174532925) > 40) || (abs(getAttitude().roll/0.0174532925) > 40))) {
  if ((ENABLE_SAFE_SAIL == 1) && (abs(getAttitude().pitch/0.0174532925) > 40)) {
    sailTimer = millis() + UPDATE_SAIL_PERIOD;
    setSail(MAX_SAIL_ANG);
  }
  updateAbsoluteWindDirFilter(((float) ABSTRACTION_UPDATE_PERIOD)/1000.0);
  updateHeading();
  if (PIDHeadingEnable)
      updatePIDHeading();
  updateRudder();
}

/* Setters */

void setPIDHeadingEnable(boolean enable) {
  PIDHeadingEnable = enable;
}

float setRudder(float ang) {
  rudderTarget = ang;
}

void updateRudder() {
  float dt = ((float) (millis() - lastRudderWriteTime))/1000.0;
  lastRudderWriteTime = millis();
  
  int saturedAng = REST_ANG_RUDDER + constrain(rudderTarget, -AMP_RUDDER, AMP_RUDDER);
  // on lit l'angle du servo
  int addingAng = saturedAng - rudder.read(); // angle à ajouter
  // on limite la rampe
  addingAng = constrain(addingAng, -MAX_SLOPE_RUDDER*dt, MAX_SLOPE_RUDDER*dt);
  // on bouge le servo
  rudder.write(rudder.read() + addingAng);
}

float setSail(float ang) {
  sail.write(constrain(ang, MIN_SAIL_ANG, MAX_SAIL_ANG));
}

void requestSetSail(float ang) {
  requestSailAng = ang;
  if (millis() < sailTimer) {
    return;
  }
  sailTimer = millis() + UPDATE_SAIL_PERIOD;
  if (abs(angularDiff(sail.read(), requestSailAng)) > UPDATE_SAIL_ANG_TREASHOLD) {
    setSail(requestSailAng);
  }
}

void setHeadingTarget(float target) {
  headingTarget = target;
}

void ledOn(unsigned long Dt) {
  digitalWrite(LED_PIN, HIGH);
  ledTimer = millis() + Dt;
}

#else

GPSPoint simuGPSpoint, offsetGPSPoint;
float simuAbsWindDir, simuHeading, simuHeadingActual, turnSpeed, simuheadingTarget;
unsigned long updateTimerSimu, lastHeadingWriteTime, updateTimerEuler;
boolean PIDHeadingEnable;

#define SIMU_STARTING_POINT {0, 0};
#define SIMU_UPDATE_PERIOD 200/SIMU_TIME_FACTOR
#define SIMU_EULER_PERIOD 50/SIMU_TIME_FACTOR
#define SIMU_BOAT_VELOCITY 8*SIMU_TIME_FACTOR // m/s
#define SIMU_TURN_SLOPE 20*SIMU_TIME_FACTOR // deg/s
#define SIMU_AMP_RUDDER 25

#define EARTH_RADIUS 6371001
#define LAT_REF 43

void initAbstraction() {
  /* init variables for the simulation */
  simuGPSpoint = {0, 0};
  offsetGPSPoint = SIMU_STARTING_POINT;
  simuAbsWindDir = 170;
  simuHeading = 0;
  simuHeadingActual = simuHeading;
  turnSpeed = 0;
  PIDHeadingEnable = true;
  updateTimerSimu = millis();
  lastHeadingWriteTime = millis();
  updateTimerEuler = millis();
}
float getHeading() {
  return simuHeading;
}
float getAbsoluteWindDir() {
  return simuAbsWindDir;
}
GPSPoint getGPSPoint() {
  return {simuGPSpoint.lat + offsetGPSPoint.lat, simuGPSpoint.lng + offsetGPSPoint.lng};
}
float setRudder(float ang) {
  // translate in term of turn speed
  turnSpeed = SIMU_TURN_SLOPE*ang/SIMU_AMP_RUDDER;
}
float setSail(float ang) {
  
}
void ledOn(unsigned long Dt) {
  
}

void updatePIDHeading(){
  turnSpeed = 0;
  float dt = (millis() - lastHeadingWriteTime)/1000;
  if (dt < 0.1)
    return;
  lastHeadingWriteTime = millis();
  int addingAng = -angularDiff(simuheadingTarget, simuHeadingActual);
  addingAng = constrain(addingAng, -SIMU_TURN_SLOPE*dt, SIMU_TURN_SLOPE*dt);
  simuHeadingActual += addingAng;
  simuHeadingActual = floatModulo(simuHeadingActual, 360);
}

void setHeadingTarget(float target) {
  simuheadingTarget = target;
}

void setPIDHeadingEnable(boolean enable) {
  PIDHeadingEnable = enable;
}

float getHeadingTarget() {
  return simuheadingTarget;
}

void updateEuler() {
  if (millis() < updateTimerEuler) {
    return;
  }

  updateTimerEuler = millis() + SIMU_EULER_PERIOD;

  /*
   * Move sailboat :
  x = r*lng*cos(lat_ref); x = x + speed*cos(heaging) => lng = lng + peed*cos(heaging)/(r*cos(lat_ref));
  y = r*lat; y += speed*sin(heading) => lat = lat + speed*sin(heading)/r;
   */
  simuGPSpoint.lng += SIMU_BOAT_VELOCITY*cos(-simuHeading*0.0174532925)*SIMU_UPDATE_PERIOD/(1000*EARTH_RADIUS*cos(LAT_REF*0.0174532925));
  simuGPSpoint.lat += SIMU_BOAT_VELOCITY*sin(-simuHeading*0.0174532925)*SIMU_UPDATE_PERIOD/(1000*EARTH_RADIUS);
  simuHeadingActual += turnSpeed*SIMU_EULER_PERIOD/1000;
  simuHeadingActual = floatModulo(simuHeadingActual, 360);
}

void updateAbstraction() {
  updateEuler();
  
  if (millis() < updateTimerSimu) {
    return;
  }

  updateTimerSimu = millis() + SIMU_UPDATE_PERIOD;
  if (PIDHeadingEnable)
      updatePIDHeading();
  simuHeading = simuHeadingActual;
}

void shareAbstractionStatus() {
  LoRaFloatData('A', getHeading());
  LoRa2FloatData('G', getGPSPoint().lat, getGPSPoint().lng, 6);
  LoRaFloatData('W', getAbsoluteWindDir());
  LoRaIntData('r', rudder.read());
  LoRaIntData('T', getHeadingTarget());
  LoRaFloatData('V', getVoltage());
  LoRaFloatData('S', GPSspeed);
}

#endif
