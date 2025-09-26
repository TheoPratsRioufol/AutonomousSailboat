
#include "Hardware.h"


TinyGPSPlus gps;
Servo rudder, sail;
MPU6050 mpu;
AS5600 windVane;
QMC5883LCompass compass;
sSoftSerial gps_serial(GPS_TX, GPS_RX);

/* I2C adresses :
compass 0x0D
accéléro 0x68
windvane 0x36
 */

GPSPoint lastUpdatedGPSPoint;
float GPSspeed;

void initHardware() {
  Wire.begin();
  Serial.println(F("SailboatOK"));
  pinMode(LED_PIN,OUTPUT);
  pinMode(KEY_PIN,INPUT);
  pinMode(A2,INPUT);
  pinMode(BATV_PIN, INPUT);
  /* Initialisation des objets */
  gps_serial.begin(GPS_BAUD);
  rudder.attach(DIR_PIN);
  sail.attach(SAIL_PIN);
  mpu.initialize();
  compass.init();
  compass.setADDR(0x0D);
  windVane.setDirection(AS5600_CLOCK_WISE);
  if (!windVane.isConnected())
    Serial.println(F("WindVaneFail"));
  compass.setSmoothing(5,true);
  //compass.setCalibrationOffsets(76.00, -358.00, -107.00);
  compass.setCalibrationOffsets(274.5, -487.00, -107.00);
  compass.setCalibrationScales(0.94, 1.12, 0.96);
  /* Initialisation des variables */
  GPSPoint lastUpdatedGPSPoint;
  lastUpdatedGPSPoint = {0, 0};
  GPSspeed = 0;
}

Attitude getAttitude() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  Attitude att;
  att.roll = -atan2(-ax, az);
  att.pitch = -atan2(-ay*cos(att.roll), az);
  return att;
}

float getWindVaneAngle() {
  return 360 * ((float) windVane.readAngle() - WIND_VANE_MIN_VALUE) / (WIND_VANE_MAX_VALUE - WIND_VANE_MIN_VALUE);
}

Compass getCompass() {
  compass.read();
  return {compass.getX(), compass.getY(), compass.getZ()};
}

void lisenGPS() {
  while ((gps_serial.available() > 0)) {
    if (gps.encode(gps_serial.read())) {
      lastUpdatedGPSPoint.lat = gps.location.lat();
      lastUpdatedGPSPoint.lng = gps.location.lng();
      GPSspeed = gps.speed.kmph();
   }
  }
}

void updateHardware() {
  lisenGPS();
}

float getVoltage() {
  return 7.7932*analogRead(BATV_PIN)/279.0;
}
