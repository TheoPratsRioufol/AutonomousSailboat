
#include "Communication.h"

//#define SIMULATION

unsigned long sendTimer, lastComTime, rebootTimer;
int numSend;

void initCommunication() {
  if (!LoRa.begin(FREQ_LORA)) {
    Serial.println(F("LoRaFail"));
  }
  /* Initialize variables */
  sendTimer = millis();
  lastComTime = millis();
  rebootTimer = millis();
  numSend = 0;
  /* Initialize parameters */
  REBOOT_LORA_PERIOD = 30;
  #ifndef SIMULATION
  SEND_INTERVAL = 2000;
  #else
  SEND_INTERVAL = 300;
  #endif
}

void sendLoRa(String msg) {
  #ifndef SIMULATION
  if (LoRa.beginPacket() == 0)
      return;
  LoRa.print(msg);
  LoRa.endPacket(true);
  #else
  Serial.print("-0");
  Serial.print(LORA_SEPARATOR);
  Serial.println(msg);
  #endif
}

void LoRaIntData(char c, int data) {
  #ifndef SIMULATION
  LoRa.print(LORA_SEPARATOR);
  LoRa.print(c);
  LoRa.print(data);
  #else
  Serial.print(LORA_SEPARATOR);
  Serial.print(c);
  Serial.print(data);
  #endif
}

void LoRaFloatData(char c, float data, byte acc=2) {
  #ifndef SIMULATION
  LoRa.print(LORA_SEPARATOR);
  LoRa.print(c);
  LoRa.print(String(data,acc));
  #else
  Serial.print(LORA_SEPARATOR);
  Serial.print(c);
  Serial.print(String(data,acc));
  #endif
}

void LoRa2FloatData(char c, float float1, float float2, byte acc=2) {
  #ifndef SIMULATION
  LoRa.print(LORA_SEPARATOR);
  LoRa.print(c);
  LoRa.print(String(float1,acc));
  LoRa.print(',');
  LoRa.print(String(float2,acc));
  #else
  Serial.print(LORA_SEPARATOR);
  Serial.print(c);
  Serial.print(String(float1,acc));
  Serial.print(',');
  Serial.print(String(float2,acc));
  #endif
}

void LoRaStr(char c, String msg) {
  #ifndef SIMULATION
  LoRa.print(LORA_SEPARATOR);
  LoRa.print(c);
  LoRa.print(msg);
  #else
  Serial.print(LORA_SEPARATOR);
  Serial.print(c);
  Serial.print(msg);
  #endif
}

void sendData() {
  numSend++;
  #ifndef SIMULATION
  if (LoRa.beginPacket() == 0)
      return;
  LoRa.print('@');
  #else
  Serial.print("-0");
  Serial.print(LORA_SEPARATOR);
  Serial.print("@");
  #endif
  shareAbstractionStatus();
  shareGuidanceStatus();
  shareNavigationStatus();
  LoRaIntData('P', numSend);
  #ifndef SIMULATION
  LoRa.endPacket(true);
  #else
  Serial.println();
  #endif
}

void sendParameters() {
  // Send all parameters values
  #ifndef SIMULATION
  if (LoRa.beginPacket() == 0)
      return;
  LoRa.print("I");
  for (int i = 0; i < SIZE_INT_VARIBLES; i++) {
     LoRa.print(String(intVariables[i]));
     LoRa.print(LORA_SEPARATOR);
  }
  LoRa.print("F");
  for (int i = 0; i < SIZE_FLOAT_VARIBLES-1; i++) {
     LoRa.print(String(floatVariables[i]));
     LoRa.print(LORA_SEPARATOR);
  }
  LoRa.print(String(floatVariables[SIZE_FLOAT_VARIBLES-1]));
  LoRa.endPacket(true);
  #else
  Serial.print("-0");
  Serial.print(LORA_SEPARATOR);
  Serial.print("I");
  for (int i = 0; i < SIZE_INT_VARIBLES; i++) {
     Serial.print(String(intVariables[i]));
     Serial.print(LORA_SEPARATOR);
  }
  Serial.print("F");
  for (int i = 0; i < SIZE_FLOAT_VARIBLES-1; i++) {
     Serial.print(String(floatVariables[i]));
     Serial.print(LORA_SEPARATOR);
  }
  Serial.println(String(floatVariables[SIZE_FLOAT_VARIBLES-1]));
  #endif
}

boolean executeCmd(String msg) {
  byte idx, idx2, wpnum, wptonum;

  if (msg.substring(0,16) == F("SET_RECOVERY_GPS")) {
    GPSPoint newRecoGPSPoint = getGPSPoint();
    setRecoveryGPSPoint(newRecoGPSPoint);
    EEPROM.put(ADR_EEPROM_RECOVERY_GPSPOINT, newRecoGPSPoint);
    if (LoRa.beginPacket() != 0) {
      LoRa.print(F("New recovery GPS: "));
      LoRa.print(String(newRecoGPSPoint.lat,6));
      LoRa.print(',');
      LoRa.print(String(newRecoGPSPoint.lng,6));
      LoRa.endPacket(true);
    }
    return false;
  }
  
  switch (msg[0]) {
    case 'W': /* Add WayPoint */
    {
      idx = msg.indexOf('-');
      wpnum = msg.substring(1,idx).toInt();
      idx2 = msg.indexOf('-', idx+1);
      wptonum = msg.substring(idx+1,idx2).toInt();
      idx = msg.indexOf(',');
      GPSPoint pt = {msg.substring(idx2+1, idx).toFloat(), msg.substring(idx+1).toFloat()};
      addWaypoint(wpnum, wptonum, pt);
      return true;
    }

    case 'I': /* Modify Int variable */
    {
      idx = msg.indexOf(LORA_SEPARATOR);
      intVariables[msg.substring(1,idx).toInt()] = msg.substring(idx+1,msg.length()).toInt();
      return true;
    }

    case 'F': /* Modify Float variable */
    {
      idx = msg.indexOf(LORA_SEPARATOR);
      floatVariables[msg.substring(1,idx).toInt()] = msg.substring(idx+1,msg.length()).toFloat();
      return true;
    }

    case 'P': /* Send parameters */
    {
      sendParameters();
      return false;
    }

    case 'D': /* Rudder overwrite */
    {
      setRudder(msg.substring(1).toInt());
      return true;
    }

    case 'S': /* sail overwrite */
    {
      setSail(msg.substring(1).toInt());
      return true;
    }

    case 'R': /* Software Reset */
    {
      sendLoRa(F("Reseting.."));
      initAbstraction();
      initNavigation();
      initCommunication();
      initGuidance();
      return true;
    }

    case 'J': /* Stop Jibe of Tack */
    {
      sendLoRa(F("Stop jibe/Tack"));
      finishManoeuvre();
      return false;
    }

    case 'A': /* Change amure */
    {
      changeAmure();
      sendLoRa(F("Changing Amure"));
      return false;
    }
    
    default: /* ping */
    {
      sendLoRa(msg);
      return false;
    }
  }
  return true;
}

void executeCmds(String msg) {
  int nindex = msg.indexOf('\n');
  int newNindex;
  boolean sendOK = false;
  if (nindex == -1) {
    if (executeCmd(msg))
        sendLoRa("ok");
    return;
  }
  sendOK = executeCmd(msg.substring(0, nindex));
  boolean uu;
  while(nindex != -1) {
    newNindex = msg.indexOf('\n', nindex+1);
    if (newNindex == -1) {
      uu = executeCmd(msg.substring(nindex+1));
      sendOK = sendOK || uu;
    } else {
      uu = executeCmd(msg.substring(nindex+1));
      sendOK = sendOK || executeCmd(msg.substring(nindex+1,newNindex));
    }
    nindex = newNindex;
  }
  if (sendOK)
      sendLoRa("ok");
}

void updateCommunication() {
  #ifndef SIMULATION
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg = "";
    while (LoRa.available()) {
      msg += (char)LoRa.read();
    }
    lastComTime = millis();
    ledOn(LED_BLINK_PERIOD);
    executeCmds(msg);
  }
  #else
  if (Serial.available()) {
    lastComTime = millis();
    String str = Serial.readString();
    str.trim();
    executeCmds(str);
  }    
  #endif

  if (((millis() - lastComTime)/1000 > REBOOT_LORA_PERIOD) && (millis() > rebootTimer)) {
    /* lost connection, try to reboot lora */
    LoRa.end();
    LoRa.begin(FREQ_LORA);
    rebootTimer = millis() + 1000*REBOOT_LORA_PERIOD;
    sendLoRa(F("LoRa Rebooted"));
  }

  if (millis() < sendTimer) {
    return;
  }

  sendTimer = millis() + max(SEND_INTERVAL, MINIMAL_SEND_INTERVAL);

  sendData();
  ledOn(LED_SEND_BLINK_PERIOD);
}

unsigned long getLastComTime() {
  return lastComTime;
}
