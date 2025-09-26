
// To communicate with the ground

#include "Arduino.h"
#include "Abstraction.h"
#include "Navigation.h"
#include "Guidance.h"
#include "Parameters.h"
#include <LoRa.h>

#define FREQ_LORA 446.01875E6
#define LED_BLINK_PERIOD 500
#define LED_SEND_BLINK_PERIOD 100
#define LORA_SEPARATOR '>'
#define SEND_INTERVAL intVariables[13]
#define REBOOT_LORA_PERIOD intVariables[29]

#ifndef SIMULATION
#define MINIMAL_SEND_INTERVAL 1200
#else
#define MINIMAL_SEND_INTERVAL 0
#endif

/* fonction d'initailaisation/reset */

void initCommunication();

/* Pour faire fonctionner la communication */

void updateCommunication();

void LoRaFloatData(char c, float data, byte acc=2);

void LoRa2FloatData(char c, float float1, float float2, byte acc=2);

void LoRaIntData(char c, int data);

void LoRaStr(char c, String msg);

unsigned long getLastComTime();
