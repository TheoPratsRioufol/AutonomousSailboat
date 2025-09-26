
// Main code

#include "Abstraction.h"
#include "Navigation.h"
#include "Communication.h"
#include "Guidance.h"

void setup() {
  Serial.begin(9600);
  initAbstraction();
  initNavigation();
  initCommunication();
  initGuidance();
}

void loop() {

  /* Guidance loop */
  updateGuidance();

  /* Then, Abtraction loop */
  updateAbstraction();

  /* Communication loop */
  updateCommunication();

}
