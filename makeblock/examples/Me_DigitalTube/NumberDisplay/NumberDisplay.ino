#include "Makeblock.h"
#include <Wire.h>
#include <SoftwareSerial.h>

MeDigitalTube tube(PORT_6);
void setup() {
  tube.display(87.16);
}
void loop() {
}

