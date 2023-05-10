#include "pyCommsLib.h"
#include "HX711.h"

HX711 lc;

String msgName[] = {"lc"};
String dataCarrier[1];

uint8_t dataPin = 4;
uint8_t clockPin = 5;

volatile float f;

void setup() {
  Serial.begin(115200);

  lc.begin(dataPin, clockPin);
  lc.set_scale(1820); //3090 for lc on dynamixel
  //lc.tare();
  
  init_python_communication();
}

void loop() {
  f = lc.get_units()-84;//183 is the offset of the loadcell with no load
  f=(f/155)*9.8;
  dataCarrier[0] = String(f);
  load_msg_to_python(msgName, dataCarrier, size_of_array(msgName));
  sync();
}
