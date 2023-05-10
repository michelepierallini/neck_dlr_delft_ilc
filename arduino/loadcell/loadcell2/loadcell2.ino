
#include "HX711.h"

HX711 lc;

String msgName[] = {"lc"};
String dataCarrier[1];

uint8_t dataPin = 5;
uint8_t clockPin = 6;

volatile float f;
volatile float a;
void setup() {
  Serial.begin(9600);

  lc.begin(dataPin, clockPin);
  lc.set_scale(500); //3090 for lc on dynamixel

  

}

void loop() {
  f = lc.get_units();//183 is the offset of the loadcell with no load
  f=f-220;
  Serial.println(f);
}
