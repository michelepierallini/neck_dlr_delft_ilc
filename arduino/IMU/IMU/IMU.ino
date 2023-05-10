#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "pyCommsLib.h"
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);
String dataCarrier[6];
String msgName[] = {"x","y","z","gx","gy","gz"};
void setup(void) 
{
  Serial.begin(115200);
  //Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
  init_python_communication();
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  dataCarrier[0] = String(event.orientation.x);
  dataCarrier[1] = String(event.orientation.y);
  dataCarrier[2] = String(event.orientation.z);
  dataCarrier[3] = String(event.gyro.x);
  dataCarrier[4] = String(event.gyro.y);
  dataCarrier[5] = String(event.gyro.z);
  load_msg_to_python(msgName, dataCarrier, size_of_array(msgName));
  /* Display the floating point data */
  sync();
 // Serial.print(event.orientation.x, 4);
 //  Serial.print(",");
 //  Serial.print(event.orientation.y, 4);
 //  Serial.print(",");
 //  Serial.print(event.orientation.z, 4);
 //  Serial.println("");
  
 // delay(100);
}
