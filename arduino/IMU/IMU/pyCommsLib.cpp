#include "pyCommsLib.h"
#include "Arduino.h"


#define size_of_array(arr) sizeof(arr) / sizeof(*arr)


String rawMsgFromPython = "NO_PYTHON_MESSAGE";
String payload = "";


String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void load_msg_to_python(String* msgName, String* msg, int numOfMsg) {
  // If we have the same number of data compared to the message
  payload = "";
  
  for (int i = 0; i < numOfMsg; i++) {
    payload.concat(msgName[i]);
    payload.concat(";");
    payload.concat(msg[i]);
    payload.concat(":");
  }
}

void receive_msg_from_python() {

  String msg = "";

  while (Serial.available()) {
    if (Serial.available()) {
      msg = Serial.readStringUntil('\n');
    }
  }

  if (msg != "") {
    rawMsgFromPython = msg;
  }

}


String latest_received_msg(int index = -1) {
  if(index == -1) { 
    return rawMsgFromPython;
  }
  else {
    return getValue(rawMsgFromPython, '@', index);
  }
}

void init_python_communication() {

  while (true) {
    // if the python side sent a message
    if (Serial.available() > 0) {
      String rawMsgFromPython = Serial.readStringUntil('\n');
      if (rawMsgFromPython == "handshake1") {
        break;
      }
    }
  }

  long timer = millis();
  while (millis() - timer < 1000) {
    Serial.println("handshake2");
  }


  while (Serial.available()) {
    Serial.read();
  }

  sync();
}


void sync() { 
  receive_msg_from_python();
  String final_payload = "<echo*;";
  final_payload.concat(latest_received_msg());
  final_payload.concat(":");
  final_payload.concat(payload); 
  final_payload.concat(">");
  Serial.println(final_payload);
}
