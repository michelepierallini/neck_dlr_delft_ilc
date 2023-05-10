#ifndef PYCOMMSLIB_H
#define PYCOMMSLIB_H

#define size_of_array(arr) sizeof(arr) / sizeof(*arr)

#include <Arduino.h>

String getValue(String data, char separator, int index);

void load_msg_to_python(String* msgName, String* msg, int numOfMsg);

String latest_received_msg(int index = -1);

void init_python_communication();

void sync();

#endif
