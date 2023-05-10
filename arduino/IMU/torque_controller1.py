# Simple PID controller

import sys
import time
import socket
import json
import math
sys.path.append('libraries/')
sys.path.append('config/')

from libraries.comms_wrapper import *
from libraries.dynamixel_controller import *
from config.config_controller1 import *

# Functions
def raw2tendon_force(value):
    return value / force_conversion * -1



# Begin UDP server
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

# Connect dynamixel
servo = dynamixel(  ID=dynamixel_config["ID"], 
                    descriptive_device_name=dynamixel_config["name"], 
                    series_name=dynamixel_config["series"], 
                    baudrate=dynamixel_config["baudrate"], 
                    port_name=dynamixel_config["port"]  )
ID_structure=[1,2,3,4]
servo = dynamixel(ID_structure, descriptive_device_name="XM430 motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=1000000, port_name="COM13")


servo.begin_communication()


#servo.write_position(2500)
time.sleep(1)

servo.set_operating_mode("current", ID = "all")


# Connect loadcell arduino
loadcell = Arduino( descriptiveDeviceName=loadcell_arduino_config["name"], 
                    portName=loadcell_arduino_config["port"], 
                    baudrate=loadcell_arduino_config["baudrate"])
loadcell_M3 = Arduino( descriptiveDeviceName=loadcell_arduino_M3["name"], 
                    portName=loadcell_arduino_M3["port"], 
                    baudrate=loadcell_arduino_M3["baudrate"])
loadcell_M1 = Arduino( descriptiveDeviceName=loadcell_arduino_M1["name"], 
                    portName=loadcell_arduino_M1["port"], 
                    baudrate=loadcell_arduino_M1["baudrate"])
loadcell_M2 = Arduino( descriptiveDeviceName=loadcell_arduino_M2["name"], 
                    portName=loadcell_arduino_M2["port"], 
                    baudrate=loadcell_arduino_M2["baudrate"])
loadcell.connect_and_handshake()
loadcell_M1.connect_and_handshake()
loadcell_M3.connect_and_handshake()
loadcell_M2.connect_and_handshake()

key = Key()

print("\n\n---- All devices connected ----\n\n")
input("press enter to begin motion")
plotjuggler_payload = {}

force_demand1 = +0.1
force_demand2 = +0.1
force_demand3 = +0.1
force_demand4 = +0.1

e_old = None
e_int = 0
t_old = 0



# Main loop
timer = time.time()
while 1: 

    #print(time.time() - timer, loadcell._rawReceivedMessage)
    loadcell.receive_message()
    loadcell_M3.receive_message()
    loadcell_M1.receive_message()
    loadcell_M2.receive_message()
    if loadcell.newMsgReceived:
        kp = 0.5#2
        kd = 0.5
        ki = 0
        t = time.time() - timer
        raw_load = float(loadcell.receivedMessages["lc"])
        raw_load_M3 = float(loadcell_M3.receivedMessages["lc"])
        raw_load_M1 = float(loadcell_M1.receivedMessages["lc"])
        raw_load_M2 = float(loadcell_M2.receivedMessages["lc"])
        measured_load_4 = raw_load/9.8
        measured_load_1 = raw_load_M1/9.8
        measured_load_2 = raw_load_M2/9.8
        measured_load_3 = raw_load_M3/9.8

        
        print(time.time() - timer,measured_load_1,measured_load_2,measured_load_3, measured_load_4)
        dt = t-t_old
        
        e1 = force_demand1 - measured_load_1
        e2 = force_demand2 - measured_load_2
        e3 = force_demand3 - measured_load_3
        e4 = force_demand4 - measured_load_4

        #if e_old is None:
        #    e_dot = 0
        #else:
        #    e_dot = (e4 - e_old)/dt

        #e_int += e4 * dt

        # Reset integral term if zero crossing
        #if e_old is not None:
        #    if (e4 > 0 and e_old <= 0) or (e4 < 0 and e_old >= 0):
        #        e_int = 0
        u1 = e1 * kp
        u2 = e2 * kp
        u3 = e3 * kp
        u4 = e4 * kp #+ e_dot * kd #+ e_int * ki)

        currentstep_Nratio=4096/(1.2/0.01)#n_steps/(Max_torque/Pulley_radius)
        current_input4 = u4*currentstep_Nratio  #* -1
        current_input1 = u1*currentstep_Nratio
        current_input2 = u2*currentstep_Nratio
        current_input3 = u3*currentstep_Nratio
        t_old = t
        e_old = e4

        servo.write_current(current_input4,ID=4)
        servo.write_current(current_input3,ID=3)
        servo.write_current(current_input2,ID=2)
        servo.write_current(current_input1,ID=1)

        force_demand3=(math.sin(t/3)+1)*2
        force_demand4=(math.sin(t/3)+1)*2
   
       



