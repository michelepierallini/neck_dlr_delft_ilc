## Close communication with dynamixel
import sys
import socket
# import json
import time
sys.path.append('libraries/')
sys.path.append('config/')
from libraries.comms_wrapper import *
from libraries.dynamixel_controller import *
from config.config_controller1 import *
from termcolor import colored 

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
print('\n')
## Connect dynamixel
servo = dynamixel(  ID=dynamixel_config["ID"], 
                    descriptive_device_name=dynamixel_config["name"], 
                    series_name=dynamixel_config["series"], 
                    baudrate=dynamixel_config["baudrate"], 
                    port_name=dynamixel_config["port"]  )
print('==================================================================================')
ID_structure = [1,2,3,4]
servo = dynamixel(ID_structure, descriptive_device_name="XM430 motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=1000000, port_name="COM5")
servo.begin_communication()
print('==================================================================================')
time.sleep(1)
servo.set_operating_mode("current", ID = "all")
key = Key()

## Deconnet dynamixel
print('==================================================================================')
servo.end_communication() 
print(colored('==================================================================================', 'red')) 
print(colored('ABORTING ... Motor comunication OFF', 'red'))
print(colored('==================================================================================', 'red')) 


