import sys
import time
import zmq
import json

sys.path.append('libraries/')
sys.path.append('config/')

from libraries.comms_wrapper import *
from libraries.dynamixel_controller import *
from config.config_get_torque_current import *
from utility import *

# Functions
def raw2tendon_force(value):
    return value / force_conversion * -1

# Begin ZMQ server
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:" + str(plotjuggler_config["port"]))

# get csv save directory
save_dir = define_csv_save_location(dataLog_relativeDirectory, exp_set_name)

# Connect dynamixel
print("... Connecting dynamixel \n\n")
servo = dynamixel(  ID=dynamixel_config["ID"], 
                    descriptive_device_name=dynamixel_config["name"], 
                    series_name=dynamixel_config["series"], 
                    baudrate=dynamixel_config["baudrate"], 
                    port_name=dynamixel_config["port"]  )

servo.begin_communication()
servo.set_operating_mode("position")

servo.write_position(1700)
time.sleep(1)

# Connect loadcell arduino
print("... Connecting arduino on the motor \n\n")
loadcell = Arduino( descriptiveDeviceName=loadcell_arduino_config["name"], 
                    portName=loadcell_arduino_config["port"], 
                    baudrate=loadcell_arduino_config["baudrate"])
loadcell.connect_and_handshake()

# Connect veficiation arduino
print("... Connecting arduino off the motor \n\n")
verification = Arduino( descriptiveDeviceName=verification_arduino_config["name"], 
                        portName=verification_arduino_config["port"], 
                        baudrate=verification_arduino_config["baudrate"])
verification.connect_and_handshake()

servo.set_operating_mode("velocity")
servo.write_velocity(-2)

timer = time.time()
while 1:
    verification.receive_message()
    if verification.newMsgReceived:
        load = -1*float(verification.receivedMessages["lc"])

        if load > 10:
            servo.write_velocity(0)
            break

        if time.time() - timer > 0.5:
            print(load)
            timer = time.time()

servo.set_operating_mode("current")


key = Key()

print("\n\n---- All devices connected ----\n\n")

input("Press enter to begin the measurement")

plotjuggler_payload = {}

time_data = []
motor_load_data = []
ground_truth_load_data = []
current_data = []

# Main loop
timer = time.time()
increase_current_timer = time.time()
current_input = 0

print("Press s to start the system")
print("Press q to stop the recording")
begin_measure = False
while 1: 
    
    if key.keyPress == "s" and begin_measure == False:
        timer = time.time()
        increase_current_timer = time.time()
        begin_measure = True

    if key.keyPress == "q":
        break
 
    if begin_measure and time.time() - increase_current_timer > 1:
        increase_current_timer = time.time()
        current_input += 1

    loadcell.receive_message()
    verification.receive_message()
    
    if verification.newMsgReceived:
        t = time.time() - timer
        raw_load = float(loadcell.receivedMessages["lc"])
        measured_load = raw2tendon_force(raw_load)
        ground_truth = -1*float(verification.receivedMessages["lc"])
        
        servo.write_current(current_input * -1)

        plotjuggler_payload["timestamp"] = t
        plotjuggler_payload["raw"] = raw_load
        plotjuggler_payload["measured"] = measured_load
        plotjuggler_payload["ground truth"] = ground_truth
        plotjuggler_payload["current input"] = current_input

        socket.send_string( json.dumps(plotjuggler_payload) )

        if begin_measure:
            print(current_input, ground_truth)

            time_data.append(t)
            motor_load_data.append(measured_load)
            ground_truth_load_data.append(ground_truth)
            current_data.append(current_input)

if time_data is not []:

    servo.write_current(0)
    servo.disable_torque()

    input("Press enter again")
    
    filename = obtain_csv_filename(save_dir)

    plot_and_save_data(plottingData= ([time_data, ground_truth_load_data], 
                                                [current_data, ground_truth_load_data, motor_load_data]),
                                xAxisLabel= ("Time", "Current Setpoint"), 
                                yAxisLabel =("Load (g)", "Load (g)"),
                                label = (["GT loadcell"], ["GT loadcell", "Motor loadcell"]), 
                                savingData= (time_data, motor_load_data, ground_truth_load_data, current_data), 
                                filename= filename,
                                saveDir= save_dir,
                                display_plot= True, 
                                saveData = True, 
                                figsize = (6,8))