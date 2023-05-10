# import contextlib
import sys
import time
import socket
import numpy as np
# from LILC import LILC
# from PCC_class import DataRobot
# from ILC_SoftRobot import ILC_SoftRobot
import CONST as C
import csv
# import matplotlib.pyplot as plt
from termcolor import colored
sys.path.append('libraries/')
sys.path.append('config/')
from utility import*
from libraries.comms_wrapper import *
from libraries.dynamixel_controller import *
from config.config_controller1 import *
from utils_data import *

######################################################################################################################################################################
######################################################################################################################################################################
#### JUST STREAMING SOME DATA AMD SAVING THEM.
######################################################################################################################################################################
######################################################################################################################################################################

print('\n')
## select 0 in gain_check if you want just to move the robot and record the data
gain_check = 1 # 0
gain_check_tre = 0 # 1
directory = os.getcwd() + '/Test_001_STREAMING_DATA_04_05_2022'
get_folder_plot(directory)

# # Begin UDP server
sock = socket.socket(socket.AF_INET,     # Internet
                     socket.SOCK_DGRAM)  # UDP

# Connect dynamixe
servo = dynamixel(ID = dynamixel_config["ID"],  
                  descriptive_device_name = dynamixel_config["name"],
                  series_name = dynamixel_config["series"],
                  baudrate = dynamixel_config["baudrate"],
                  port_name = dynamixel_config["port"])
print('========================================================================================')

ID_structure = [1, 2, 3, 4]
servo = dynamixel(ID_structure,
                  descriptive_device_name = "XM430 motor",
                  series_name = ["xm", "xm", "xm", "xm"],
                  baudrate = 1000000,
                  port_name = "COM5") 
t_old = 0

## write data into a file.csv
fileHeader = ["time", \
        "measured_load_1", "measured_load_2", "measured_load_3", "measured_load_4", \
        "IMU_x", "IMU_y", "IMU_z", "IMU_gx", "IMU_gy", "IMU_gz",\
        "IMU_rad_x", "IMU_rad_y", "IMU_rad_z", \
        "current_input1", "current_input2", "current_input3", "current_input4",\
        "Delta_x", "Delta_y", "dL",  \
        "phi", "theta", "dL", \
        "dt_exp_vett"]
    
csvFile = open( directory + "/dataset_stream" + ".csv", "w")
writer = csv.writer(csvFile)
writer.writerow(fileHeader)

servo.begin_communication()
print('========================================================================================')
time.sleep(1)
servo.set_operating_mode("current", ID = "all")
# Connect loadcell arduino
    
loadcell = Arduino(descriptiveDeviceName = loadcell_arduino_config["name"],
                    portName = loadcell_arduino_config["port"],
                    baudrate = loadcell_arduino_config["baudrate"])

loadcell_M3 = Arduino(descriptiveDeviceName = loadcell_arduino_M3["name"],
                        portName = loadcell_arduino_M3["port"],
                        baudrate = loadcell_arduino_M3["baudrate"])

loadcell_M1 = Arduino(descriptiveDeviceName = loadcell_arduino_M1["name"],
                        portName = loadcell_arduino_M1["port"],
                        baudrate = loadcell_arduino_M1["baudrate"])

loadcell_M2 = Arduino(descriptiveDeviceName = loadcell_arduino_M2["name"],
                        portName = loadcell_arduino_M2["port"],
                        baudrate = loadcell_arduino_M2["baudrate"])

arduino_IMU = Arduino(descriptiveDeviceName = arduino_IMU["name"],
                        portName = arduino_IMU["port"],
                        baudrate = arduino_IMU["baudrate"])
    
print('========================================================================================')
loadcell.connect_and_handshake()
loadcell_M1.connect_and_handshake()
loadcell_M3.connect_and_handshake()
loadcell_M2.connect_and_handshake()
arduino_IMU.connect_and_handshake()
print('========================================================================================')

key = Key()
print('========================================================================================')
print("\t\tAll devices connected")
input("\t\tPress ENTER to begin motion ")
print('========================================================================================')

# reference force (torque)
# offset to tire each tendon which will be a reference at the beginning of the exp
# 0.1 is the minimum value you can set to check how much they are loose
force_demand1 = C.force_demand
force_demand2 = C.force_demand
force_demand3 = C.force_demand # (-) # check this afterwards
force_demand4 = C.force_demand
    
timer = time.time() # init time
conta_time = 0
for j in range(0, 2 * C.max_step): # time
        
    try: 
        loadcell.receive_message()
        loadcell_M3.receive_message()
        loadcell_M1.receive_message()
        loadcell_M2.receive_message()
        arduino_IMU.receive_message()
        IMU = [0, 0, 0]
        IMU_gyro = [0, 0, 0]
        IMU_rad = [0, 0, 0]

        if loadcell.newMsgReceived:
            ### oppure fare un else qua che mi carica nelle variabili quando perdo dei campioni.
            conta_time = conta_time + 1
            ## getting data from Arduino
            t = time.time() - timer
            # probably but we are not sure they are expressed in [Nm]
            raw_load = float(loadcell.receivedMessages["lc"])
            raw_load_M3 = float(loadcell_M3.receivedMessages["lc"])
            raw_load_M1 = float(loadcell_M1.receivedMessages["lc"])
            raw_load_M2 = float(loadcell_M2.receivedMessages["lc"])
            ## Newton values onto the tendons force
            measured_load_4 = raw_load / 9.8       # [N]
            measured_load_1 = raw_load_M1 / 9.8    # [N]
            measured_load_2 = raw_load_M2 / 9.8    # [N]
            measured_load_3 = raw_load_M3 / 9.8    # [N]

            IMU[0] = float(arduino_IMU.receivedMessages["x"])
            IMU[1] = float(arduino_IMU.receivedMessages["y"])
            IMU[2] = float(arduino_IMU.receivedMessages["z"])

            IMU_gyro[0] = float(arduino_IMU.receivedMessages["gx"])
            IMU_gyro[1] = float(arduino_IMU.receivedMessages["gy"])
            IMU_gyro[2] = float(arduino_IMU.receivedMessages["gz"])
                
            dt_exp_vett = t - t_old # ~10 Hz

            ## Force error [N]. THIS IS A FEEDBACK TO SET THE PRE-TENSION ON THE TENDONS
            e1 = force_demand1 - measured_load_1
            e2 = force_demand2 - measured_load_2
            e3 = force_demand3 - measured_load_3
            e4 = force_demand4 - measured_load_4

            ## THIS IS THE ACTUAL TASK TO BE PERFORMD, HERE THE PID CONTROLLER IS IMPLEMENTED 
            ## Note that ek is a feedback on the compression while u_old is a feedforward
            ## [N] force. Then, we will covnert into current
            u1 = e1 * C.kp
            u2 = e2 * C.kp 
            u3 = e3 * C.kp * gain_check_tre
            u4 = e4 * C.kp   

            ## actual control action current&torque conversion here 
            currentstep_Nratio = 4096 / (1.2 / 0.01)  # n_steps/(Max_torque/Pulley_radius)
            current_input4 = u4 * currentstep_Nratio * gain_check 
            current_input1 = u1 * currentstep_Nratio * gain_check
            current_input2 = u2 * currentstep_Nratio * gain_check
            current_input3 = u3 * currentstep_Nratio * gain_check
            t_old = t
                
            IMU_rad = convertIMUStep(IMU)
            q_pcc, q_delta = ypr2pcc(IMU_rad, C.l0)
            q = np.array(q_delta).reshape((-1,))

            if (C.WANNA_PRINT and j%C.TIME2PRINT == 0):
                print('[INFO]:\n\tTime: {} [sec] Step: {} dt: {}\n\tF Tendon [ {}, {}, {}, {}]'\
                        .format(np.round(t,2), j, np.round(dt_exp_vett,3), \
                        np.round(measured_load_1,2), np.round(measured_load_2,3), np.round(measured_load_3,2), np.round(measured_load_4,2)))
                print('\tIMU LINE: [{}, {}, {}]\n\tIMU GYRO: [{}, {}, {}]'\
                        .format(np.round(IMU[0],2),np.round(IMU[1],2),np.round(IMU[2],2),\
                        np.round(IMU_gyro[0],2),np.round(IMU_gyro[1],2),np.round(IMU_gyro[2],2)))
                print('\tIMU rad: [{}, {}, {}]'\
                        .format(np.round(IMU_rad[0],2),np.round(IMU_rad[1],2),np.round(IMU_rad[2],2)))
                print('\tErr Load: [{}, {}, {}, {}]'\
                    .format(np.round(e1,2),np.round(e2,2),np.round(e3,2), np.round(e4, 2)))
                print('\tq: [{}, {}, {}]'\
                        .format(np.round(q[0],2),np.round(q[1],2),np.round(q[2],2)))
                print('\tq PCC: [{}, {}, {}]'\
                        .format(np.round(q_pcc[0],2),np.round(q_pcc[1],2),np.round(q_pcc[2],2)))
                print('========================================================================================')
                print('\tCurrent input: [{}, {}, {}, {}]'\
                        .format(np.round(current_input1,2),np.round(current_input2,2),\
                        np.round(current_input3,2), np.round(current_input4,2)))
                print('========================================================================================')
            else:
                pass
        
            ## sending the control to the robot 
            servo.write_current(current_input4,ID=4)
            servo.write_current(current_input3,ID=3) 
            servo.write_current(current_input2,ID=2)
            servo.write_current(current_input1,ID=1)
                
            data = [time.time() - timer, \
                measured_load_1, measured_load_2, measured_load_3, measured_load_4, \
                IMU[0], IMU[1], IMU[2], \
                IMU_gyro[0],IMU_gyro[1], IMU_gyro[2], \
                IMU_rad[0], IMU_rad[1], IMU_rad[2],\
                current_input1, current_input2, current_input3, current_input4,\
                q[0], q[1], q[2], \
                q_pcc[0], q_pcc[1], q_pcc[2]]
            writer.writerow(data)
                
    except KeyboardInterrupt:
        print('==================================================================================') 
        print(colored('ABORTING Manually ... Motor comunication OFF', 'red'))
        servo.end_communication()
        # del loadcell, loadcell_M1, loadcell_M2, loadcell_M3, arduino_IMU
        print('==================================================================================')
        print('\n')
        raise SystemExit
    ############################################################# END COMUNICATION FOR ONE ITERATION
############################################################# END TIME FOR ONE ITERATION 
servo.end_communication()    
csvFile.close()