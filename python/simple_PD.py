# import contextlib
import sys
import time
import socket
import numpy as np
from PCC_class import DataRobot
from ILC_SoftRobot import ILC_SoftRobot
import CONST as C
import csv
import matplotlib.pyplot as plt
import math
plt.rcParams['text.latex.preamble'] = ''.join([r'\usepackage{siunitx}', r'\usepackage{amsmath}'])
from termcolor import colored
sys.path.append('libraries/')
sys.path.append('config/')
from utility import*
from libraries.comms_wrapper import *
from libraries.dynamixel_controller import *
from config.config_controller1 import *
from utils_data import *
from scipy.signal import savgol_filter

######################################################################################################################################################################
######################################################################################################################################################################
############ NOTE THAT ALL DATA HAS TO BE (n_time x dim)
############ THIS CODE HAS NOT (!!!) BEEN TESTED 
######################################################################################################################################################################
######################################################################################################################################################################

print('\n')
# print(colored('========================================================================================', 'yellow'))
# print(colored("\t\tHave you changed the path to store data?", 'yellow'))
# # input(colored("\t\tIf YES, Press <ENTER>", 'yellow'))
print(colored('========================================================================================', 'yellow'))

if C.WANNA_STAY_STILL:
    gain_check = 0.0
else:
    gain_check = 1.0

directory = os.getcwd() + '/Test_EXP_001_04_05_2023_' + 'PD'
if os.path.isdir(directory):
    directory = directory + '_ANOTHERONE_DJKHALED_' +  str(int(np.floor(np.random.rand(1) * 100)))
get_folder_plot(directory)

print('[INFO]:\tData will saved in:\n{}'.format(directory))
print(colored('========================================================================================', 'yellow'))


## Begin UDP server
sock = socket.socket(socket.AF_INET,     # Internet
                    socket.SOCK_DGRAM)   # UDP

# Connect dynamixel
servo = dynamixel(ID = dynamixel_config["ID"],
                    descriptive_device_name = dynamixel_config["name"],
                    series_name = dynamixel_config["series"],
                    baudrate = dynamixel_config["baudrate"],
                    port_name = dynamixel_config["port"])

ID_structure = [1, 2, 3, 4]
servo = dynamixel(ID_structure,
                    descriptive_device_name = "XM430 motor",
                    series_name = ["xm", "xm", "xm", "xm"],
                    baudrate = 1000000,
                    port_name = "COM5")

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

loadcell.connect_and_handshake()
loadcell_M1.connect_and_handshake()
loadcell_M3.connect_and_handshake()
loadcell_M2.connect_and_handshake()
arduino_IMU.connect_and_handshake()
print('========================================================================================')
key = Key()
print('========================================================================================')

## initial condition
q0 = np.array([-0.07, -0.03, 0])
q_dot0 = np.zeros(3)
q_ddot0 = np.zeros(3)
u0 = np.zeros(C.n_actuator)
## desired final position
target_pcc = np.array([-0.3, 0.1, -0.002])

print('[INFO]\tINIT :{}\tFINAL :{}'.format(np.round(q0, 3), np.round(target_pcc, 3)))
print('========================================================================================')

q_des = np.zeros((C.max_step, C.n_state)) 
q_dot_des = np.zeros((C.max_step, C.n_state))
q_ddot_des = np.zeros((C.max_step, C.n_state))

## desired trajactrory to track 
for j in range(0, C.n_state):
    q_des[:,j], q_dot_des[:,j], q_ddot_des[:,j] = mjRef(q0[j],
                                                        target_pcc[j],
                                                        C.t_f,
                                                        C.max_step)

    if C.WANNA_PLOT:
        plt.figure(clear=True)
        plt.xlabel(r'Time $[s]$')
        plt.ylabel(r'$q_{des}$')
        for j in range(0, C.n_state):
            plt.plot(np.linspace(0, C.max_step * C.dt, C.max_step), q_des[:,j], linestyle='-.', label= r'$q_{des}$' + str(j))
        plt.grid()
        plt.legend(loc='best', shadow = True, fontsize=C.font_size)
        plt.show()

        plt.figure(clear=True)
        plt.xlabel(r'Time $[s]$')
        plt.ylabel(r'$\dot{q}_{des}$')
        for j in range(0, C.n_state):
            plt.plot(np.linspace(0, C.max_step * C.dt, C.max_step), q_dot_des[:,j], linestyle='-.', label= r'$\dot{q}_{des}$' + str(j))
        plt.grid()
        plt.legend(loc='best', shadow = True, fontsize=C.font_size)
        plt.show()

        plt.figure(clear=True)
        plt.xlabel(r'Time $[s]$')
        plt.ylabel(r'$\ddot{q}_{des}$')
        for j in range(0, C.n_state):
            plt.plot(np.linspace(0, C.max_step * C.dt, C.max_step), q_ddot_des[:,j], linestyle='-.', label= r'$\ddot{q}_{des}$' + str(j))
        plt.grid()
        plt.legend(loc='best', shadow = True, fontsize=C.font_size)
        plt.show()
    else:
        pass

new_row_q = np.array([q_des[0, :]] * C.max_step)
q_des = np.concatenate((new_row_q, q_des))
new_row_q_dot = np.array([q_dot_des[0, :]] * C.max_step)
q_dot_des = np.concatenate((new_row_q_dot, q_dot_des))
new_row_q_ddot = np.array([q_ddot_des[0, :]] * C.max_step)
q_ddot_des = np.concatenate((new_row_q_ddot, q_ddot_des))

Q_DES = directory + '/' + 'q_des.csv'
np.savetxt(Q_DES, q_des, delimiter=',')
Q_DOT_DES = directory + '/' + 'q_dot_des.csv'
np.savetxt(Q_DOT_DES, q_dot_des, delimiter=',')
U_0 = directory + '/' + 'u0.csv'
np.savetxt(U_0, u0, delimiter=',')

t_old = 0
err_RMS = np.zeros(C.max_step)
for i in range(0, C.max_iter):

    ## write data into a file.csv
    ## this is very time-consuming
    fileHeader = ["time", \
        "measured_load_1", "measured_load_2", "measured_load_3", "measured_load_4", \
        "current_input1", "current_input2", "current_input3", "current_input4",\
        "IMU_x", "IMU_y", "IMU_z", "IMU_gx", "IMU_gy", "IMU_gz",\
        "IMU_rad_x", "IMU_rad_y", "IMU_rad_z", \
        "Delta_x", "Delta_y", "dL",  \
        "phi", "theta", "dL", \
        "phi_des", "theta_des", "dL_des", 
        "u1", "u2", "u3", "u4"]

    csvFile = open( directory + "/dataset_" + str(i) + ".csv", "w")
    writer = csv.writer(csvFile)
    writer.writerow(fileHeader)

    print(colored('========================================================================================', 'yellow'))
    print(colored("\t\tArduino connected", 'yellow'))
    input(colored("\t\tPress <ENTER> to begin motion", 'yellow'))
    print(colored('========================================================================================', 'yellow'))
    servo.begin_communication()
    print('========================================================================================')
    time.sleep(1)
    servo.set_operating_mode("current", ID = "all")
    print('========================================================================================')

    ## offset to tire each tendon which will be a reference at the beginning of the exp
    ## 0.1 is the minimum value you can set to check how much they are loose
    force_demand1 = C.force_demand
    force_demand2 = C.force_demand
    force_demand3 = C.force_demand
    force_demand4 = C.force_demand

    ## data2save
    imu_lin_vett, imu_gyro_vett, imu_rad_vett = [], [], []
    current_input_vett = [] # this is the control action actual sended to the robot, which includes the precompression action
    ## and the conversion into current.
    time_vett, q_vett, q_dot_vett, q_ddot_vett, dt_exp_vett, e_vett, q_des_vett, q_dot_des_vett = [], [], [], [], [], [], [], []
    u_sent_vett = [] # this is the ilc control within iteration the one that needs to be updated.
    ## underactuation matrix
    A = np.array([[1/2, -1/2, 1/4], [-1/2, 1/2, 1/4], [-1/2, -1/2, 1/4]])
    A_inv = np.linalg.pinv(A)
    ## proportional and derivatives gains for PD control
    K_P = 1
    K_V = 0.1
    robot = DataRobot(q0,
                    q_dot0,
                    q_ddot0,
                    u0,
                    underactuation_matrix=np.eye(C.n_state))

    u_new = np.zeros((2 * C.max_step, C.n_actuator))
    error_ilc_pos = np.zeros((2 * C.max_step, C.n_output))
    C.PRINT_ONE_ILC = True

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

            IMU_rad = convertIMUStep(IMU)
            q_pcc, q_delta = ypr2pcc(IMU_rad, C.l0)
            q = np.array(q_delta).reshape((-1,))

            if loadcell.newMsgReceived:
                conta_time = conta_time + 1
                # getting data from Arduino
                t = time.time() - timer
                # probably but we are not sure they are expressed in [Nm]
                raw_load = float(loadcell.receivedMessages["lc"])
                raw_load_M3 = float(loadcell_M3.receivedMessages["lc"])
                raw_load_M1 = float(loadcell_M1.receivedMessages["lc"])
                raw_load_M2 = float(loadcell_M2.receivedMessages["lc"])
                # Newton values onto the tendons force
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

                dt_exp_vett.append(t - t_old) # ~10 Hz

                ## Force error [N]. THIS IS A FEEDBACK TO SET THE PRE-TENSION ON THE TENDONS
                e1 = force_demand1 - measured_load_1
                e2 = force_demand2 - measured_load_2
                e3 = force_demand3 - measured_load_3
                e4 = force_demand4 - measured_load_4

                if j >= C.max_step:
                    k_p = C.kp_ilc
                else:
                    k_p = C.kp
                    u1_pre = 0
                    u2_pre = 0
                    u3_pre = 0
                    u4_pre = 0
                ## adding the pre-tension terms to the overall control
                if j == C.max_step:
                    u1_pre = e1 * k_p
                    u2_pre = e2 * k_p
                    u3_pre = e3 * k_p
                    u4_pre = e4 * k_p

                q_old = q_pcc 
                                   
                IMU_rad = convertIMUStep(IMU)
                q_pcc, q_delta = ypr2pcc(IMU_rad, C.l0)
                q = np.array(q_delta).reshape((-1,))
                ## this may leads to some error in the code
                q_vel_pcc = (q_pcc - q_old) / (t - t_old)

                ## REAL PD CONTROL 
                err_phi = q_des[j, 0] - q_pcc[0] 
                err_theta = q_des[j, 1] - q_pcc[1] 
                err_deltaL = q_des[j, 2] - q_pcc[2] 
                err_pcc = np.array([err_phi, err_theta, err_deltaL])

                ## REAL PD CONTROL 
                err_phi_dot = q_dot_des[j, 0] - q_vel_pcc[0] 
                err_theta_dot = q_dot_des[j, 1] - q_vel_pcc[1] 
                err_deltaL_dot = q_dot_des[j, 2] - q_vel_pcc[2] 
                err_pcc_dot = np.array([err_phi_dot, err_theta_dot, err_deltaL_dot])

                action_pos = np.dot(A_inv, err_pcc)
                action_vel = np.dot(A_inv, err_pcc_dot)

                u1 = e1 * k_p + u1_pre + K_P * action_pos[0] + K_V * action_vel[0]
                u2 = e2 * k_p + u2_pre + K_P * action_pos[1] + K_V * action_vel[1]
                u3 = e3 * k_p + u3_pre + K_P * action_pos[2] + K_V * action_vel[2]
                u4 = e4 * k_p + u4_pre + K_P * action_pos[3] + K_V * action_vel[3]

                # actual control action current&torque conversion here
                currentstep_Nratio = 4096 / (1.2 / 0.01) # n_steps/(Max_torque/Pulley_radius) # approx 34
                current_input4 = u4 * currentstep_Nratio #* gain_check
                current_input1 = u1 * currentstep_Nratio #* gain_check
                current_input2 = u2 * currentstep_Nratio #* gain_check
                current_input3 = u3 * currentstep_Nratio * 0 #* gain_check
                t_old = t
                q_old = q_pcc

                if (C.WANNA_PRINT and j%C.TIME2PRINT == 0):
                    print('[INFO]:\n\tTime: {} [sec] Step: {}'.format(np.round(t,2), j))
                    # print('[INFO]:\n\tTime: {} [sec] Step: {}\n\tInput Nm: [ {}, {}, {}, {}]'\
                    #     .format(np.round(t,2), j, np.round(current_input1, 2), np.round(current_input2, 3),
                    #             np.round(current_input3, 2), np.round(current_input4, 2)))
                    print('\tIMU: [{}, {}, {}]\n\tq Delta: [{}, {}, {}]'\
                        .format(np.round(IMU[0],2),np.round(IMU[1],2),np.round(IMU[2],2), \
                            np.round(q[0],2),np.round(q[1],2),np.round(q[2],2)))
                    print('\tq PCC: [{}, {}, {}]'\
                        .format(np.round(q_pcc[0],2),np.round(q_pcc[1],2),np.round(q_pcc[2],2)))
                    # print('\tIMU rad: [{}, {}, {}]'\
                    #     .format(np.round(IMU_rad[0],2),np.round(IMU_rad[1],2),np.round(IMU_rad[2],2)))
                    # print('\tErr Load: [{}, {}, {}, {}]'\
                    #     .format(np.round(e1,2),np.round(e2,2),np.round(e3,2), np.round(e4, 2)))
                    print('\tu_i: [{}, {}, {}, {}]'\
                        .format(np.round(u1,2),np.round(u2,2),np.round(u3,2), np.round(u4, 2)))
                    print('========================================================================================')
                else:
                    pass

                q_vett.append(q)
                q_des_vett.append(q_des[conta_time,:])
                q_dot_des_vett.append(q_dot_des[conta_time,:])
                imu_gyro_vett.append(IMU_gyro)
                imu_rad_vett.append(IMU_rad)
                imu_lin_vett.append(IMU)
                e_vett.append(np.array([e1, e2, e3, e4]))
                time_vett.append(t)
                u_sent_vett.append([u1, u2, u3, u4])

                ## sending the control to the robot
                servo.write_current(current_input4, ID=4)
                servo.write_current(current_input3, ID=3)
                servo.write_current(current_input2, ID=2)
                servo.write_current(current_input1, ID=1) # motore gira dall'altra parte non so perchè

                data = [time.time() - timer, \
                        measured_load_1, measured_load_2, measured_load_3, measured_load_4, \
                        current_input1, current_input2, current_input3, current_input4, \
                        IMU[0], IMU[1], IMU[2], \
                        IMU_gyro[0],IMU_gyro[1], IMU_gyro[2], \
                        IMU_rad[0], IMU_rad[1], IMU_rad[2],\
                        q[0], q[1], q[2], 
                        q_pcc[0], q_pcc[1], q_pcc[2], 
                        q_des[0], q_des[1], q_des[2], 
                        u_sent_vett[0], u_sent_vett[1], u_sent_vett[2], u_sent_vett[3]]
                writer.writerow(data)

        except KeyboardInterrupt:
            print(colored('========================================================================================', 'red'))
            print(colored('ABORTING Manually ... Motor comunication OFF', 'red'))
            servo.end_communication()
            print(colored('========================================================================================', 'red'))
            print('\n')
            raise SystemExit
        ############################################################# END COMUNICATION FOR ONE ITERATION
    ############################################################# END TIME FOR ONE ITERATION
    servo.end_communication()
    csvFile.close()
    print('========================================================================================')
    time.sleep(1)

    #############################################################
    ############### OFFLINE PROCEDURE
    #############################################################

    ## Looking at data coming from the current iteration
    dt_exp = np.mean(dt_exp_vett)
    q_vett = np.asanyarray(q_vett)
    ## This is stupid but It won't work with a forloop
    app_q_dot_1 = np.gradient(q_vett[:,0], dt_exp)
    app_q_dot_2 = np.gradient(q_vett[:,1], dt_exp)
    app_q_dot_3 = np.gradient(q_vett[:,2], dt_exp)
    app_q_ddot_1 = np.gradient(app_q_dot_1, dt_exp)
    app_q_ddot_2 = np.gradient(app_q_dot_2, dt_exp)
    app_q_ddot_3 = np.gradient(app_q_dot_3, dt_exp)
    q_dot_vett = np.column_stack([app_q_dot_1,app_q_dot_2, app_q_dot_3])
    q_ddot_vett = np.column_stack([app_q_ddot_1,app_q_ddot_2, app_q_ddot_3])

    if conta_time < 2 * C.max_step:
        time_vett = resampleData(time_vett, 2 * C.max_step)
        q_vett = resampleData(q_vett, 2 * C.max_step)
        q_des_vett = resampleData(q_des_vett, 2 * C.max_step)
        q_dot_des_vett = resampleData(q_dot_des_vett, 2 * C.max_step)
        q_dot_vett = resampleData(q_dot_vett, 2 * C.max_step)
        q_ddot_vett = resampleData(q_ddot_vett, 2 * C.max_step)
        current_input_vett = resampleData(current_input_vett, 2 * C.max_step)
        imu_gyro_vett = resampleData(imu_gyro_vett, 2 * C.max_step)
        imu_rad_vett = resampleData(imu_rad_vett, 2 * C.max_step)
        imu_lin_vett = resampleData(imu_lin_vett, 2 * C.max_step)
        e_vett = resampleData(e_vett, 2 * C.max_step)
        time_vett = resampleData(time_vett, 2 * C.max_step)

        print('[INFO]:\tI had lost some data, i.e., {} over {}, dt: {}. Resampling ...'.format(conta_time, 2 * C.max_step, np.round(dt_exp,3)))
        print('========================================================================================')
    else:
        pass

    for kkk in range(0, C.n_state):
        q_vett[:,kkk] = savgol_filter(q_vett[:,kkk], window_length=9, polyorder=2)
        q_dot_vett[:,kkk] = savgol_filter(q_dot_vett[:,kkk], window_length=9, polyorder=2)
        q_ddot_vett[:,kkk] = savgol_filter(q_ddot_vett[:,kkk], window_length=9, polyorder=2)

    plt.figure(clear=True)
    plt.xlabel(r'Time $[s]$')
    plt.ylabel(r'Test $q_{pcc}$ AFTER TRASL')
    for i_q in range(0, C.n_state):
        plt.plot(time_vett, q_des[:, i_q], color=C.color_list[i_q],\
                label=C.label_list_q[i_q], linestyle='-.', linewidth=C.line_widt)
        plt.plot(time_vett, q_vett[:, i_q], color=C.color_list[i_q],\
                label=C.label_list_q[i_q], linewidth=C.line_widt)
    plt.grid()
    plt.legend(loc='best', shadow=True, fontsize=C.font_size)
    plt.show()

    print('========================================================================================')
    print('[INFO]:\tSaving Stuffs ...')
    print('========================================================================================')
    time.sleep(2)

    if i%C.TIME2SAVE == 0:
        save_data(directory,
                i,
                imu_rad_vett,
                q_vett,
                q_dot_vett,
                q_ddot_vett,
                imu_gyro_vett,
                imu_lin_vett,
                e_vett,
                current_input_vett,
                time_vett,
                u_new,
                err_RMS,
                q_des_vett, 
                q_des)
        save_plot(directory, i)
    else:
        pass
    
