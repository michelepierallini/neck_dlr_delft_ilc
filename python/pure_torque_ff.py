# import contextlib
import sys
import time
import socket
import numpy as np
from PCC_class import DataRobot
from ILC_SoftRobot import ILC_SoftRobot
import CONST as C
import csv
# import matplotlib
# matplotlib.use('Agg')
# from matplotlib import pyplot as plt
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
############ THIS SCRIPT IMPLEMENT JUST A PURE TORQUE ACTION TO BE SEND TO THE MOTOR 
############ THIS I MAINLY NEEDED TO RECORD THE VIDEOS THAT I FORGOT
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

directory2load = 'Test_EXP_PURE_TORQUE_014_30_03_2023_initialguess'
iter_des = 10
directory = os.getcwd() + '/Test_EXP_PURE_TORQUE_015_30_03_2023_initialguess'
if os.path.isdir(directory):
    directory = directory + '_ANOTHERONE_DJKHALED_' +  int(np.floor(np.random.rand(1) * 100))
get_folder_plot(directory)

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

q0 = np.array([-0.5, 0.1, 0]) 
q_dot0 = np.zeros(3)
q_ddot0 = np.zeros(3)
u0 = np.zeros(C.n_actuator)

target_pcc = np.array([-1.3, 0.5, -0.03])
q_des = np.zeros((C.max_step, C.n_state))
q_dot_des = np.zeros((C.max_step, C.n_state))
q_ddot_des = np.zeros((C.max_step, C.n_state))

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
## write data into a file.csv
fileHeader = ["time", \
    "measured_load_1", "measured_load_2", "measured_load_3", "measured_load_4", \
    "current_input1", "current_input2", "current_input3", "current_input4",\
    "u_old_1", "u_old_2", "u_old_3", "u_old_4",
    "IMU_x", "IMU_y", "IMU_z", "IMU_gx", "IMU_gy", "IMU_gz",\
    "IMU_rad_x", "IMU_rad_y", "IMU_rad_z", \
    "phi", "theta", "dL", 
    "phi_des", "theta_des", "dL_des" ]

csvFile = open( directory + "/dataset_pure_torque" + ".csv", "w")
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
#] 0.1 is the minimum value you can set to check how much they are loose
force_demand1 = C.force_demand
force_demand2 = C.force_demand
force_demand3 = C.force_demand
force_demand4 = C.force_demand

# data2save current test 
imu_lin_vett, imu_gyro_vett, imu_rad_vett, current_input_vett, u_sent_vett = [], [], [], [], []
time_vett, q_vett, q_dot_vett, q_ddot_vett, dt_exp_vett, e_vett, q_des_vett, q_dot_des_vett = [], [], [], [], [], [], [], []
MSE = np.zeros(C.n_output)

robot = DataRobot(q0,
                q_dot0,
                q_ddot0,
                u0,
                underactuation_matrix=np.eye(C.n_state))

ilc_controller = ILC_SoftRobot(robot,
                                C.n_actuator,
                                C.n_state,
                                q_des,
                                q_dot_des,
                                q_ddot_des)

path_old_control = directory2load + '/Iter_{}/'.format(iter_des)
CONTROL_PATH = path_old_control + 'u_new.csv'
TIME_PATH = path_old_control + 'time_vett.csv'
time_ = np.genfromtxt(TIME_PATH, delimiter=',')
u_old = np.genfromtxt(CONTROL_PATH, delimiter=',')

if C.WANNA_PLOT:
    plt.figure(clear=True)
    plt.xlabel(r'Time $[s]$')
    plt.ylabel(r'Control Input 2BeSent RN')
    for i_a in range(0, C.n_actuator):
        plt.plot(time_, u_old[:, i_a], color=C.color_list[i_a],\
                label='u_' + str(i_a + 1), linewidth=C.line_widt)
    plt.grid()
    plt.legend(loc='best', shadow=True, fontsize=C.font_size)
    plt.show()

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

            if C.JUST_TORQUE_CONTR:
            ######################################################################################################
            ######## TORQUE CONTROL MOTOR LEVEL 
            ######################################################################################################

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

                u1 = (e1 * k_p + u1_pre - u_old[j, 1] * gain_check) 
                u2 = e2 * k_p + u2_pre + u_old[j, 0] * gain_check
                u3 = e3 * k_p + u3_pre + u_old[j, 2] * gain_check
                u4 = e4 * k_p + u4_pre + u_old[j, 3] * gain_check

            else:

            ######################################################################################################
            ######## TORQUE CONTROL ON THE LOADCELL
            ######################################################################################################

                force_demand1 = force_demand1 + u_old[j, 1] 
                force_demand2 = force_demand2 + u_old[j, 0]
                force_demand3 = force_demand3 + u_old[j, 2]
                force_demand4 = force_demand4 +  u_old[j, 3]

                ## Force error [N]. THIS IS A FEEDBACK TO SET THE PRE-TENSION ON THE TENDONS
                e1 = force_demand1 - measured_load_1
                e2 = force_demand2 - measured_load_2
                e3 = force_demand3 - measured_load_3
                e4 = force_demand4 - measured_load_4

                k_p = C.kp

                u2 = e2 * k_p * gain_check
                u1 = e1 * k_p * gain_check
                u3 = e3 * k_p * gain_check
                u4 = e4 * k_p * gain_check

                ## security check 
                if abs(u1) > C.u_sat/2:
                    u1 = np.sign(u1) * C.u_sat/2
                
                if abs(u2) > C.u_sat/2:
                    u2 = np.sign(u2) * C.u_sat/2
                
                if abs(u3) > C.u_sat/2:
                    u3 = np.sign(u3) * C.u_sat/2

                if abs(u4) > C.u_sat/2:
                    u4 = np.sign(u4) * C.u_sat/2

            ## actual control action current&torque conversion here
            currentstep_Nratio = 4096 / (1.2 / 0.01)  # n_steps/(Max_torque/Pulley_radius) # approx 34
            current_input4 = u4 * currentstep_Nratio #* gain_check
            current_input1 = u1 * currentstep_Nratio #* gain_check
            current_input2 = u2 * currentstep_Nratio #* gain_check
            current_input3 = u3 * currentstep_Nratio * 0 # colla #* gain_check
            t_old = t

            IMU_rad = convertIMUStep(IMU)
            q = ypr2pcc(IMU_rad, C.l0)

            if (C.WANNA_PRINT and j%C.TIME2PRINT == 0):
                print('[INFO]:\n\tTime: {} [sec] Step: {}'.format(np.round(t,2), j))
                # print('[INFO]:\n\tTime: {} [sec] Step: {}\n\tInput Nm: [ {}, {}, {}, {}]'\
                #     .format(np.round(t,2), j, np.round(current_input1, 2), np.round(current_input2, 3),
                #             np.round(current_input3, 2), np.round(current_input4, 2)))
                print('\tIMU: [{}, {}, {}]\n\tq: [{}, {}, {}]'\
                    .format(np.round(IMU[0],2),np.round(IMU[1],2),np.round(IMU[2],2), \
                        np.round(q[0],2),np.round(q[1],2),np.round(q[2],2)))
                # print('\tIMU rad: [{}, {}, {}]'\
                #     .format(np.round(IMU_rad[0],2),np.round(IMU_rad[1],2),np.round(IMU_rad[2],2)))
                # print('\tErr Load: [{}, {}, {}, {}]'\
                #     .format(np.round(e1,2),np.round(e2,2),np.round(e3,2), np.round(e4, 2)))
                print('\tu_i: [{}, {}, {}, {}]'\
                    .format(np.round(u1,2),np.round(u2,2),np.round(u3,2), np.round(u4, 2)))
                print('\tu ILC: [{}, {}, {}, {}]'\
                    .format(np.round(u_old[j, 0],2),np.round(u_old[j, 1],2),np.round(u_old[j, 2],2), np.round(u_old[j, 3], 2)))
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
            current_input_vett.append(np.array([u_old[j, 0], u_old[j, 1], u_old[j, 2], u_old[j, 3]]))
            time_vett.append(t)
            u_sent_vett.append(u_old)

            ## sending the control to the robot
            servo.write_current(current_input4, ID=4)
            servo.write_current(current_input3, ID=3)
            servo.write_current(current_input2, ID=2)
            servo.write_current(current_input1, ID=1)

            data = [time.time() - timer, \
                    measured_load_1, measured_load_2, measured_load_3, measured_load_4, \
                    current_input1, current_input2, current_input3, current_input4, \
                    u_old[j, 0],u_old[j, 1], u_old[j, 2], u_old[j, 3],
                    IMU[0], IMU[1], IMU[2], \
                    IMU_gyro[0],IMU_gyro[1], IMU_gyro[2], \
                    IMU_rad[0], IMU_rad[1], IMU_rad[2],\
                    q[0], q[1], q[2], 
                    q_des[0], q_des[1], q_des[2]]
            writer.writerow(data)

    except KeyboardInterrupt:
        print(colored('========================================================================================', 'red'))
        print(colored('ABORTING Manually ... Motor comunication OFF', 'red'))
        servo.end_communication()
        print(colored('========================================================================================', 'red'))
        print('\n')
        raise SystemExit
    ############################################################# END COMUNICATION FOR ONE ITERATION
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

init_disp_real = [0, 0, 0]
for iii in range(0, C.n_state):
    init_disp_real[iii] = q_vett[C.max_step - 1, iii]
    # init_disp_real[iii] = np.mean(q_vett[C.max_step - int(np.floor(C.max_step/5)):C.max_step - 3, iii])
disp_init = q0 - init_disp_real
print('[INFO]: Initial Display too big: [{}, {}, {}]'\
        .format(np.round(disp_init[0],3), np.round(disp_init[1],3), np.round(disp_init[2],3)))
print('========================================================================================')
q_vett = q_vett + disp_init

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

for kkk in range(0, C.n_state):
    q_vett[:,kkk] = savgol_filter(q_vett[:,kkk], window_length=9, polyorder=2)
    q_dot_vett[:,kkk] = savgol_filter(q_dot_vett[:,kkk], window_length=9, polyorder=2)
    q_ddot_vett[:,kkk] = savgol_filter(q_ddot_vett[:,kkk], window_length=9, polyorder=2)

## compute the error during this test only for that porpouse
for time_i in range(0, 2 * C.max_step):
    if C.ONLY_DATA_CONTROL:
        # Control limit 4 N
        u_new[time_i, :], error_ilc_pos[time_i, :] = ilc_controller.getNewControlEachTime(q_vett[time_i, :],
                                                                                        q_dot_vett[time_i, :],
                                                                                        q_ddot_vett[time_i, :],
                                                                                        u_old[time_i, :],
                                                                                        time_i)
        u_new[time_i, :] = checkControlUpZero(u_new[time_i, :])
    else:        
        u_new[time_i, :], error_ilc_pos[time_i, :] = ilc_controller.GetControlModelBasedDelayIssue(u_old[time_i, :],
                                                                                            q_vett[time_i, :],
                                                                                            q_dot_vett[time_i, :],
                                                                                            q_vett[time_i, :],
                                                                                            q_dot_vett[time_i, :],
                                                                                            q_ddot_vett[time_i, :],
                                                                                            q_des[time_i, :], 
                                                                                            q_dot_des[time_i, :],
                                                                                            time_i) 
        u_new[time_i, :] = checkControlUpZero(u_new[time_i, :])

## computing error within the iteration
for iii in range(0, C.n_output):
    MSE[iii] = np.square(error_ilc_pos[:, iii]).mean()
    err_RMS = math.sqrt(np.linalg.norm(MSE, 2))

print(colored('========================================================================================', 'yellow'))
print(colored('[INFO]:\t\tPure Torque lasts {} [sec] with dt: {} [sec]'.format(np.round(t,2), np.round(dt_exp, 3)),'yellow'))
print(colored('[INFO]:\t\tErr RMS is {}'.format(np.round(err_RMS, 5)), 'yellow'))
print(colored('========================================================================================', 'yellow'))

print('========================================================================================')
print('[INFO]:\tSaving Stuffs ...')
print('========================================================================================')
time.sleep(2)

save_data(directory,
        0,
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
save_plot(directory, 0)
