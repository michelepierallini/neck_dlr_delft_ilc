import numpy as np
import shutil
import CONST as C
import matplotlib.pyplot as plt
plt.rcParams['text.latex.preamble'] = ''.join([r'\usepackage{siunitx}', r'\usepackage{amsmath}'])
import os
import time

def get_folder_plot(directory):
    """
    Create a folder to store data
    Args:
        directory (string): path to the new folder to create
        
    Returns:
        _: create the folder
    
    """
    if os.path.exists(directory):
        shutil.rmtree(directory)
    os.makedirs(directory) 
 
def save_data(directory, 
            iteration, 
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
            q_des, 
            q_des_vett):
    """
    This function saves all the data coming from the simulation into .csv files.
    
    Args:
        directory (string): name of the directory to save the data /prova
        iteration (int): iteration number 
        imu_gyro_vett (double): Cartesian orinetation XYZ on the head wrt time
        imu_lin_vett (double): Cartesian positions XYZ on the head and wrt time
        e_vett (double): error wrt time. Note that this is a force
        current_input_vett (double): control input vector, i.e., current
        time_vett (double): time vector internal clock          
    """
    dir_plot = directory + '/Iter_{}/'.format(iteration)
    if os.path.exists(dir_plot):
        shutil.rmtree(dir_plot)
    os.makedirs(dir_plot)
    
    q_vett = np.array(q_vett)
    q_des_vett = np.array(q_des_vett)
    imu_rad_vett = np.array(imu_rad_vett)
    q_dot_vett = np.array(q_dot_vett)
    q_ddot_vett = np.array(q_ddot_vett)
    imu_gyro_vett = np.array(imu_gyro_vett) 
    imu_lin_vett = np.array(imu_lin_vett)
    e_vett = np.array(e_vett)
    current_input_vett = np.array(current_input_vett)
    time_vett = np.array(time_vett)
    u_new = np.array(u_new)
    err_RMS = np.array(err_RMS)
    q_des = np.array(q_des)

    Q = dir_plot + 'q_vett.csv'
    Q_DES_VETT = dir_plot + 'q_des_vett.csv'
    IMU_RAD = dir_plot + 'imu_rad_vett.csv'
    Q_DOT = dir_plot + 'q_dot_vett.csv'
    Q_DDOT = dir_plot + 'q_ddot_vett.csv'
    IMU_GYRO = dir_plot + 'imu_gyro_vett.csv'
    IMU_LIN = dir_plot + 'imu_lin_vett.csv'
    ERR = dir_plot + 'e_vett.csv'
    CURRENT_CONTROL_INPUT = dir_plot + 'current_input_vett.csv'
    TIME = dir_plot + 'time_vett.csv'
    ERR_RMS = dir_plot + 'err_rms.csv'
    U_NEW = dir_plot + 'u_new.csv'
    Q_DES = dir_plot + 'q_des.csv'

    np.savetxt(IMU_RAD, imu_rad_vett, delimiter=',') 
    np.savetxt(Q, q_vett, delimiter=',')
    np.savetxt(Q_DES_VETT, q_des_vett, delimiter=',')
    np.savetxt(Q_DOT, q_dot_vett, delimiter=',')
    np.savetxt(Q_DDOT, q_ddot_vett, delimiter=',')
    np.savetxt(IMU_GYRO, imu_gyro_vett, delimiter=',')
    np.savetxt(IMU_LIN, imu_lin_vett, delimiter=', ')
    np.savetxt(ERR, e_vett, delimiter=',')
    np.savetxt(CURRENT_CONTROL_INPUT, current_input_vett, delimiter=',')
    np.savetxt(TIME, time_vett, delimiter=",")
    np.savetxt(ERR_RMS, err_RMS, delimiter=",")
    np.savetxt(U_NEW, u_new, delimiter=",")
    np.savetxt(Q_DES, q_des, delimiter=",")

def save_plot(directory, iteration):
        
    """
    Save the plot for the current iteration
    Args:
        directory (string): name of the directory to save the data /prova
        iteration (int): iteration number
    """
    dir_plot = directory + '/Iter_{}/'.format(iteration)
    if os.path.exists(dir_plot + "plot/"):
        shutil.rmtree(dir_plot + "plot/")
    os.makedirs(dir_plot + "plot/")

    Q = dir_plot + 'q_vett.csv'
    IMU_RAD = dir_plot + 'imu_rad_vett.csv'
    Q_DOT = dir_plot + 'q_dot_vett.csv'
    Q_DDOT = dir_plot + 'q_ddot_vett.csv'
    IMU_GYRO = dir_plot + 'imu_gyro_vett.csv'
    IMU_LIN = dir_plot + 'imu_lin_vett.csv'
    ERR = dir_plot + 'e_vett.csv'
    CURRENT_CONTROL_INPUT = dir_plot + 'current_input_vett.csv'
    TIME = dir_plot + 'time_vett.csv'
    ERR_RMS = dir_plot + 'err_rms.csv'
    U_NEW = dir_plot + 'u_new.csv'
    Q_DES = dir_plot + 'q_des.csv'
    
    q_vett = np.genfromtxt(Q, delimiter=',')
    imu_rad_vett = np.genfromtxt(IMU_RAD, delimiter=',')
    q_dot_vett = np.genfromtxt(Q_DOT, delimiter=',')
    q_ddot_vett = np.genfromtxt(Q_DDOT, delimiter=',')
    imu_gyro_vett = np.genfromtxt(IMU_GYRO, delimiter=',')
    imu_lin_vett = np.genfromtxt(IMU_LIN, delimiter=',')
    err = np.genfromtxt(ERR, delimiter=',')
    current_input_vett = np.genfromtxt(CURRENT_CONTROL_INPUT, delimiter=',')
    time_task = np.genfromtxt(TIME, delimiter=',')
    u_new = np.genfromtxt(U_NEW, delimiter=',')
    err_rms = np.genfromtxt(ERR_RMS, delimiter=',')
    q_des = np.genfromtxt(Q_DES, delimiter=',')
    
    plt.figure(clear=True)
    plt.ylabel(r'RMS $[m]$')
    plt.xlabel(r'Iterations')
    plt.plot(range(0, iteration+1), err_rms[0:iteration+1], color='red', marker='o', \
             linestyle='-.', linewidth=C.line_widt, markersize=C.marker_size)
    plt.grid()
    plt.savefig(dir_plot + 'plot/' +'rms.svg', format='svg')
    if C.WANNA_PLOT:
        plt.show(block=False)
        plt.pause(C.TIME_2_PAUSE_PLOT)
    plt.close()
        
    plt.figure(clear=True)
    plt.ylabel(r'Action Already Sent $[Nm]$')
    plt.xlabel(r'Time $[s]$')
    for i in range(0, C.n_actuator):
        # assuming matrix n_time x n_actuators
        plt.plot(time_task, current_input_vett[:, i], label= 'u_' + str(i + 1), linewidth=C.line_widt)
    plt.grid()
    plt.legend(loc='best', shadow = True, fontsize=C.font_size)
    plt.savefig(dir_plot + 'plot/' + 'current_input.svg', format='svg')
    if C.WANNA_PLOT:
        plt.show(block=False)
        plt.pause(C.TIME_2_PAUSE_PLOT)
    plt.close()
    
    plt.figure(clear=True)
    plt.ylabel(r'Action New $[Nm]$')
    plt.xlabel(r'Time $[s]$')
    for i in range(0, C.n_actuator):
        # assuming matrix n_time x n_actuators
        plt.plot(time_task, u_new[:, i], label= 'u_' + str(i + 1), linewidth=C.line_widt)
    plt.grid()
    plt.legend(loc='best', shadow = True, fontsize=C.font_size)
    plt.savefig(dir_plot + 'plot/' + 'action_NEW.svg', format='svg')
    if C.WANNA_PLOT:
        plt.show(block=False)
        plt.pause(C.TIME_2_PAUSE_PLOT)
    plt.close()
    
    plt.figure(clear=True)
    plt.ylabel(r'q')
    plt.xlabel(r'Time $[s]$')
    for i in range(0, C.n_state):
        # assuming matrix n_time x n_actuators
        plt.plot(time_task, q_vett[:, i], label= r'q_' + str(i + 1), linewidth=C.line_widt)
    plt.grid()
    plt.legend(loc='best', shadow = True, fontsize=C.font_size)
    plt.savefig(dir_plot + 'plot/' + 'q.svg', format='svg')
    if C.WANNA_PLOT:
        plt.show(block=False)
        plt.pause(C.TIME_2_PAUSE_PLOT)
    plt.close()
    
    plt.figure(clear=True)
    plt.ylabel(r'q')
    plt.xlabel(r'Time $[s]$')
    for i in range(0, C.n_state):
        # assuming matrix n_time x n_actuators
        plt.plot(time_task, q_des[:, i], label= r'q_' + str(i + 1) + 'Des', linewidth=C.line_widt, linestyle='-.')
        plt.plot(time_task, q_vett[:, i], label= r'q_' + str(i + 1), linewidth=C.line_widt)

    plt.grid()
    plt.legend(loc='best', shadow = True, fontsize=C.font_size/2)
    plt.savefig(dir_plot + 'plot/' + 'Z_q_comapre.svg', format='svg')
    if C.WANNA_PLOT:
        plt.show(block=False)
        plt.pause(C.TIME_2_PAUSE_PLOT)
    plt.close()

    plt.figure(clear=True)
    plt.ylabel(r'\dot{q}')
    plt.xlabel(r'Time $[s]$')
    for i in range(0, C.n_state):
        # assuming matrix n_time x n_actuators
        plt.plot(time_task, q_dot_vett[:, i], label= C.label_list_q_dot[i], color=C.color_list[i], linewidth=C.line_widt)
    plt.grid()
    plt.legend(loc='best', shadow = True, fontsize=C.font_size)
    plt.savefig(dir_plot + 'plot/' + 'q_dot.svg', format='svg')
    if C.WANNA_PLOT:
        plt.show(block=False)
        plt.pause(C.TIME_2_PAUSE_PLOT)
    plt.close()
    
    plt.figure(clear=True)
    plt.ylabel(r'\ddot{q} $[N]$')
    plt.xlabel(r'Time $[s]$')
    for i in range(0, C.n_state):
        # assuming matrix n_time x n_actuators
        plt.plot(time_task, q_ddot_vett[:, i], label=C.label_list_q_ddot[i], color=C.color_list[i], linewidth=C.line_widt)
    plt.grid()
    plt.legend(loc='best', shadow = True, fontsize=C.font_size)
    plt.savefig(dir_plot + 'plot/' + 'q_ddot.svg', format='svg')
    if C.WANNA_PLOT:
        plt.show(block=False)
        plt.pause(C.TIME_2_PAUSE_PLOT)
    plt.close()
    
    plt.figure(clear=True)
    plt.xlabel(r'Time $[s]$')
    plt.ylabel(r'IMU Rad $[rad]$')
    for i in range(0, 3):
        plt.plot(time_task, imu_rad_vett[:, i], color=C.color_list[i],label=C.label_list_cart[i], linewidth=C.line_widt)
    plt.grid()
    plt.legend(loc='best', shadow=True, fontsize=C.font_size)
    plt.savefig(dir_plot + 'plot/' + 'imu_rad.svg', format='svg')
    if C.WANNA_PLOT:
        plt.show(block=False)
        plt.pause(C.TIME_2_PAUSE_PLOT)
    plt.close()

    plt.figure(clear=True)
    plt.xlabel(r'Time $[s]$')
    plt.ylabel(r'IMU Degrees$[^\circ]$')
    for i in range(0, 3):
        plt.plot(time_task, imu_lin_vett[:, i], color=C.color_list[i],label=C.label_list_cart[i], linewidth=C.line_widt)
    plt.grid()
    plt.legend(loc='best', shadow=True, fontsize=C.font_size)
    plt.savefig(dir_plot + 'plot/' + 'imu_degree.svg', format='svg')
    if C.WANNA_PLOT:
        plt.show(block=False)
        plt.pause(C.TIME_2_PAUSE_PLOT)
    plt.close()
    
    plt.figure(clear=True)
    plt.xlabel(r'Time $[s]$')
    plt.ylabel(r'IMU Gyro')
    for i in range(0, 3):
        plt.plot(time_task, imu_gyro_vett[:, i], color=C.color_list[i],label=C.label_list_cart[i], linewidth=C.line_widt)
    plt.grid()
    plt.legend(loc='best', shadow=True, fontsize='10')
    plt.savefig(dir_plot + 'plot/' + 'imu_gyro.svg', format='svg')
    if C.WANNA_PLOT:
        plt.show(block=False)
        plt.pause(C.TIME_2_PAUSE_PLOT)
    plt.close()
    
    plt.figure(clear=True)
    plt.ylabel(r'Error Tracking')
    plt.xlabel(r'Time $[s]$')
    for i in range(0, C.n_output):
        plt.plot(time_task, err[:, i], label= 'err_' + str(i + 1), linewidth=C.line_widt)
    plt.grid()
    plt.legend(loc='best', shadow=True, fontsize=C.font_size)
    plt.savefig(dir_plot + 'plot/' + 'error.svg', format='svg')
    if C.WANNA_PLOT:
        plt.show(block=False)
        plt.pause(C.TIME_2_PAUSE_PLOT)
    plt.close()
        
    time.sleep(1)    

