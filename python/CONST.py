import numpy as np

robot_name = 'PCC_Delta'
### REFERENCES FETAURES
t_f = 2
dt = 0.3
max_step = 50 # 80 # 200
time_task = np.linspace(0, t_f, max_step + 1)
max_iter = 100

### BOOL USEFUL FOR CONTROL
JUST_TORQUE_CONTR = True # False # False # True
WANNA_STAY_STILL = False # True # False
PRINT_ONE_ILC = True
WANNA_FB = False
WANNA_MODEL_BASED = False
PRE_TIME = True
WANNA_MODEL_BASED_INIT = True # True # False
WANNA_PRINT = True
WANNA_PLOT = False
TIME2PRINT = 5
TIME2SAVE = 1
ONLY_DATA_CONTROL = False

### MODEL FETAURES
n_state = 3
n_output = 3
n_actuator = 4
C_out = np.eye(n_state)
toll_state = 1e-2 # toll onto theta (singularity in the PCC model)
u_sat = 2.3 # torque mode 4.2
toll_err = 1e-2

### PLOT CONSTANTS
line_widt = 3
font_size = 1 * 15
off_set = 0.2
marker_size = 5 * 3 
line_width_robot = 5 * 3
TIME_2_PAUSE_PLOT = 0.5
WANNA_SAVE_BIG = False
ITER2SAVE = 10
PRINT_ONE = True
label_list_cart = [r'X',r'Y',r'Z']
color_list = ['r', 'g', 'b', 'm', 'k', 'c', 'y']
if robot_name == 'PCC_Delta':
    label_list_q = [r'$\Delta_x$',r'$\Delta_x$', r'$\delta L$']
    label_list_q_dot = [r'$\dot{\Delta_x}$',r'$\dot{\Delta_y}$', r'$\dot{\delta L}$']
    label_list_q_ddot = [r'$\ddot{\Delta_x}$',r'$\ddot{\Delta_y}$', r'$\ddot{\delta L}$']
    label_list_y = [r'$\Delta_x$', r'$\Delta_y$', r'$\delta L$']
elif robot_name == 'PCC':
    label_list_q = [r'$\phi$',r'$\theta$', r'$\delta L$']
    label_list_q_dot = [r'$\dot{\phi}$',r'$\dot{\theta}$', r'$\dot{\delta L}$']
    label_list_q_ddot = [r'$\ddot{\phi}$',r'$\ddot{\theta}$', r'$\ddot{\delta L}$']
    label_list_y = [r'$\phi$', r'$\theta$', r'$\delta L$']
else:
    pass

### CONTROL GAINS TO SET THE SAME TENSION ONTO THE TENDONDS
force_demand = 0.05 # 0.1 # 0.08
kp = 0.7 # 1 # 0.08
kp_ilc = 0.0
kd = 0.5
ki = 0

### PHYSICAL MEASURES
theta_base = np.pi/4 # [rad] base angle from the frame to the actuator
theta_head_1 = np.pi/4 # [rad] head angle from the frame to the actuator
r_base = 0.5 # [m] base length (module) from the frame to the actuator
r_head_1 = 0.5 # [m] head length (module) from the frame to the actuator
r = 1 # [m] model Delta how to get to the actuator. 
d = 1 # [m] grandezza della lunghezza check
l0 = 0.1 # 0.15 # 0.3 # [m]
ampl = 1
g = 0 # 9.81 gravity [Nm/s^2] positive sign 
m_0 = 0.5 # [Kg]
I_xx = 1 * m_0 * l0**2
I_yy = 1 * m_0 * l0**2
I_zz = 1/2 * m_0 * l0**2
d_ii = 1.0 * ampl # [Nm rad/s] # 5.0
kappa_theta = 5.0 * ampl # 1.0 # [Nm rad]
kappa_phi = 5.0 * ampl # 1.0 # [Nm rad]
kappa_dL = 25.0 * ampl # 5.0 # [Nm rad]