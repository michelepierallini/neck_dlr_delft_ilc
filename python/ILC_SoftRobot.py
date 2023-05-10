import numpy as np
import CONST as c
from utility import*

class ILC_SoftRobot():
    def __init__(self,
                robot, 
                m, 
                p,
                y_des,
                y_dot_des,
                y_ddot_des,
                learning_gain_ff = None, 
                learning_gain_fb = None):
        """
        This class implements a model free ILC action for a system with relative degree equal to 2.
        Both the learning gains have to be square matrices 

        NOTE: the ILC control weights have to be selected  

        Args:
            m (int): number of actuators of the robot.
            p (int): number of outputs of the robot.
            max_step (int): numbr of time steps for each trajectory.
            dt (double): sampling time for each trajectory.
            y_des (double, optional): desired position trajectory.
            y_dot_des (double, optional): desired velocity trajectory.
            y_ddot_des (double, optional): desired acceleration trajectory.
            learning_gain_ff (float, optional): feedforward learning gain
            learning_gain_fb (float, optional): feedback learning gain
        """
                
        self.m = int(m)
        self.p = int(p)
        self.y_des = np.asarray(y_des, dtype=np.float64)
        self.y_dot_des = np.asarray(y_dot_des, dtype=np.float64)
        self.y_ddot_des = np.asarray(y_ddot_des, dtype=np.float64)
        self.robot = robot

        self.small = 0.05
        
        if self.m > 1:
            ## MIMO
            ## Gains
            if learning_gain_fb is None:
                self.Gamma_fb = np.zeros((self.m, self.m))
            else: 
                self.Gamma_fb = learning_gain_fb
            
            if learning_gain_ff is None:
                self.Gamma_ff = np.eye(self.m, self.p)
            else: 
                self.Gamma_ff = learning_gain_ff

            ## ff gains control weight 
            self.gamma0 = 20 * np.eye(self.p, self.p) 
            self.gamma1 = 0.5 * np.eye(self.p, self.p) 
            self.gamma2 = 0.01 * np.eye(self.p, self.p)
            
            ## fb gains control weight 
            self.K_P = 0.1 * np.eye(self.p, self.p)
            self.K_V = 0.01 * np.eye(self.p, self.p)
            self.K_A = 0.0001 * np.eye(self.p, self.p)
        else:
            ## SISO
            ## Gains
            if learning_gain_fb is None:
                self.Gamma_fb = 0.1 
            else: 
                self.Gamma_fb = learning_gain_fb
            
            if learning_gain_ff is None:
                self.Gamma_ff = 0.1
            else: 
                self.Gamma_ff = learning_gain_ff

            self.gamma0 = 10
            self.gamma1 = 1
            self.gamma2 = 0.001 

            ## fb gains control weight 
            self.K_P = 0.1
            self.K_V = 0.05 
            self.K_A = 0.001 
    
    def initalGuessModelbased(self):
        ## This if-case allows to select for underactuated robot, which has not been tested yet
        if c.robot_name == 'PCC_Delta':
            u0 = np.zeros((c.max_step, c.n_actuator))
            if c.PRE_TIME:
                print('[INFO]:\tAssuming pretime 2xtime_task ...')
                print('========================================================================================')

            if c.PRE_TIME: 
                for i in range(0, c.max_step):
                    self.robot.spinOnes(self.y_des[i + c.max_step, :], self.y_dot_des[i + c.max_step,:], np.zeros(c.n_actuator))
                    self.robot.A = -10 * self.small * np.array([[1/2, 1/2, -1/4], [1/2, -1/2, -1/4], [-1/2, 1/2, -1/4], [-1/2, -1/2, -1/4] ]) # correct to use with 4 tendonds
                
                    u0[i, :] = np.dot(self.robot.A, \
                        np.dot(self.robot.M, self.y_ddot_des[i + c.max_step, :]) + np.dot(self.robot.C, self.y_dot_des[i + c.max_step, :]) \
                        + self.robot.G + self.robot.D + self.robot.K)
                    
                    u0[i, :] = self.satAction(u0[i, :])
                    u0[i, :] = checkControlUpZero(u0[i, :])
            else:
                for i in range(0, c.max_step):
                    self.robot.spinOnes(self.y_des[i, :], self.y_dot_des[i,:], np.zeros(c.n_actuator)) ## never tested underactuation
                    u0[i, :] = np.dot(np.linalg.pinv(self.robot.A), \
                        np.dot(self.robot.M, self.y_ddot_des[i, :]) + np.dot(self.robot.C, self.y_dot_des[i, :]) \
                        + self.robot.G + self.robot.D + self.robot.K)
                    
                    u0[i, :] = self.satAction(u0[i, :])
                    u0[i, :] = checkControlUpZero(u0[i, :])

            return u0
        else: 
            pass
            
                
    def getNewControlEachTime(self,
                                y,
                                y_dot,
                                y_ddot,
                                u_old,
                                i):

        """
        Compute the new control each time.
        Note that the iteration domain does not affect this function.
        This function just implements a PDD-like action w.r.t. the iteration domain.
        
        NOTE THAT THIS FUNCTION WORKS FOR BOTH SISO AND MIMO SYSTEMS

        Args:
            y (double): curent position trajectory
            y_dot (double): curent velocity trajectory
            y_ddot (double): curent acceleration trajectory
            u_old (double): old  control action
            i (int): time index

        Returns:
            u_new (double): new control action purely feedforward
            err_for_control (double): error to use for the control action
            error_in_iter (double): error wrt time for each iteration
        """
        u_new = np.zeros(self.m)
        err_for_control = np.zeros(self.p)
        u_new_ff = []
        error_in_iter = np.zeros(self.p)
        
        if i >= c.max_step:
            if self.p == 1:
                err_for_control = self.gamma0 * (self.y_des[i] - y) + \
                    self.gamma1 * (self.y_dot_des[i] - y_dot) + \
                    self.gamma2 * (self.y_ddot_des[i] - y_ddot)
            else:
                err_for_control = np.dot(self.gamma0, self.y_des[i, :] - y) + \
                    np.dot(self.gamma1, self.y_dot_des[i, :] - y_dot) + \
                    np.dot(self.gamma2, self.y_ddot_des[i, :] - y_ddot)
                
            u_new_ff = np.dot(self.Gamma_ff, err_for_control)
            u_new = u_old + u_new_ff
            if self.p == 1:
                error_in_iter[i] = abs(y - self.y_des[i])
            else:    
                for mm in range(0, self.p):
                    error_in_iter[mm] = abs(y[mm] - self.y_des[i, mm])
        else:
            ## pre-time to allow the IMU to warm up
            u_new = np.zeros(self.m)
            error_in_iter = np.zeros(self.p)

        u_new = self.satAction(u_new)
        
        return u_new, error_in_iter
    
    def satAction(self, tau_old):
        n = c.n_actuator
        
        for i in range(0,n):
            if abs(tau_old[i]) > c.u_sat and i < 2:
                tau_old[i] = np.sign(tau_old[i]) * c.u_sat
            ## it seems that the motor 3 and 4 needs a little more torque to move the neck 
            elif i >= 2 and abs(tau_old[i]) > c.u_sat + 0.3:
                tau_old[i] = np.sign(tau_old[i]) * c.u_sat + 0.3
                pass
        return tau_old
    
   
    def GetControlModelBasedDelayIssue(self, 
                        tau_old, 
                        y, 
                        y_dot, 
                        q, 
                        q_dot, 
                        q_ddot,
                        q_des, 
                        q_dot_des,
                        i):
        """_summary_
        
        This function implent a model-based control action over one time instant and one iteration.
        Note that this could be implemented for both the FEEDFORWARD AND FEEDBACK learning gains.
        This code does not imply any dimensional constraints on the dimension of the learning gians.
        
        NOTE THAT THIS FUNCTION ONLY WORKS FOR MIMO SYSTEMS

        Args:
            robot (class): robot which all the methods related to it 
            tau_old (double): previous control action
            y (double): current output 
            y_dot (double): current output derivative 
            q (double): current robot configuration 
            q_dot (double): current robot configuration derivative
            q_ddot (double): current robot configuration derivative derivative
            i (int): index which runs on the time domain
        Returns:
            _type_: 
            err (double): error 
            err_dot (double): error derivattive
            uNew (double): new control action
        """

        u_new = np.zeros(self.m)
        err_for_control = np.zeros(self.p)
        error_in_iter = np.zeros(self.p)

        if i >= c.max_step:
            self.robot.spinOnes(q, q_dot, tau_old)
            err_dot = q_dot_des - y_dot
            err = q_des - y
            err_for_control = np.dot(self.gamma1, err_dot) + np.dot(self.gamma0, err)
            tau_new = np.dot(c.C_out, self.robot.q_ddot) # y_ddot
            
            ## tau_new = q_ddot # only data 
            ## purely model based control 
            learning_gain2 = -20 * self.small * np.array([[1/2, 1/2, -1/4], [1/2, -1/2, -1/4], [-1/2, 1/2, -1/4], [-1/2, -1/2, -1/4] ])
            u_new = tau_old + self.small * np.dot(learning_gain2, (np.dot(self.gamma2, self.y_ddot_des[i, :] - tau_new) + err_for_control))
            u_new = self.satAction(u_new) 

            for mm in range(0, self.p):
                error_in_iter[mm] = abs(y[mm] - self.y_des[i, mm])
        else:
            ## pre-time to allow the IMU to warm up
            u_new = np.zeros(self.m)
            error_in_iter = np.zeros(self.p)

        return u_new, error_in_iter
    
## Main to test ONLY the intial guess 
# if __name__ == '__main__':

#     print('-----------------------------------------\n\tStarting Main ...')
#     q0 = np.array([-0.4, 0.1, 0]) # piego verso 12
#     q_dot0 = np.zeros(3)
#     q_ddot0 = np.zeros(3)
#     u0 = np.zeros(4)
#     t_f = 10
#     max_step = 150
#     target_pcc = np.array([0.8, 0.8, -0.2]) # piego verso 24
#     q_des = np.zeros((max_step, 3))
#     q_dot_des = np.zeros((max_step, 3))
#     q_ddot_des = np.zeros((max_step, 3))

#     for j in range(0, 3):
#         q_des[:,j], q_dot_des[:,j], q_ddot_des[:,j] = mjRef(q0[j],
#                                                             target_pcc[j],
#                                                             t_f,
#                                                             max_step)
        
#     time = np.linspace(0, t_f, max_step)
    
#     robot = DataRobot(q0,
#                     q_dot0,
#                     q_ddot0,
#                     u0,
#                     underactuation_matrix=np.eye(3))

#     ilc_controller = ILC_SoftRobot(robot,
#                                    4,
#                                    3,
#                                    q_des,
#                                    q_dot_des,
#                                    q_ddot_des)
    
#     u = ilc_controller.initalGuessModelbased()
#     print(np.shape(u))
#     plt.figure(clear=True)
#     plt.xlabel(r'Time $[s]$')
#     plt.ylabel(r'Initial Guess')
#     for i_a in range(0, c.n_actuator):
#         plt.plot(time, u[:, i_a], color=c.color_list[i_a],\
#                 label='u_' + str(i_a + 1), linewidth=3)
#     plt.grid()
#     plt.legend(loc='best', shadow=True, fontsize=10)
#     plt.show()
    
