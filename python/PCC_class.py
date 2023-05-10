import numpy as np
import CONST as c
from testDynPCC_Classic import*
from scipy.integrate import solve_ivp, odeint, ode
from scipy.optimize import root
from copy import copy
from termcolor import colored 
import matplotlib.pyplot as plt
plt.rcParams['text.latex.preamble'] = ''.join([r'\usepackage{siunitx}', r'\usepackage{amsmath}'])
np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning) 

### ToDo: nonsense to define spinOnes and compute the all dynamics three times in this code.
### Action: just use spinOnce to determine the dynamics evolution.    
class DataRobot():
    def __init__(self, 
                q0,
                q0_dot,
                q0_ddot,
                action_eq,
                underactuation_matrix=None):
        
        self.x0 = np.concatenate([q0, q0_dot], dtype=np.float64)
        self.q0 = np.asanyarray(q0, dtype=np.float64)
        self.q0_dot = np.asanyarray(q0_dot, dtype=np.float64)
        self.q = np.asanyarray(q0, dtype=np.float64)
        self.q_dot = np.asanyarray(q0_dot, dtype=np.float64)
        self.q_ddot = np.asanyarray(q0_ddot, dtype=np.float64)
        self.action = np.asanyarray(action_eq, dtype=np.float64)
        self.x = np.concatenate([self.q, self.q_dot], dtype=np.float64)
        self.x_dot = np.concatenate([self.q_dot, self.q_ddot], dtype=np.float64)
        self.n = len(self.q)
        self.y = np.dot(c.C_out, self.q)
        self.y_dot = np.dot(c.C_out, self.q_dot)
        self.y_ddot = np.dot(c.C_out, self.q_ddot)
        
        self.M = np.asanyarray(inertiaMatrix(self.q, float(c.l0), float(c.I_xx), float(c.I_yy), float(c.I_zz), float(c.m_0)), dtype=np.float64).reshape((self.n,self.n))
        self.C = np.asanyarray(coriolisMatrix(self.q, self.q_dot, float(c.l0), float(c.I_xx), float(c.I_yy), float(c.I_zz), float(c.m_0)), dtype=np.float64).reshape((self.n,self.n))
        self.G = np.asanyarray(gravityVector(self.q, float(c.l0), float(c.m_0), float(c.g)), dtype=np.float64).reshape((-1,))
        self.K = np.asanyarray(stiffnessTorque(self.q, float(c.kappa_theta), float(c.kappa_phi), float(c.kappa_dL)), dtype=np.float64).reshape((-1,))
        self.D = np.asanyarray(dampingTorque(self.q_dot, float(c.d_ii)), dtype=np.float64).reshape((-1,))
        self.pos = np.asanyarray(forwardKine(self.q, float(c.l0)), dtype=np.float64).reshape((-1,))
        self.T = np.asanyarray(directKine(self.q, float(c.l0)), dtype=np.float64).reshape((4,4))
        self.iM = np.linalg.pinv(self.M)
        # self.iM = torch.linalg.pinv(self.M)
        self.Alin = np.asanyarray(tendonMatrixLinear(self.q,float(c.l0), float(c.r_head_1), float(c.theta_head_1)), dtype=np.float64).reshape((3,4))
        self.A = np.asanyarray(tendonMatrix(self.q, float(c.l0), float(c.r_head_1), float(c.theta_head_1)), dtype=np.float64).reshape((3,4))

        # self.action = np.asanyarray(np.dot(self.Alin, self.action), dtype=np.float64).reshape((-1,))
        self.action = np.asanyarray(np.dot(self.A, self.action), dtype=np.float64).reshape((-1,))
        
        self.q_ddot = np.asanyarray(-np.dot(self.iM, np.dot(self.C, self.q_dot) + self.G + self.D + self.K) \
                    + np.dot(self.iM, self.action), dtype=np.float64).reshape((-1,))
        
        if underactuation_matrix is None:
            self.underactuation_matrix = np.eye(self.n)
        else:
            self.underactuation_matrix = underactuation_matrix
        # print('=================================================================================================================')
    
    def solveSingularityStatic(self, x):
        # this regads only theta 
        n = int(len(x)/2)
        q = x[:n]
        q_dot = x[-n:]
        if abs(q[1]) < c.toll_state:
            # solve the zero-sign issue
            if np.sign(1) <= 0.5 and np.sign(1) >= -0.5:
                q[1] = c.toll_state
            else:
                q[1] = np.sign(1) * c.toll_state
        else:
            pass
        x_check = np.concatenate([q, q_dot], dtype=np.float64)
        return x_check
                    
    def solveSingularityStaticAll(self, x):
        # x = np.asanyarray(x, dtype=np.float64)
        n = int(len(x)/2)
        q = x[:n]
        q_dot = x[-n:]
        conta = 0
        for i in q:
            # this is not correct it should be done only for theta
            if abs(i) < c.toll_state:
                # solve the zero-sign issue
                if np.sign(i) <= 0.5 and np.sign(i) >= -0.5:
                    q[conta] = c.toll_state
                else:
                    q[conta] = np.sign(i) * c.toll_state
            else:
                pass
            conta = conta + 1
        x_check = np.concatenate([q, q_dot], dtype=np.float64)
        return x_check
        
    def spinOnes(self, q, q_dot, action):
        self.q = np.asanyarray(q, dtype=np.float64)
        self.q_dot = np.asanyarray(q_dot, dtype=np.float64)
        self.action = np.asanyarray(action, dtype=np.float64)
        self.y = np.dot(c.C_out, self.q)
        self.y_dot = np.dot(c.C_out, self.q_dot)
        # self.q_ddot = np.asanyarray(q_ddot, dtype=np.float64)
        self.x_dot = np.concatenate([self.q_dot, self.q_ddot], dtype=np.float64)
        self.x = np.concatenate([self.q, self.q_dot],  dtype=np.float64)
        self.M = np.asanyarray(inertiaMatrix(self.q, float(c.l0), float(c.I_xx), float(c.I_yy), float(c.I_zz), float(c.m_0)), dtype=np.float64).reshape((self.n,self.n))
        self.C = np.asanyarray(coriolisMatrix(self.q, self.q_dot, float(c.l0), float(c.I_xx), float(c.I_yy), float(c.I_zz), float(c.m_0)), dtype=np.float64).reshape((self.n,self.n))
        self.G = np.asanyarray(gravityVector(self.q, float(c.l0), float(c.m_0), float(c.g)), dtype=np.float64).reshape((-1,))
        self.K = np.asanyarray(stiffnessTorque(self.q, float(c.kappa_theta), float(c.kappa_phi), float(c.kappa_dL)), dtype=np.float64).reshape((-1,))
        self.D = np.asanyarray(dampingTorque(self.q_dot, float(c.d_ii)), dtype=np.float64).reshape((-1,))
        self.pos = np.asanyarray(forwardKine(self.q, float(c.l0)), dtype=np.float64).reshape((-1,))
        self.T = np.asanyarray(directKine(self.q, float(c.l0)), dtype=np.float64).reshape((4,4))
        self.iM = np.linalg.pinv(self.M)
        # self.iM = torch.linalg.pinv(self.M)
        self.Alin = np.asanyarray(tendonMatrixLinear(self.q,float(c.l0), float(c.r_head_1), float(c.theta_head_1)), dtype=np.float64).reshape((3,4))
        self.A = np.asanyarray(tendonMatrix(self.q, float(c.l0), float(c.r_head_1), float(c.theta_head_1)), dtype=np.float64).reshape((3,4))
    
        # self.action = np.asanyarray(np.dot(self.Alin, self.action), dtype=np.float64).reshape((-1,))
        self.action = np.asanyarray(np.dot(self.A, self.action), dtype=np.float64).reshape((-1,))
        
        self.q_ddot = np.asanyarray(-np.dot(self.iM, np.dot(self.C, self.q_dot) + self.G + self.D + self.K) + \
                       np.dot(self.iM, self.action), dtype=np.float64).reshape((-1,))
        self.y_ddot = np.dot(c.C_out, self.q_ddot)
        
    def eulerStep(self, x, u, dt):
        x = self.solveSingularityStatic(x)
        x_new = x + np.multiply(self.getNewStateExplicit(x, u), dt)
        self.x = np.asanyarray(x_new, dtype=np.float64)
        self.q = x_new[:self.n]
        self.q_dot = x_new[-self.n:]
        return x_new
    
    def adaptiveEulerStep(self, x, u, dt):
        # Chat GPT variabe step integrator, modifying Euler (my function above eulerStep)
        x = self.solveSingularityStatic(x)
        max_steps = 1000  # Maximum number of steps before giving up
        h = dt  # Initial step size
        t = 0.0  # Current time
        tol = 1e-3
        while True:
            # Try taking a step with step size h
            x1 = x + self.getNewStateExplicit(x, u) * h
            x2 = x + self.getNewStateExplicit(x1, u) * h
            # Use x2 as an estimate of the solution at t+h
            # and x1 as an estimate of the solution at t+2h
            # Then estimate the error as the difference between
            # these two estimates
            err = np.linalg.norm((x2 - x1) / h)
            # If the error is small enough, accept the step and update
            # the state and time
            if err <= tol:
                self.x = np.asanyarray(x2, dtype=np.float64)
                self.q = x2[:self.n]
                self.q_dot = x2[-self.n:]
                return x2
            # If the error is too large, reduce the step size and try again
            else:
                h = h / 2.0
                t += h
                # If we've taken too many steps, give up and raise an exception
                if t >= dt or h < 1e-10:
                    raise Exception("Adaptive Euler step failed: maximum number of steps exceeded.")
    
    def rk4Step(self, x, u, dt):
        x = self.solveSingularityStatic(x)
        k1 = self.getNewStateExplicit(x, u)
        k2 = self.getNewStateExplicit(x + np.multiply(k1, dt / 2), u) 
        k3 = self.getNewStateExplicit(x + np.multiply(k2, dt / 2), u) 
        k4 = self.getNewStateExplicit(x + np.multiply(k3, dt), u) 
        x_new = x + np.multiply(k1 + np.multiply(k2 + k3, 2) + k4, dt / 6)
        self.x = np.asanyarray(x_new, dtype=np.float64)
        self.q = x_new[:self.n]
        self.q_dot = x_new[-self.n:]
        return x_new
    
    def rkfStep(self, x, u, dt):
        # Chat GPT variabe step integrator, modifying Euler (my function above eulerStep)
        # RKF coefficients
        a = np.array([[0, 0, 0, 0, 0, 0],
                    [1/4, 0, 0, 0, 0, 0],
                    [3/32, 9/32, 0, 0, 0, 0],
                    [1932/2197, -7200/2197, 7296/2197, 0, 0, 0],
                    [439/216, -8, 3680/513, -845/4104, 0, 0],
                    [-8/27, 2, -3544/2565, 1859/4104, -11/40, 0]])
        b = np.array([16/135, 0, 6656/12825, 28561/56430, -9/50, 2/55])
        b_star = np.array([25/216, 0, 1408/2565, 2197/4104, -1/5, 0])
        c = np.array([0, 1/4, 3/8, 12/13, 1, 1/2])

        # Initializations
        t = 0
        x = self.solveSingularityStatic(x)
        k = np.zeros((6, len(x)))
        k[0] = self.getNewStateExplicit(x, u)
        err_toll = 1e-2 # 1e-10
        # Adaptive time stepping loop
        while t < dt:
            dt_left = dt - t
            if dt_left < err_toll:
                break
            dt_next = min(dt_left, 1e-3)
            for i in range(1, 6):
                k[i] = self.getNewStateExplicit(x + dt_next * np.dot(a[i,:i], k[:i]), u)
            x_err = dt_next * np.dot((b - b_star), k)
            err_norm = np.linalg.norm(x_err)
            if err_norm < err_toll:
                x_new = x + x_err
                t += dt_next
                # x = self.solveSingularityStatic(x_new)
                k[0] = self.getNewStateExplicit(x, u)
            else:
                dt_next = 0.8 * dt_left * (1 / err_norm) ** 0.25
        x = self.solveSingularityStatic(x_new)
        self.x = np.asanyarray(x, dtype=np.float64)
        self.q = x[:self.n]
        self.q_dot = x[-self.n:]
        return x
    
    def rk4AdaptiveStep(self, x, u, dt, tol=1e-6, max_dt=0.01, min_dt=1e-8):
        x = self.solveSingularityStatic(x)

        # Define the Butcher tableaus for the fourth-order and fifth-order methods
        c = np.array([0, 1/2, 1/2, 1])
        a = np.array([[0, 0, 0, 0],
                    [1/2, 0, 0, 0],
                    [0, 1/2, 0, 0],
                    [0, 0, 1, 0]])
        b4 = np.array([1/6, 1/3, 1/3, 1/6])
        b5 = np.array([1/10, 0, 3/10, 4/10, 1/10])
        b_diff = b5 - b4
        
        # Initialize step size and error
        t = 0
        error = np.inf
        dt_new = dt

        while t < dt:
            # Adjust step size if necessary
            if dt_new + t > dt:
                dt_new = dt - t

            # Take a step using the fourth-order and fifth-order methods
            k = np.zeros((6, self.n))
            k[0] = self.getNewStateExplicit(x, u)
            for i in range(4):
                k[i+1] = self.getNewStateExplicit(x + np.dot(a[i], k), u)
            k[5] = self.getNewStateExplicit(x + np.dot(a[4], k), u)
            x_new4 = x + np.dot(b4, k[:4])
            x_new5 = x + np.dot(b5, k)

            # Estimate error and adapt step size
            error = np.linalg.norm(b_diff.dot(k))
            if error <= tol:
                # Accept the fifth-order result and update state
                t += dt_new
                x = x_new5
                self.x = np.asanyarray(x, dtype=np.float64)
                self.q = x[:self.n]
                self.q_dot = x[-self.n:]
                dt_new *= min(5, 0.8*(tol/error)**(1/5))
            else:
                # Reject the fifth-order result and try again with smaller step size
                dt_new *= min(5, 0.8*(tol/error)**(1/5))
                if dt_new < min_dt:
                    raise Exception('Step size too small')
                elif dt_new > max_dt:
                    dt_new = max_dt

        return x

    def trapzOptiStep(self, x_n, x, u, u_n, dt):
        x = self.solveSingularityStatic(x)
        x_n = self.solveSingularityStatic(x_n)
        f = self.getNewStateExplicit(x, u)
        f_n = self.getNewStateExplicit(x_n, u_n)
        x_new = x + np.multiply(f + f_n, dt / 2) - x_n
        self.x = np.asanyarray(x_new, dtype=np.float64)
        self.q = x_new[:self.n]
        self.q_dot = x_new[-self.n:]
        return x_new

    def trapzStep(self, x, u, u_n, dt):
        x = self.solveSingularityStatic(x)
        x_0 = self.eulerStep(x, u, dt)
        x_n = root(self.trapzOptiStep, x_0, (x, u, u_n, dt))
        #  x_n = self.solveSingularityStatic(x_n)
        x_new = x_n.x
        self.x = x_new
        self.q = x_new[:self.n]
        self.q_dot = x_new[-self.n:]
        return x_new

    def getNewStateExplicit(self, x, action):
        self.action = action
        self.x = self.solveSingularityStatic(x)
        self.q = np.asanyarray(x[:self.n], dtype=np.float64)
        self.q_dot = np.asanyarray(x[-self.n:], dtype=np.float64)
        # it is what before I had colled computeWhatINeed function
        self.M = inertiaMatrix(self.q, float(c.l0), float(c.I_xx), float(c.I_yy), float(c.I_zz), float(c.m_0))
        self.C = coriolisMatrix(self.q, self.q_dot, float(c.l0), float(c.I_xx), float(c.I_yy), float(c.I_zz), float(c.m_0))
        self.G = gravityVector(self.q, float(c.l0), float(c.m_0), float(c.g))
        self.K = stiffnessTorque(self.q, float(c.kappa_theta), float(c.kappa_phi), float(c.kappa_dL))
        self.D = dampingTorque(self.q_dot, float(c.d_ii))
        self.pos = forwardKine(self.q, float(c.l0))
        self.T = directKine(self.q, float(c.l0))  
        self.iM = np.linalg.pinv(self.M)
        # self.iM = torch.linalg.pinv(self.M)
        cond_M = np.linalg.cond(self.iM)
        self.Alin = tendonMatrixLinear(self.q, float(c.l0), float(c.r_head_1), float(c.theta_head_1))
        self.A = tendonMatrix(self.q, float(c.l0), float(c.r_head_1), float(c.theta_head_1))
        
        if cond_M > 1e5 and c.PRINT_ONE:
            print('[INFO]: Robot Configuration: {}'.format(np.round(self.q, 4)))
            print('[INFO]: Condition number of M is bad ~ {}...'.format(np.round(cond_M), 2))
            c.PRINT_ONE = False
        else:
            pass
        
        self.M = np.asanyarray(self.M, dtype=np.float64)
        self.iM = np.asanyarray(self.iM, dtype=np.float64)
        Cq_dot = np.asanyarray(np.dot(self.C, self.q_dot), dtype=np.float64).reshape((-1,))        
        self.K = np.asanyarray(self.K, dtype=np.float64).reshape((-1,))
        self.D = np.asanyarray(self.D, dtype=np.float64).reshape((-1,))
        self.G = np.asanyarray(self.G, dtype=np.float64).reshape((-1,))
        self.pos = np.asanyarray(self.pos, dtype=np.float64).reshape((-1,))
        self.T = np.asanyarray(self.T, dtype=np.float64).reshape((4,4))
        self.Alin = np.asanyarray(self.Alin, dtype=np.float64).reshape((3,4))
        self.A = np.asanyarray(self.A, dtype=np.float64).reshape((3,4))
        
        # self.action = np.asanyarray(np.dot(self.Alin, self.action), dtype=np.float64).reshape((-1,))
        self.action = np.asanyarray(np.dot(self.A, self.action), dtype=np.float64).reshape((-1,))
        
        self.q_ddot = np.asanyarray(-np.dot(self.iM, Cq_dot + self.G + self.D + self.K) + \
                       np.dot(self.iM, self.action), dtype=np.float64).reshape((-1,))
        
        x_dot = np.concatenate([self.q_dot, self.q_ddot], dtype=np.float64)                             
        self.x_dot = x_dot
        self.x = np.concatenate([self.q, self.q_dot], dtype=np.float64)
        return x_dot
        
 