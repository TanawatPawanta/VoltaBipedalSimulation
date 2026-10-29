import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem

from volta.estimation.kalman_filter import KalmanFilter, block_diag
from volta.robot.states import States


class StateEstimator(LeafSystem):
    """Kalman filter on the LIP model that smooths the measured CoM and ZMP.

    Inputs:  0 CoM state [12], 1 measured ZMP [3], 2 MPC input u (ZMP velocity) [3]
    Outputs: 0 filtered CoM state [12], 1 filtered ZMP [3]
    """
    def __init__(self, dt, initial, mpc):
        super().__init__()
        self.dt = dt
        self.mpc = mpc
        self.initial = initial
        A = np.identity(3) + self.dt * self.mpc.A_lip
        B = self.dt * self.mpc.B_lip
        d = np.zeros(9)
        d[7] = - self.dt * 9.81
        H = np.identity(3)
        # H[1,1] = 0.0
        # H[2,2] = 0.0
        Q = block_diag(1., 1.0, 1.)
        # R = block_diag(1e1, 1e2, 1e4)
        R = block_diag(1e1, 1e1, 1e4)
        P = np.identity(3)
        x = np.array([self.initial['com']['pos'][0], self.initial['com']['vel'][0], self.initial['zmp']['pos'][0], \
                      self.initial['com']['pos'][1], self.initial['com']['vel'][1], self.initial['zmp']['pos'][1], \
                      self.initial['com']['pos'][2], self.initial['com']['vel'][2], self.initial['zmp']['pos'][2]])
        self.noisy_CoM_state = States(name="noisy com state")
        self.CoM_filtered = States(name="filtered com state")
        self.CoM_filtered.position = np.array([0,0,0.35])
        self.ZMP_filtered = self.initial['zmp']['pos']
        self.kf = KalmanFilter(block_diag(A, A, A), \
                                      block_diag(B, B, B), \
                                      d, \
                                      block_diag(H, H, H), \
                                      block_diag(Q, Q, Q), \
                                      block_diag(R, R, R), \
                                      block_diag(P, P, P), \
                                      x)
        # Declare an output port 
        self.input_port_0 = self.DeclareVectorInputPort("CoM state",BasicVector(12))
        self.input_port_1 = self.DeclareVectorInputPort("ZMP",BasicVector(3))
        self.input_port_2 = self.DeclareVectorInputPort("U",BasicVector(3))
        self.DeclareVectorOutputPort("Filtered CoM states", BasicVector(12), self.CalcOutput_CoM)
        self.DeclareVectorOutputPort("Filtered ZMP ",BasicVector(3),self.CalcOutput_ZMP)
        # Declare periodic events
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=self.dt,  
            offset_sec=0.0,  
            update=self.DiscreteUpdate)
        
    def DiscreteUpdate(self, context, discrete_state):
        CoM_state = self.input_port_0.Eval(context)
        self.noisy_CoM_state.orientation = CoM_state[0:3]
        self.noisy_CoM_state.position = CoM_state[3:6]
        self.noisy_CoM_state.angular_velocity = CoM_state[6:9]
        self.noisy_CoM_state.linear_velocity = CoM_state[9:12]
        zmp = self.input_port_1.Eval(context)   
        u = self.input_port_2.Eval(context)
        # print("--"*50)
        # print("CoM ori",self.noisy_CoM_state.orientation)
        self.kf.predict(u)
        x_flt, _ = self.kf.update(np.array([self.noisy_CoM_state.position[0], self.noisy_CoM_state.linear_velocity[0], zmp[0], \
                                            self.noisy_CoM_state.position[1], self.noisy_CoM_state.linear_velocity[1], zmp[1], \
                                            self.noisy_CoM_state.position[2], self.noisy_CoM_state.linear_velocity[2], zmp[2]]))
        self.CoM_filtered.position[0]           = x_flt[0]
        self.CoM_filtered.linear_velocity[0]    = x_flt[1]
        self.ZMP_filtered[0]                    = x_flt[2]
        self.CoM_filtered.position[1]           = x_flt[3]
        self.CoM_filtered.linear_velocity[1]    = x_flt[4]
        self.ZMP_filtered[1]                    = x_flt[5]
        self.CoM_filtered.position[2]           = x_flt[6]
        self.CoM_filtered.linear_velocity[2]    = x_flt[7]
        self.ZMP_filtered[2]                    = 0.0
        self.CoM_filtered.orientation = self.noisy_CoM_state.orientation
        self.CoM_filtered.angular_velocity = self.noisy_CoM_state.angular_velocity

    def CalcOutput_CoM(self, context, output):
        out = self.CoM_filtered.GetStateVector(acc=False)
        # print("out",out)
        output.SetFromVector(out)

    def CalcOutput_ZMP(self, context, output):
        out = self.ZMP_filtered
        output.SetFromVector(out)