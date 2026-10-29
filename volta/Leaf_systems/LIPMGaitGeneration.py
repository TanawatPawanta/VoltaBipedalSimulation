import math

import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem


class LIPMGaitGeneration(LeafSystem):
    def __init__(self, g, z_com):
        super().__init__()
        self.g = g
        self.z_c = z_com
        self.x = []
        self.foot_holdx = 0.01
        self.foot_holdy = 0.0
        self.A = np.array([[0,1,0,0],
                           [self.g/self.z_c, 0, 0, 0],
                           [0, 0, 0, 1],
                           [0, 0, self.g/self.z_c, 0]])
        self.num_state_vars = 4
        self.time_step = 0.01
        self.curr_state = np.array([0,0,0,0])
        self.state = "Idle"
        state_index = self.DeclareContinuousState(self.num_state_vars)
        self.DeclareStateOutputPort("Y", state_index)
        dis_state_index = self.DeclareDiscreteState(4)  
        self.accel = np.zeros(2)
        self.DeclareStateOutputPort("dis_y", dis_state_index)  
        self.DeclareVectorOutputPort(
            "CoM acceleration",
            BasicVector(2),
            self.CalcOut
        )
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=0.01,  
            offset_sec=0.0,  
            update=self.Update)

    def Update(self, context, discrete_state):
        LIP_system_time = context.get_time()
        x   = context.get_continuous_state_vector().GetAtIndex(0)
        xd  = context.get_continuous_state_vector().GetAtIndex(1)
        y   = context.get_continuous_state_vector().GetAtIndex(2)
        yd  = context.get_continuous_state_vector().GetAtIndex(3)
        self.curr_state = np.array([x , xd, y, yd])
        for i in range(self.num_state_vars):
            discrete_state.get_mutable_vector().SetAtIndex(i, self.curr_state[i])

    def CalcOut(self, context, output):
        output.SetFromVector(self.accel)

    def CalOrbitalEnergy(self, x, xd):
        E = -((self.g/(2*self.z_c)) * x**2) + 0.5* xd**2
        return E
    
    def CalInitialCondition(self, step_time, y_0, step_length):
        Tc = math.sqrt(self.z_c/self.g)
        x_mid = 0
        x_0 = -step_length*0.5
        xd_mid = step_length/(2*Tc*np.sinh(step_time/(2*Tc)))
        E = self.CalOrbitalEnergy(x=x_mid, xd=xd_mid)
        xd_0 = math.sqrt(2*E + (self.g/self.z_c)*(x_0**2)) 
        # T_ss = 2*np.asinh(step_length/2.0/(Tc*xd_mid))*Tc
        T_ss = step_time
        tf = T_ss*0.5
        yd_0 = -(y_0/Tc)*np.sinh(tf/Tc)/np.cosh(tf/Tc)
        # digits = 2  # Number of decimal places
        # T_ss_rounded_up = math.ceil(T_ss * 10**digits) / 10**digits
        return [x_0, xd_0, y_0, yd_0, T_ss]
        
    def DoCalcTimeDerivatives(self, context, derivatives):
        x   = context.get_continuous_state_vector().GetAtIndex(0)
        xd  = context.get_continuous_state_vector().GetAtIndex(1)
        y   = context.get_continuous_state_vector().GetAtIndex(2)
        yd  = context.get_continuous_state_vector().GetAtIndex(3)
        LIP_system_time = context.get_time()
        self.t_past = LIP_system_time
        self.x.append(x)
        X = np.array([[x , xd, y, yd]]).T
        # state equation: X_dot = A*X 
        X_dot = np.matmul(self.A,X)
        self.accel[0]=X_dot[1]
        self.accel[1]=X_dot[3]
        for i in range(self.num_state_vars):
            derivatives.get_mutable_vector().SetAtIndex(i, X_dot[i])
