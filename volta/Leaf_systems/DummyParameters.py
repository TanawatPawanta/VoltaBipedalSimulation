import numpy as np
from pydrake.all import BasicVector, LeafSystem

from volta.robot.states import WalkStates


class DummyParameters(LeafSystem):
    def __init__(self,q_init_data, sw_leg_q, st_leg_q, sw_leg_qd, st_leg_qd):
        super().__init__()  

        self.DeclareNumericParameter(BasicVector([0]))
        self.DeclareNumericParameter(BasicVector([0]))
        self.q_init_L  = q_init_data['arr_0']
        self.q_init_R  = q_init_data['arr_1']
        self.qd_init_L = q_init_data['arr_2']
        self.qd_init_R = q_init_data['arr_3']         
        self.sw_leg_q = sw_leg_q
        self.st_leg_q = st_leg_q
        self.sw_leg_qd = sw_leg_qd
        self.st_leg_qd = st_leg_qd
        self.DeclareNumericParameter(BasicVector([0.0, 0.0, 0.0, 0.6, 0.0,0.0]))
        self.DeclareNumericParameter(BasicVector([1.3]))
        self.DeclareNumericParameter(BasicVector([0]))
        self.DeclareNumericParameter(BasicVector([1]))
        self.dt = 0.01
        self.ind = 0
        self.L_leg_q = self.q_init_L[:,self.ind]
        self.R_leg_q = self.q_init_R[:,self.ind]

        self.L_leg_qd = self.qd_init_L[:,self.ind]
        self.R_leg_qd= self.qd_init_R[:,self.ind]

        q = np.concatenate((np.array([self.L_leg_q]), np.array([self.R_leg_q])),axis=1)
        qd = np.concatenate((np.array([self.L_leg_qd]), np.array([self.R_leg_qd])),axis=1)
        self.leg_state =  np.concatenate((q,qd),axis=1)

        self.L_stance = False
        self.walk_state = WalkStates.init

        self.DeclareVectorOutputPort(name="L_leg_q",
                                     size=6,
                                     calc=self.Output_qL)
        self.DeclareVectorOutputPort(name="R_leg_q",
                                     size=6,
                                     calc=self.Output_qR)
        self.DeclareVectorOutputPort(name="L_leg_qd",
                                     size=6,
                                     calc=self.Output_qdL)
        self.DeclareVectorOutputPort(name="R_leg_qd",
                                     size=6,
                                     calc=self.Output_qdR)
        self.DeclareVectorOutputPort(name="joints_states_reference",
                                     size=24,
                                     calc=self.Output_ref)
        self.DeclareVectorOutputPort(name="is_L_stance",
                                     size=2,
                                     calc=self.Output_is_L_stance)
        self.DeclarePeriodicUnrestrictedUpdateEvent(period_sec=self.dt                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   ,
                                                    offset_sec=0.0,
                                                    update=self.Update)
    
    def Update(self, context, state):
        match self.walk_state:
            case WalkStates.stand:
                pass
            case WalkStates.init:
                if self.ind <= self.q_init_L.shape[1] - 1:
                    self.L_leg_q = self.q_init_L[:,self.ind]
                    self.R_leg_q = self.q_init_R[:,self.ind]

                    self.L_leg_qd = self.qd_init_L[:,self.ind]
                    self.R_leg_qd = self.qd_init_R[:,self.ind]

                elif self.ind > 400:
                    self.walk_state = WalkStates.walk
                    self.ind = 0
                self.ind += 1
            case WalkStates.walk:
                if self.ind > self.sw_leg_q.shape[1] - 1:
                    self.ind = 0
                    for i in [1,5]:
                        self.st_leg_q[i,:] = self.st_leg_q[i,:]*-1.0
                        self.sw_leg_q[i,:] = self.sw_leg_q[i,:]*-1.0

                        self.st_leg_qd[i,:] = self.st_leg_qd[i,:]*-1.0
                        self.sw_leg_qd[i,:] = self.sw_leg_qd[i,:]*-1.0

                    self.L_stance = not self.L_stance

                if self.L_stance:
                    self.L_leg_q = self.st_leg_q[:,self.ind]
                    self.R_leg_q = self.sw_leg_q[:,self.ind]

                    self.L_leg_qd = self.st_leg_qd[:,self.ind]
                    self.R_leg_qd = self.sw_leg_qd[:,self.ind]

                else:
                    self.L_leg_q = self.sw_leg_q[:,self.ind]
                    self.R_leg_q = self.st_leg_q[:,self.ind]

                    self.L_leg_qd = self.sw_leg_qd[:,self.ind]
                    self.R_leg_qd = self.st_leg_qd[:,self.ind]                    
                self.ind += 1

        q = np.concatenate((np.array([self.L_leg_q]), np.array([self.R_leg_q])),axis=1)
        qd = np.concatenate((np.array([self.L_leg_qd]), np.array([self.R_leg_qd])),axis=1)
        self.leg_state =  np.concatenate((q,qd),axis=1)
                
    def Output_qL(self, context, output):
        output.SetFromVector(self.L_leg_q)

    def Output_qR(self, context, output):
        output.SetFromVector(self.R_leg_q)

    def Output_qdL(self, context, output):
        output.SetFromVector(self.L_leg_qd)

    def Output_qdR(self, context, output):
        output.SetFromVector(self.R_leg_qd)

    def Output_ref(self, context, output):
        output.SetFromVector(self.leg_state[0])
    
    def Output_is_L_stance(self, context, output):
        output.SetFromVector([self.walk_state.value,False])
