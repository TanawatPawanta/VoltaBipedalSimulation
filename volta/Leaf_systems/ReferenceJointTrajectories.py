import numpy as np
from pydrake.all import BasicVector, LeafSystem

from volta.planning.offline_planning import LIPJointTrajectory
from volta.robot.states import WalkStates


class ReferenceJointTrajectories(LeafSystem):
    def __init__(self,model_series, 
                 model_name,
                 step_length=0.1, 
                 step_height=0.05, 
                 step_time=5.0, 
                 z_com=0.7,
                 desire_torso_high=0.4, 
                 y_0=0.125):
        super().__init__()  
        self.GaitPlanner = LIPJointTrajectory(model_series=model_series,
                                 model_name=model_name,
                                 step_length=step_length,
                                 step_height=step_height,
                                 step_time=step_time,
                                 z_com=z_com,
                                 desire_torso_high=desire_torso_high,
                                 y_0=y_0)
        self.GaitPlanner.CalcTrajectory()
        self.q_init_L  = self.GaitPlanner.L_leg_init_joints_positions.T
        self.q_init_R  = self.GaitPlanner.R_leg_init_joints_positions.T
        self.qd_init_L = self.GaitPlanner.L_leg_init_joints_velocities.T
        self.qd_init_R = self.GaitPlanner.R_leg_init_joints_velocities.T

        self.sw_leg_q = self.GaitPlanner.L_leg_joints_positions.T
        self.st_leg_q = self.GaitPlanner.R_leg_joints_positions.T
        self.sw_leg_qd = self.GaitPlanner.L_leg_joints_velocities.T
        self.st_leg_qd = self.GaitPlanner.R_leg_joints_velocities.T
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
        
        self.joints_state_feedback_input_port_1 = self.DeclareVectorInputPort(
            "joints state feedback",
            BasicVector(24)
        )
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

                else:
                    pass
                    self.walk_state = WalkStates.walk
                    self.ind = 0
                self.ind += 1
            case WalkStates.walk:
                if self.ind > self.sw_leg_q.shape[1] - 1:
                    self.ind = 0
                    for i in range(6):
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
        # self.leg_state =  np.zeros((1,24))
                
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