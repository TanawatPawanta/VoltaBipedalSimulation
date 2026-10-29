import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem

from volta.utils import DefinePlant


class MITController(LeafSystem):
    def __init__(self, kp = np.zeros(12), kd= np.zeros(12),dt = 0.01):
        super().__init__()
        self.dt = dt
        self.system_context = None
        self.desirer_joint_position = []   
        self.kp = kp
        self.kd = kd
        self.tol = 0.01 # [deg]
        self.q_err     = np.zeros(12)
        self.q_err_m1  = np.zeros(12)
        self.qd_err    = np.zeros(12)
        self.qd_err_m1 = np.zeros(12)
        self.qd_m1 = np.zeros(12)
        self.torque_u = np.zeros(12)
        self.vd_desire = np.zeros(12)
        self.qdd = np.zeros(12)
        self.plant, dump = DefinePlant(model_name="humanoid_half_body",
                             is_fixed=True,
                             fixed_frame="torso")
        self.plant_context = self.plant.CreateDefaultContext()
        # Declare an output port for torque commands
        self.joints_state_desire_input_port_0 = self.DeclareVectorInputPort(
            "joints state desire",
            BasicVector(24)
        )
        self.joints_state_feedback_input_port_1 = self.DeclareVectorInputPort(
            "joints state feedback",
            BasicVector(24)
        )
        self.DeclareVectorOutputPort(
            "torque command",
            BasicVector(12),
            self.CalcOutput_torque_command
        )
        self.DeclareVectorOutputPort(
            "q error",
            BasicVector(12),
            self.CalcOutput_q_Err
        )
        self.DeclareVectorOutputPort(
            "qd error",
            BasicVector(12),
            self.CalcOutput_qd_Err
        )
        self.DeclarePeriodicUnrestrictedUpdateEvent(period_sec=self.dt                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             ,
                                                    offset_sec=0.0,
                                                    update=self.Update)
    
    def CalcOutput_q_Err(self,context,output):
        output.SetFromVector(self.q_err)

    def CalcOutput_qd_Err(self,context,output):
        output.SetFromVector(self.qd_err) 

    def Update(self, context, state):
        joints_desire = self.joints_state_desire_input_port_0.Eval(context)
        joints_feedback = self.joints_state_feedback_input_port_1.Eval(context)
        self.qd = joints_feedback[12:]
        self.qdd = (self.qd - self.qd_m1)/self.dt
        # print("qdd: \n", self.qdd)
        self.qd_m1 = self.qd
        joints_err = joints_desire - joints_feedback
        self.q_err = joints_err[0:12]
        # print("q error: \n", joints_err)
        self.qd_err = joints_err[12:]
        self.q_err[abs(self.q_err) <= self.tol*np.pi/180.0] = 0.0
        self.qd_err[abs(self.qd_err) <= 0.00001*np.pi/180.0] = 0.0
        for i in range(12):
            self.torque_u[i] = self.kp[i%6]*self.q_err[i] + self.kd[i%6]*self.qd_err[i]
        # external_forces = MultibodyForces(self.plant)
        # self.plant.SetPositions(self.plant_context, joints_feedback[0:12])
        # self.plant.SetVelocities(self.plant_context, self.qd)
        # self.torque_u = -1.0*self.plant.CalcInverseDynamics(self.plant_context, self.qdd + self.vd_desire, external_forces)
        # print("torque_u: \n",self.torque_u)
    def CalcOutput_torque_command(self, context, output):
        torque_output = self.torque_u.copy()
        torque_output[0] = self.torque_u[2]
        torque_output[2] = self.torque_u[0]
        torque_output[6] = self.torque_u[8]
        torque_output[8] = self.torque_u[6]
        # output.SetFromVector(np.array([0.0,0.0,0.0,0.0,0.0,0.1,
        #                                0.0,0.0,0.0,0.0,0.0,0.1]))
        # print("torque_output: \n", torque_output)
        output.SetFromVector(torque_output)
        # output.SetFromVector(np.zeros(12)