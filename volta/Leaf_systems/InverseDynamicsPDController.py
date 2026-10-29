import numpy as np
from pydrake.multibody.math import SpatialForce
from pydrake.multibody.tree import MultibodyForces
from pydrake.systems.framework import BasicVector, LeafSystem

from volta.robot.kinematics import CalcContactPointPositionsInCoM
from volta.utils import IsInsideRectangle


class InverseDynamicsPDController(LeafSystem):
    def __init__(self, plant,
                 kp=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]), 
                 kd=np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]),
                 initial_joint_position = np.zeros(12),
                 dt=0.01):
        super().__init__()
        self.dt = dt
        self.kp = np.concatenate((kp, kp))
        self.kd = np.concatenate((kd, kd))
        self.plant = plant
        self.q = initial_joint_position
        self.tau_g = np.zeros(12)
        self.DefineVariables()
        self.DefineIOPort()

        # Declare periodic events 
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=self.dt,  
            offset_sec=0.0,  
            update=self.DiscreteUpdate)
        
    def DefineVariables(self):
        self.total_torque = np.zeros(12)
        self.plant_context = self.plant.CreateDefaultContext()
        self.generalize_velocity_m1 = np.zeros(self.plant.num_velocities())
        self.qd_desire_m1 = np.zeros(self.plant.num_actuated_dofs())
        self.selection_matrix = np.concatenate((np.zeros((12, 6)), np.eye(12)), axis=1)
        self.contact_frames = [
            "L_foot_contact_FL", "L_foot_contact_FR", "L_foot_contact_RL", "L_foot_contact_RR",
            "R_foot_contact_FL", "R_foot_contact_FR", "R_foot_contact_RL", "R_foot_contact_RR"
        ]
        self.plant.SetPositions(self.plant_context,np.concatenate(([1,0,0,0,0,0,0],self.q)))
        self.CaclForceDoitributionMatrix(active_contact_frames=self.contact_frames)
        self.leg_state = "double_stance" # L_stance or R_stance or double_stance
    
    def DefineIOPort(self):
        self.joints_feedback_port = self.DeclareVectorInputPort(
            "Joints feedback(q,qd)",
            BasicVector(37)  
        )
        self.joint_desire_port = self.DeclareVectorInputPort(
            "Joints desire(q,qd)",
            BasicVector(24)  
        )
        self.DeclareVectorOutputPort(
            "Actuation torques",
            BasicVector(12),
            self.CalcOutput_actuation_torques
        )

    def DiscreteUpdate(self, context, discrete_state):
        self.robot_states = self.joints_feedback_port.Eval(context)
        self.desire = self.joint_desire_port.Eval(context)
        # self.GRF = self.GRF_port.Eval(context)
        # self.ZMP = self.ZMP_port.Eval(context)
        print("time step:", context.get_time())
        self.UpdateVariables()
        known_vdot = np.zeros(18)
        f_u = np.zeros(3)
        gravitational_vector  = np.array([0, 0, -9.81])
        m = self.plant.CalcTotalMass(self.plant_context)
        desired_GAF = f_u + m*gravitational_vector
        desired_CoP_position = np.array([0.0, 0.0])  # same as projected CoM position to the ground
        contact_forces_Z = self.CalcContactForcesDistribution(desired_GAF=desired_GAF,
                                                              desired_CoP_position=desired_CoP_position)
        self.CalcWrenchAtContactPoints(-1.0*contact_forces_Z)
        torque_g = self.plant.CalcGravityGeneralizedForces(context=self.plant_context)
        torque_id = self.CalcTorqueFromInverseDynamics(known_vdot=known_vdot,
                                                       contact_forces=-1.0*contact_forces_Z)
        print("torque_id: ", self.selection_matrix @torque_id)
        self.tau_g = self.selection_matrix @ torque_g
       
        torque_u = self.CalcTorqueFromPDController() 
        self.total_torque = (self.selection_matrix@(torque_id - torque_g))  + torque_u
        
    def CalcWrenchAtContactPoints(self,contact_forces_Z):
        momemt_L = np.zeros(3)
        momemt_R = np.zeros(3)
        fz_L = 0.0
        fz_R = 0.0
        contect_position = np.array([[0.08073, 0.02475, 0.0],
                                    [0.08073, -0.02475, 0.0],
                                    [-0.0907, 0.02475, 0.0],
                                    [-0.0907, -0.02475, 0.0]])
        for i in range(len(contact_forces_Z)):
            force = np.array([0.0, 0.0, contact_forces_Z[i]])
            moment = np.cross(contect_position[i%4,:], force)
            if i < 4:
                momemt_L += moment
                fz_L += contact_forces_Z[i]
            else:
                momemt_R += moment
                fz_R += contact_forces_Z[i]
        print("fz L foot: ", fz_L)
        print("fz R foot: ", fz_R)
        print("moment L foot: ", momemt_L)
        print("moment R foot: ", momemt_R)

    def UpdateVariables(self):
        # Feedback state update
        self.generalize_position = self.robot_states[0:19]
        self.q = self.generalize_position[7:]
        self.generalize_velocity = self.robot_states[19:37]
        self.qd = self.generalize_velocity[6:]
        self.generalize_acceleration = (self.generalize_velocity - self.generalize_velocity_m1)/self.dt
        self.generalize_velocity_m1 = self.generalize_velocity
        # Desired state update
        self.q_desire = self.desire[:12]
        self.qd_desire = self.desire[12:]
        self.qdd_desire = (self.qd_desire -  self.qd_desire_m1)/self.dt
        self.qd_desire_m1 = self.qd_desire
        self.plant.SetPositions(self.plant_context,self.generalize_position)
        self.plant.SetVelocities(self.plant_context,np.zeros(self.plant.num_velocities()))
        self.UpdateLegState(generalized_position=self.generalize_position)
        print("leg state: ", self.leg_state)
        match self.leg_state:
            case "L_stance":
                active_contact_frames = self.contact_frames[0:4]
            case "R_stance":
                active_contact_frames = self.contact_frames[4:8]
            case "double_stance":
                active_contact_frames = self.contact_frames
        self.CaclForceDoitributionMatrix(active_contact_frames=active_contact_frames)
    
    def CaclForceDoitributionMatrix(self,active_contact_frames):
        self.contact_positions = CalcContactPointPositionsInCoM(plant=self.plant, 
                                                                context=self.plant_context, 
                                                                contact_frame_names=active_contact_frames)
        force_distribution_z = []
        for i in range(self.contact_positions.shape[0]):
            force_distribution_z.append([self.contact_positions[i,0],
                                        self.contact_positions[i,1],
                                        1.0])
        self.force_distribution_matrix_Z = np.array(force_distribution_z).T
        self.force_distribution_matrix_X = np.concatenate((np.array(self.force_distribution_matrix_Z[0,:]).reshape(1, -1),
                                                    np.ones((1, self.force_distribution_matrix_Z.shape[1]))), axis=0)
        self.force_distribution_matrix_Y = np.concatenate((np.array(self.force_distribution_matrix_Z[1,:]).reshape(1, -1),
                                                    np.ones((1, self.force_distribution_matrix_Z.shape[1]))), axis=0)
    
    def UpdateLegState(self, generalized_position):
        self.plant.SetPositions(self.plant_context, generalized_position)
        W_P_com = self.plant.CalcCenterOfMassPositionInWorld(self.plant_context)
        # print("W_P_com : ", W_P_com[0:2])
        torso = self.plant.GetFrameByName("torso")
        W_P_torso = torso.CalcPoseInWorld(self.plant_context).translation()
        L_foot_contact = self.plant.GetFrameByName("L_foot_contact")
        L_P_com = L_foot_contact.CalcPoseInWorld(self.plant_context).inverse().multiply(W_P_com)
        # L_P_torso = L_foot_contact.CalcPoseInWorld(self.plant_context).inverse().multiply(W_P_torso)
        R_foot_contact = self.plant.GetFrameByName("R_foot_contact")
        R_P_com = R_foot_contact.CalcPoseInWorld(self.plant_context).inverse().multiply(W_P_com)
        # R_P_torso = R_foot_contact.CalcPoseInWorld(self.plant_context).inverse().multiply(W_P_torso)
        offset = 2 # [mm]
        L_foot_rect = np.array([-110+offset,-49+offset,
                                110-offset, 49-offset]) * 1e-3
        R_foot_rect = np.array([-110+offset,-49+offset,
                                110-offset, 49-offset]) * 1e-3
        if IsInsideRectangle(point=L_P_com[0:2], rect=L_foot_rect):
            self.leg_state = "L_stance"
        elif IsInsideRectangle(point=R_P_com[0:2], rect=R_foot_rect):
            self.leg_state = "R_stance"
        else:
            self.leg_state = "double_stance"

    def CalcTorqueFromPDController(self):
        q_error = self.q_desire - self.q
        qd_error = self.qd_desire - self.qd
        return self.kp * q_error + self.kd * qd_error
    
    def CalcContactForcesDistribution(self,desired_GAF,desired_CoP_position):
        contact_forces_Z = np.linalg.pinv(self.force_distribution_matrix_Z) @ np.array([desired_CoP_position[0], desired_CoP_position[1], 1.0]) * desired_GAF[2]
        if self.leg_state == "L_stance":
            contact_forces_Z = np.concatenate((contact_forces_Z, np.zeros(4)), axis=0)
        elif self.leg_state == "R_stance":
            contact_forces_Z = np.concatenate((np.zeros(4), contact_forces_Z), axis=0)
        return contact_forces_Z

    def CalcTorqueFromInverseDynamics(self,known_vdot,contact_forces):
        external_forces = MultibodyForces(self.plant)
        for i in range(len(self.contact_frames)):
            contact_frame = self.plant.GetBodyByName(self.contact_frames[i])
            F_Bo_W = SpatialForce(tau=np.zeros(3), f=np.array([0,0,contact_forces[i]]))
            contact_frame.AddInForceInWorld(self.plant_context, F_Bo_W, external_forces)
        # known_vdot = np.zeros(18)

        torque_id = self.plant.CalcInverseDynamics(context=self.plant_context,
                                    known_vdot=known_vdot,
                                    external_forces=external_forces)
        return torque_id
    
    def CalcOutput_actuation_torques(self, context, output):
        
        torque_output = self.total_torque.copy()
    #     torque_output = np.array([ 0.4143, -1.1702, -0.6165,  1.1012, -0.0006,  0.0006, -1.6558,
    #    -4.6686, -2.4577, -4.41  ,  0.0014, -0.0003]) - self.tau_g
    #     torque_output = np.array([ 0.4352, -1.1742, -0.6136,  1.1569, -0.0006, -0.0613, -1.6349,
    #    -4.5613, -2.3961, -4.3543,  0.0014, -0.061 ]) - self.tau_g
        torque_output[0] = self.total_torque[2]
        torque_output[2] = self.total_torque[0]
        torque_output[6] = self.total_torque[8]
        torque_output[8] = self.total_torque[6]
        output.SetFromVector(torque_output)

