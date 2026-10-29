import numpy as np
from pydrake.multibody.math import SpatialForce
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import MultibodyForces
from pydrake.systems.framework import BasicVector, LeafSystem

from volta.utils import DefinePlant, IsInsideRectangle


class GravityCompensation(LeafSystem):
    def __init__(self, model_series:str="Volta", dt:float=0.01):
        super().__init__()
        self.model_series = model_series
        self.dt = dt
        self.torque = np.zeros(12)
        self.walk_states = "double_stance" # double stance, L_stance, R_stance
        self.PreparePlant()
        # Declare an output port 
        self.input_port_0 = self.DeclareVectorInputPort(
            "Robot states",
            BasicVector(37)
            # BasicVector(26)   
        )
        self.input_port_1 = self.DeclareVectorInputPort(
            "Ground reaction force",
            BasicVector(6)  
        )
        self.DeclareVectorOutputPort(
            "torque_g",
            BasicVector(12),
            self.CalcOutput_torque_g
        ) 
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=self.dt,  
            offset_sec=0.0,  
            update=self.DiscreteUpdate)
    
    def PreparePlant(self):
        if "Hanuman" in self.model_series:
            self.plant_half, dump = DefinePlant(model_name="humanoid_half_body",
                                is_fixed=False,
                                fixed_frame="torso")
            self.plant_half_context = self.plant_half.CreateDefaultContext()       

            self.plant_R_fixed, dump = DefinePlant(model_name="humanoid_R_fixed",
                                is_fixed=True,
                                fixed_frame="R_foot_contact")
            self.plant_R_fixed_context = self.plant_R_fixed.CreateDefaultContext()

            self.plant_L_fixed, dump = DefinePlant(model_name="humanoid_L_fixed",
                                is_fixed=True,
                                fixed_frame="L_foot_contact")
            self.plant_L_fixed_context = self.plant_L_fixed.CreateDefaultContext()

            self.plant_double_stance_R, dump = DefinePlant(model_name="humanoid_only_leg",
                                is_fixed=True,
                                fixed_frame="L_foot_contact")
            self.plant_double_stance_R_context = self.plant_double_stance_R.CreateDefaultContext()
            self.plant_double_stance_L, dump = DefinePlant(model_name="humanoid_only_leg",
                                is_fixed=True,
                                fixed_frame="L_foot_contact")
            self.plant_double_stance_L_context = self.plant_double_stance_L.CreateDefaultContext()
        elif "Volta" in self.model_series:
            self.plant_half, dump = DefinePlant(model_name="Volta_rev4",
                                is_fixed=False,
                                fixed_frame="torso")
            self.plant_half_context = self.plant_half.CreateDefaultContext()       

            self.plant_R_fixed, dump = DefinePlant(model_name="Volta_rev4_R_fixed",
                                is_fixed=True,
                                fixed_frame="R_foot_contact")
            self.plant_R_fixed_context = self.plant_R_fixed.CreateDefaultContext()

            self.plant_L_fixed, dump = DefinePlant(model_name="Volta_rev4_L_fixed",
                                is_fixed=True,
                                fixed_frame="L_foot_contact")
            self.plant_L_fixed_context = self.plant_L_fixed.CreateDefaultContext()

            self.plant_double_stance_R, dump = DefinePlant(model_name="Volta_rev4_only_R",
                                is_fixed=True,
                                fixed_frame="R_foot_contact")
            self.plant_double_stance_R_context = self.plant_double_stance_R.CreateDefaultContext()
            self.plant_double_stance_L, dump = DefinePlant(model_name="Volta_rev4_only_L",
                                is_fixed=True,
                                fixed_frame="L_foot_contact")
            self.plant_double_stance_L_context = self.plant_double_stance_L.CreateDefaultContext()

    def DiscreteUpdate(self, context, discrete_state):
        robot_states = self.input_port_0.Eval(context)
        foot_contact_results = self.input_port_1.Eval(context)
        q = robot_states[0:19]
        q_L = q[7:13]
        q_R = q[13:19]
        # q_L = q[:6]
        # q_R = q[6:12]
        qd = np.zeros(12)
        self.UpdateWalkState(q=q, context=context)
        # print(context.get_time()," : ",self.walk_states)
        print("qL :", q_L)
        print("qR :", q_R)
        match self.walk_states:
            case("double_stance"):
                self.plant_double_stance_L.SetPositions(self.plant_double_stance_L_context,
                                                    -np.array(q_L)[::-1])
                tau_g_L = self.plant_double_stance_L.CalcGravityGeneralizedForces(context=self.plant_double_stance_L_context)
                tau_g_L = [1.0,1.0,1.0,1.0,1.0,0.7]*np.array(tau_g_L)[::-1]
                self.plant_double_stance_R.SetPositions(self.plant_double_stance_R_context,
                                                    -np.array(q_R)[::-1])
                tau_g_R = self.plant_double_stance_R.CalcGravityGeneralizedForces(context=self.plant_double_stance_R_context)
                tau_g_R = [1.0,1.0,1.0,1.0,1.0,0.7]*np.array(tau_g_R)[::-1]
            case("R_stance"):
                self.plant_R_fixed.SetPositions(self.plant_R_fixed_context, 
                                                np.concatenate((-q_R[::-1],q_L)))
                tau_g = 1*self.plant_R_fixed.CalcGravityGeneralizedForces(context=self.plant_R_fixed_context)
                tau_g_L = -1.0*tau_g[6:]
                tau_g_R = 1.0*tau_g[0:6][::-1]
            case("L_stance"):
                self.plant_L_fixed.SetPositions(self.plant_L_fixed_context, 
                                                np.concatenate((-q_L[::-1],q_R)))
                tau_g = 1*self.plant_L_fixed.CalcGravityGeneralizedForces(context=self.plant_L_fixed_context)
                tau_g_L = 1.0*tau_g[0:6][::-1]
                tau_g_R = -1.0*tau_g[6:]        
        print("tau_g_L :", tau_g_L)
        print("tau_g_R :", tau_g_R)
        print("=" * 50)
        self.torque = np.concatenate((tau_g_L,tau_g_R))

    # def DiscreteUpdate(self, context, discrete_state):
    #     robot_states = self.input_port_0.Eval(context)
    #     foot_contact_results = self.input_port_1.Eval(context)
    #     q = robot_states[0:19]
    #     q_L = q[7:13]
    #     q_R = q[13:19]
    #     # q_L = q[:6]
    #     # q_R = q[6:12]
    #     qd = np.zeros(12)
    #     q = np.array([np.concatenate((q_L, q_R),axis=0)])
    #     init_orien = np.asarray([0, 0, 0])
    #     body_quanternion_xyzw = np.array([R.from_euler('xyz', init_orien*np.pi/180.0).as_quat()])[0]
    #     body_quanternion_wxyz = [body_quanternion_xyzw[3], body_quanternion_xyzw[0], body_quanternion_xyzw[1], body_quanternion_xyzw[2]]
    #     body_pose = np.concatenate((np.array([body_quanternion_wxyz]), np.array([[0,0,0]])), axis=1)

    #     generalize_positions = np.concatenate((body_pose, q),axis=1)[0] 

    #     plant_id, robot_id_idx = DefinePlant(model_name="Volta_rev4",
    #                                 is_fixed=False, 
    #                                 fixed_frame="torso")  
    #     plant_id_context = plant_id.CreateDefaultContext()
    #     plant_id.SetPositions(plant_id_context,generalize_positions)
    #     W_P_com = plant_id.CalcCenterOfMassPositionInWorld(plant_id_context)
    #     L_foot_contact = plant_id.GetFrameByName("L_foot_contact") 
    #     W_T_L = L_foot_contact.CalcPoseInWorld(plant_id_context)  
    #     L_P_com = W_T_L.inverse().multiply(W_P_com)
    #     R_foot_contact = plant_id.GetFrameByName("R_foot_contact") 
    #     W_T_R = R_foot_contact.CalcPoseInWorld(plant_id_context)  
    #     R_P_com = W_T_R.inverse().multiply(W_P_com)
    #     m = plant_id.CalcTotalMass(plant_id_context)
    #     print("Total mass of the robot: ", m)
    #     g = 9.81
    #     force_L = np.array([0, 0.0, 0.5*m*g])
    #     force_R = np.array([0, 0.0, 0.5*m*g])
    #     # force_L = np.array([0, 0.0, 46.0])
    #     # force_R = np.array([0, 0.0, 46.0])
    #     force_mg = np.array([0, 0.0, -1.0*m*g])

    #     contact_frame_L = plant_id.GetBodyByName("L_foot_contact") 
    #     contact_frame_R = plant_id.GetBodyByName("R_foot_contact") 
    #     torso_frame = plant_id.GetBodyByName("torso")
    #     ZMP_L_LOCATION = np.array([-0.000, -0.000, 0])
    #     ZMP_R_LOCATION = np.array([-0.0,  0.0, 0])
    #     # ZMP_L_LOCATION = np.array([-0.0045,  0.02110889, 0])
    #     # ZMP_R_LOCATION = np.array([-0.0073, -0.02196527, 0])
    #     # ZMP_L_LOCATION = np.array([0.015,  -0.02, 0])
    #     # ZMP_R_LOCATION = np.array([0.015, 0.02196527, 0])
    #     torque_L = np.cross(ZMP_L_LOCATION,force_L)
    #     torque_R = np.cross(ZMP_R_LOCATION,force_R)
    #     print("Torque for left foot contact :\n",torque_L)
    #     print("Torque for right foot contact :\n",torque_R)
    #     # torque = np.zeros(3)
    #     F_Bo_W_L = SpatialForce(tau=torque_L, f=force_L)
    #     F_Bo_W_R = SpatialForce(tau=torque_R, f=force_R)
    #     weight = SpatialForce(tau=np.zeros(3), f=force_mg)
    #     external_forces = MultibodyForces(plant_id)

    #     contact_frame_L.AddInForceInWorld(plant_id_context, F_Bo_W_L, external_forces)
    #     contact_frame_R.AddInForceInWorld(plant_id_context, F_Bo_W_R, external_forces)
    #     # torso_frame.AddInForceInWorld(plant_id_context, weight, external_forces)
    #     known_vdot = np.zeros(18)
    #     known_vdot[5] = 0
    #     tau_g = plant_id.CalcGravityGeneralizedForces(context=plant_id_context)
    #     tau_id = plant_id.CalcInverseDynamics(context=plant_id_context,
    #                                 known_vdot = known_vdot,
    #                                 external_forces=external_forces)
    #     total_torque = tau_id-tau_g
    #     self.torque = total_torque[6:]
    #     print("tau_g : ", tau_g)
    #     print("tau_id : ", tau_id)    

    def UpdateWalkState(self, q, context):
        self.plant_half.SetPositions(self.plant_half_context, q)
        W_P_com = self.plant_half.CalcCenterOfMassPositionInWorld(self.plant_half_context)
        # print("W_P_com : ", W_P_com[0:2])
        torso = self.plant_half.GetFrameByName("torso")
        W_P_torso = torso.CalcPoseInWorld(self.plant_half_context).translation()
        L_foot_contact = self.plant_half.GetFrameByName("L_foot_contact")
        L_P_com = L_foot_contact.CalcPoseInWorld(self.plant_half_context).inverse().multiply(W_P_com)
        # L_P_torso = L_foot_contact.CalcPoseInWorld(self.plant_half_context).inverse().multiply(W_P_torso)
        R_foot_contact = self.plant_half.GetFrameByName("R_foot_contact")
        R_P_com = R_foot_contact.CalcPoseInWorld(self.plant_half_context).inverse().multiply(W_P_com)
        # R_P_torso = R_foot_contact.CalcPoseInWorld(self.plant_half_context).inverse().multiply(W_P_torso)
        offset = 7 # [mm]
        if "Hanuman" in self.model_series:
            L_foot_rect = np.array([-69+offset,-27+offset,
                                    96-offset, 63-offset]) * 1e-3
            R_foot_rect = np.array([-69+offset,-63+offset,
                                    96-offset, 27-offset]) * 1e-3
        elif "Volta" in self.model_series:
            L_foot_rect = np.array([-110+offset,-49+offset,
                                    110-offset, 49-offset]) * 1e-3
            R_foot_rect = np.array([-110+offset,-49+offset,
                                    110-offset, 49-offset]) * 1e-3
        # print("est L_P_com : ",L_P_com[0:2])
        # print("est R_P_com : ",R_P_com[0:2])
        # print("L_foot_rect : ",L_foot_rect)
        if IsInsideRectangle(point=L_P_com[0:2], rect=L_foot_rect):
            self.walk_states = "L_stance"
        elif IsInsideRectangle(point=R_P_com[0:2], rect=R_foot_rect):
            self.walk_states = "R_stance"
        else:
            self.walk_states = "double_stance"

    def CalcTorqueFromGRF(self,plant:MultibodyPlant,contact_results:list):
        L_force = contact_results[0:3]
        # print("L_force :",L_force)
        R_force = contact_results[3:6]
        # print("R_force :",R_force)
        L_force_location = contact_results[6:9]
        R_force_location = contact_results[9:]
        L_foot_contact = plant.GetBodyByName("L_foot_contact")  
        L_wrench = SpatialForce(tau=np.zeros(3), f=L_force)
        R_foot_contact = plant.GetBodyByName("R_foot_contact")  
        R_wrench = SpatialForce(tau=np.zeros(3), f=R_force)
        external_forces = MultibodyForces(plant)
        L_foot_contact.AddInForce(
            context=plant.CreateDefaultContext(),
            p_BP_E=L_force_location,
            F_Bp_E=L_wrench,
            frame_E=plant.world_frame(),
            forces=external_forces
        ) 
        R_foot_contact.AddInForce(
            context=plant.CreateDefaultContext(),
            p_BP_E=R_force_location,
            F_Bp_E=R_wrench,
            frame_E=plant.world_frame(),
            forces=external_forces
        ) 
        tau_ext = plant.CalcGeneralizedForces(plant.CreateDefaultContext(), external_forces)
        return tau_ext
    
    def CalcOutput_torque_g(self, context, output):
        self.torque = np.array([-2.33027321e-02, -4.33789260e-02, -2.53122745e-02,  4.66667791e+00,
        8.34129257e-01, -2.79090287e-03, -3.05349702e-02,  5.56714405e-03,
        3.30221238e-03, -4.69905077e+00, -8.32353801e-01, -2.93088889e-03])
        torque_output = self.torque.copy()
        torque_output[0] = self.torque[2]
        torque_output[2] = self.torque[0]
        torque_output[6] = self.torque[8]
        torque_output[8] = self.torque[6]

        output.SetFromVector(1.0*torque_output)

        # Uncomment the line below to set
        # output.SetFromVector(np.zeros(12))