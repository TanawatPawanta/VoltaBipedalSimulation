import time

import numpy as np
from pydrake.math import VectorToSkewSymmetric
from pydrake.multibody.math import SpatialForce
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import MultibodyForces
from pydrake.systems.framework import BasicVector, LeafSystem

from volta.control.contact_wrench_solver import CalcContactJacobian, ContactWrenchSolver
from volta.control.stance_leg_fsm import StanceLegStatemachine
from volta.robot.kinematics import CalcCoMPositionFromStanceLeg, CalcContactPointPositionsInCoM
from volta.robot.states import States


class PassivityBasedWBC2(LeafSystem):
    """Passivity-based whole-body controller: task-space references -> joint torques.

    Inputs:  0 robot state (q, v) [37], 1 desired CoM [18], 2 desired lfoot [12], 3 desired rfoot [12],
             4 contact flags [2], 5 CoM state [12], 6 lfoot state [12], 7 rfoot state [12],
             8 CoP in each foot frame [4]
    Outputs: 0 joint torques [12], 1 desired foot wrenches [12], 2 Fz bounds + force impedance [5],
             3 walking phase [1]
    12-vectors are [rpy, xyz, angular vel, linear vel]; 18-vectors append [angular acc, linear acc].
    """
    def __init__(self, 
                 plant:MultibodyPlant,
                 CoM_kp=np.array([1.0, 1.0, 1.0]),  
                 CoM_kd=np.array([0.01, 0.01, 0.01]),
                 CoM_k_stiffness=np.array([1.0, 1.0, 1.0]),
                 CoM_k_damping=np.array([0.1, 0.1, 0.1]),
                 sw_kp=np.array([1.0, 1.0, 1.0]),
                 sw_kd=np.array([0.01, 0.01, 0.01]),
                 sw_k_stiffness=np.array([3.0, 3.0, 3.0]),
                 sw_k_damping=np.array([0.0, 0.0, 0.0]),
                 initial_joint_position = np.zeros(12),
                 dt=0.01):
        super().__init__()
        self.dt = dt
        self.CoM_kp = CoM_kp
        self.CoM_kd = CoM_kd
        self.CoM_k_stiffness = CoM_k_stiffness
        self.CoM_k_damping = CoM_k_damping
        self.sw_kp = sw_kp
        self.sw_kd = sw_kd
        self.sw_k_stiffness = sw_k_stiffness
        self.sw_k_damping = sw_k_damping
        self.plant = plant
        self.plant_context = self.plant.CreateDefaultContext()
        self.q = initial_joint_position
        self.stance_leg_state_machine = StanceLegStatemachine(transition_duration=0.2)
        self.DefineVariables()
        self.DefineIOPort()
        # Declare periodic events 
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=self.dt,  
            offset_sec=0.0,  
            update=self.DiscreteUpdate)
        
    def DefineVariables(self):
        self.gravitational_vector  = np.array([0, 0, -9.81])
        self.contact_frames = [
            "L_foot_contact_FL", "L_foot_contact_FR", "L_foot_contact_RL", "L_foot_contact_RR",
            "R_foot_contact_FL", "R_foot_contact_FR", "R_foot_contact_RL", "R_foot_contact_RR"
        ]
        self.current_CoM_states       = States("current_CoM_states")
        self.current_lfoot_states       = States("current_lfoot_states")
        self.current_rfoot_states       = States("current_rfoot_states")
        self.current_sw_leg_states    = States("current_sw_leg_states")
        self.desired_CoM_states       = States("desired_CoM_states")
        self.desired_lfoot_states       = States("desired_lfoot_states")
        self.desired_rfoot_states       = States("desired_rfoot_states")
        self.desired_sw_leg_states    = States("desired_sw_leg_states")     
        self.current_sw_leg_states.position = np.array([0.0, -0.125, 0.0])  # [x, y, z]
        self.desired_sw_leg_states.position = np.array([0.0, -0.125, 0.0])  # [x, y, z]
        self.selection_matrix = np.concatenate((np.zeros((12, 6)), np.eye(12)), axis=1)
        body_pose = np.concatenate((np.array([1.0,0.0,0.0,0.0]), np.array([0,0,0])))
        self.generalize_positions = np.concatenate((body_pose, np.array(self.q)))
        self.generalize_velocities = np.zeros(self.plant.num_velocities())
        self.plant.SetPositions(self.plant_context, self.generalize_positions)
        initial_CoM_position = CalcCoMPositionFromStanceLeg(plant=self.plant,
                                                            plant_context=self.plant_context,
                                                            stance_leg="L_foot_contact")
        self.desired_CoM_states.position = initial_CoM_position
        self.do_onece = 0
        self.total_torque = np.zeros(12)
        self.tau_g = np.zeros(12)
        self.leg_state = "double_stance" # L_stance or R_stance or double_stance.
        self.leg_phase = [1,1] # [L_leg_phase, R_leg_phase], 1 for stance, 0 for swing.
        self.contact_wrench_solver = ContactWrenchSolver(fz_boundaries=np.array([5.0, 100.0, 5.0, 100.0]))
        self.f_imp = np.zeros(6) # impedance force initial
        self.transition_rate = 0.0
        self.desired_foot_wrenchs = np.zeros(12)
        self.next_stance_leg = "double_stance"
        self.fz_boundary = np.zeros(4)  # [L_fz_min, L_fz_max, R_fz_min, R_fz_max]
        self.reference_leg = [1,0]
        self.CaclForceDistributionMatrix(next_stance_leg=self.next_stance_leg,
                                leg_state=self.leg_state,
                                leg_phase=self.leg_phase)
        self.walking_phase = 2 # 0=L stance, 1=R stance, 2=Double stance
        self.lfoot_frame = self.plant.GetFrameByName("L_foot_contact")
        self.rfoot_frame = self.plant.GetFrameByName("R_foot_contact")
        self.base_frame = self.plant.GetFrameByName("torso") 
        self.CoP_l = np.zeros(2)
        self.CoP_r = np.zeros(2)

    def DefineIOPort(self):  
        # Inout ports
        self.robot_states_feedback_port = self.DeclareVectorInputPort(
            "robot states feedback(q,qd)",
            BasicVector(37)  
        )
        self.desired_CoM_states_port = self.DeclareVectorInputPort(
            "Desired CoM states ",
            BasicVector(18)  
        )
        self.desired_lfoot_states_port = self.DeclareVectorInputPort(
            "Desired lfoot states ",
            BasicVector(12)  
        )
        self.desired_rfoot_states_port = self.DeclareVectorInputPort(
            "Desired rfoot states ",
            BasicVector(12)  
        )
        self.contact_port = self.DeclareVectorInputPort(
            "Contact",
            BasicVector(2)  
        )
        self.current_CoM_states_port = self.DeclareVectorInputPort(
            "Current CoM states", BasicVector(12)
        )
        self.current_lfoot_states_port = self.DeclareVectorInputPort(
            "Current lfoot states", BasicVector(12)
        )
        self.current_rfoot_states_port = self.DeclareVectorInputPort(
            "Current rfoot states", BasicVector(12)
        )
        self.current_CoP_port = self.DeclareVectorInputPort(
            "Current CoP", BasicVector(4)
        )
        # Output ports
        self.DeclareVectorOutputPort(
            "Actuation torques",
            BasicVector(12),
            self.CalcOutput_actuation_torques 
        )
        self.DeclareVectorOutputPort(
            "Desired Foot Wrenches",
            BasicVector(12),
            self.CalcOutput_desired_foot_wrenches
        )   
        self.DeclareVectorOutputPort(
            "Fz Boundaries + force impedance",
            BasicVector(5),
            self.CalcOutput_fz_boundaries
        )    
        self.DeclareVectorOutputPort(
            "Walking Phase",
            BasicVector(1),
            self.CalcOutput_walking_phase
        )  
    def UpdateVariables(self,context):
        # 1. Update internal plant for dynamics calculations (Jacobians, Mass matrix, ID)
        self.robot_states = self.robot_states_feedback_port.Eval(context)
        self.generalize_position = self.robot_states[0:19]
        self.q = self.generalize_position[7:]
        self.generalize_velocities = self.robot_states[19:37]
        self.qd = self.generalize_velocities[6:]

        self.plant.SetPositions(self.plant_context, self.generalize_position)
        self.plant.SetVelocities(self.plant_context, self.generalize_velocities)

        # 2. Read Current States from the Estimator Ports
        current_com_data = self.current_CoM_states_port.Eval(context)
        current_lfoot_data = self.current_lfoot_states_port.Eval(context)
        current_rfoot_data = self.current_rfoot_states_port.Eval(context)

        # 3. Update Current CoM State Object
        self.current_CoM_states.UpdateStates(data=current_com_data)

        # 4. Update Desired CoM State Object
        desired_com_states = self.desired_CoM_states_port.Eval(context)
        self.desired_CoM_states.UpdateStates(data=desired_com_states)

        # 5. Handle Contact and Leg States
        next_stance_leg = self.contact_port.Eval(context)
        self.next_stance_leg = self.MapNextLegStates(next_leg_state=next_stance_leg)

        desired_lfoot_states = self.desired_lfoot_states_port.Eval(context)
        desired_rfoot_states = self.desired_rfoot_states_port.Eval(context)
        self.current_lfoot_states.UpdateStates(data=current_lfoot_data)
        self.current_rfoot_states.UpdateStates(data=current_rfoot_data)

        # 6. Assign swing leg tracking based on stance leg
        # Note: The estimator already handles the 1e-3 thresholding, so we don't need it here anymore.
        if self.next_stance_leg == "L_stance":
            self.desired_sw_leg_states.UpdateStates(data=desired_rfoot_states)
            self.current_sw_leg_states.UpdateStates(data=current_rfoot_data)

        elif self.next_stance_leg == "R_stance":  
            self.desired_sw_leg_states.UpdateStates(data=desired_lfoot_states)
            self.current_sw_leg_states.UpdateStates(data=current_lfoot_data)
        # print("next stance leg:", self.next_stance_leg)
        # print("desired sw",self.desired_sw_leg_states.orientation)
        # print("current sw",self.current_sw_leg_states.orientation)
        # print("desired com",self.desired_CoM_states.position)
        # print("current com orientation",self.current_CoM_states.orientation)
        # print("current com position",self.current_CoM_states.position)
        # print("generalized position",self.generalize_position)
        CoP = self.current_CoP_port.Eval(context)
        self.CoP_l = CoP[0:2]
        self.CoP_r = CoP[2:4]


    def DiscreteUpdate(self, context, discrete_state):
        # print("-"*50)
        # print("time step:", context.get_time())
        start_time = time.time()
        self.UpdateVariables(context=context) 
        self.stance_leg_state_machine.SetNextLegState(next_leg_state=self.next_stance_leg)
        self.stance_leg_state_machine.UpdateStanceLegStateMachine()
        self.transition_rate = self.stance_leg_state_machine.GetTransitionRate()
        self.leg_phase = self.stance_leg_state_machine.GetlegPhase()
        self.leg_state = self.stance_leg_state_machine.GetCurrentLegState()
        # print(f"Leg state: {self.leg_state}, Leg phase: {self.leg_phase}, Transition rate: {self.transition_rate}")
        m = self.plant.CalcTotalMass(self.plant_context) 
        # print("m:",m)
        wff_c = np.array([0.0, 
                        0.0, 
                        0.0, 
                        1.0*m*self.desired_CoM_states.linear_acceleration[0], 
                        1.0*m*self.desired_CoM_states.linear_acceleration[1], 
                        1.0*m*self.desired_CoM_states.linear_acceleration[2]])     # feedforward wrench
        # print("wff_c: ",wff_c)
        wg = np.array([0.0, 
                        0.0, 
                        0.0, 
                        0.0, 
                        0.0, 
                        m*self.gravitational_vector[2]])  # gravity force  
        # print("w_g:", wg)
        wimp_c = self.CalcCoMWrenchImpedance() # impedance wrench
        # wimp_c = np.zeros(6)
        # print("wimp_c:", wimp_c)
        self.f_imp = self.CalcSwingLegWrenchImpedance()
        # print("f_imp:", self.f_imp)
        desired_GAF = m*self.gravitational_vector 
        desired_CoP_position = np.array([0.0, 0.0])  # same as projected CoM position to the ground
        contact_forces_Z = self.CalcContactForcesDistribution(desired_GAF=desired_GAF,
                                                              desired_CoP_position=desired_CoP_position,
                                                              next_stance_leg=self.next_stance_leg,
                                                              leg_state=self.leg_state,
                                                              leg_phase=self.leg_phase)
        # print("Precomput contact force: ",contact_forces_Z)
        fz_ref = self.CalcFz(contact_forces_Z=contact_forces_Z)
        # fz_ref = np.zeros(2)
        # print("fz ref:", fz_ref)
        J_grf_c = CalcContactJacobian(self.plant,
                                      self.plant_context,
                                      left_foot_name="L_foot_contact",
                                      right_foot_name="R_foot_contact") 
        # Solve for contact wrenches       
        desired_contact_wrenchs, self.fz_boundary = self.contact_wrench_solver.SolveContactWrench(wff_c=wff_c, 
                                                                                    wg=wg, 
                                                                                    wimp_c=wimp_c, 
                                                                                    com_J_leg=J_grf_c,
                                                                                    f_imp=self.f_imp,
                                                                                    fz_ref=fz_ref,
                                                                                    transition_rate=self.transition_rate,
                                                                                    leg_phase=self.leg_phase,
                                                                                    leg_state=self.leg_state,
                                                                                    next_leg_state=self.next_stance_leg)
        # print("Fz boundary:", self.fz_boundary)
        wrench_stabilize = self.StanceLegStabilizeController(K=20.0,
                                                             D_lin=0.5,
                                                             D_ang=5.0)
        # wrench_stabilize = self.StanceLegStabilizeController(K=0.0,
        #                                                      D_lin=0.5,
        #                                                      D_ang=25.0)
        # print("Wrench stabilize: ", wrench_stabilize)
        # desired_contact_wrenchs[0:3] = np.zeros(3)  # Remove x and y forces from optimization result, rely on impedance controller for that.
        # desired_contact_wrenchs[6:9] = np.zeros(3)
        # print("Desired contact wrench before stabilization: ", desired_contact_wrenchs)
        self.desired_foot_wrenchs = desired_contact_wrenchs + wrench_stabilize
        # Compute total torque command from inverse dynamics
        known_vdot = np.zeros(18)
        torque_id = self.CalcTorqueFromInverseDynamics(known_vdot=known_vdot,
                                                       feet_wrenches=self.desired_foot_wrenchs)
        # Compute gravitational term
        torque_g = self.plant.CalcGravityGeneralizedForces(context=self.plant_context)
        self.tau_g = self.selection_matrix @ torque_g
        self.total_torque = (self.selection_matrix@(torque_id - torque_g))
        end_time = time.time()
        # print(f"Time consumed in WBC: {end_time-start_time}")
        # print(f"Desired contact wrench {self.desired_foot_wrenchs}")

    def CaclForceDistributionMatrix(self, 
                                    next_stance_leg, 
                                    leg_state,
                                    leg_phase):
        match next_stance_leg:
            case "L_stance":
                active_contact_frames = self.contact_frames[0:4]
            case "R_stance":
                active_contact_frames = self.contact_frames[4:8]
            case "double_stance":
                active_contact_frames = self.contact_frames
                if leg_state == "transition":
                    match leg_phase:
                        case [1,0]:
                            active_contact_frames = self.contact_frames[0:4]
                        case [0,1]:
                            active_contact_frames = self.contact_frames[4:8]
            case "transition": 
                active_contact_frames = self.contact_frames

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
        return active_contact_frames
    
    def CalcContactForcesDistribution(self,
                                      desired_GAF,
                                      desired_CoP_position,
                                      next_stance_leg, 
                                      leg_state,
                                      leg_phase):
        active_contact_frames=self.CaclForceDistributionMatrix(next_stance_leg, 
                                         leg_state,
                                         leg_phase)
        contact_forces_Z = np.linalg.pinv(self.force_distribution_matrix_Z) @ np.array([desired_CoP_position[0], desired_CoP_position[1], 1.0]) * desired_GAF[2]
        if contact_forces_Z.shape[0] == 8:
            return contact_forces_Z
        else:
            if active_contact_frames[0][0]=="L":
                return np.concatenate((contact_forces_Z, np.zeros(4)), axis=0)
            elif active_contact_frames[0][0]=="R":
                return np.concatenate((np.zeros(4), contact_forces_Z), axis=0)

    def MapNextLegStates(self, next_leg_state):
        """
        Maps the next leg state to the corresponding stance leg.
        """
        next_leg_state = next_leg_state.tolist()
        if next_leg_state == [1,0]:
            return "L_stance"
        elif next_leg_state == [0,1]:
            return "R_stance"
        elif next_leg_state == [1,1]:
            return "double_stance"
        else:
            raise ValueError("Invalid next leg state: {}".format(next_leg_state))

    def CalcCoMWrenchImpedance(self):
            # --- 1. Position Control (Translational) ---
            # print("curr com position",self.current_CoM_states.position)
            # print("desired com position", self.desired_CoM_states.position)
            # print("curr com orientation",self.current_CoM_states.orientation)
            # print("desired com orientation", self.desired_CoM_states.orientation)
    
            position_error = self.desired_CoM_states.position - self.current_CoM_states.position
            velocity_error = self.desired_CoM_states.linear_velocity - self.current_CoM_states.linear_velocity
            position_threshold = 1e-04
            position_error[abs(position_error) <= position_threshold] = 0.0
            velocity_threshold = 2e-03
            velocity_error[abs(velocity_error) <= velocity_threshold] = 0.0
            # if abs(position_error[0]) >=0.08:
            #     position_error[0]=0.0
            # if abs(position_error[1]) >=0.2:
            #     position_error[1]=0.0
            fc_imp = self.CoM_kp * position_error + self.CoM_kd * velocity_error
            max_fc_imp = [5.0,5.0,50.0]
            for i in range(3):
                if abs(fc_imp[i])>max_fc_imp[i]:
                    fc_imp[i] = np.sign(fc_imp[i])*max_fc_imp[i]
            # --- 2. Orientation Control (Rotational) ---
            R_b = self.current_CoM_states.GetOrientationInRotationMatrix()
            R_desired = self.desired_CoM_states.GetOrientationInRotationMatrix()
            # R_desired = RotationMatrix.MakeZRotation(np.deg2rad(0.0))@RotationMatrix.MakeYRotation(np.deg2rad(0.0))@RotationMatrix.MakeXRotation(np.deg2rad(0.0)) 
            tau_stiffness_body = self.CaclOrientationImpedant(
                                                            R_b=R_b,
                                                            R_d=R_desired,
                                                            K_r=self.CoM_k_stiffness
            )
            # D. COMPUTE ANGULAR VELOCITY IN BODY FRAME (Fixes Problem 2)
            # Assuming current_CoM_velocity_in_W[0:3] is World angular velocity (omega_W)
            w_world = self.current_CoM_states.angular_velocity
            
            # Transform World angular velocity to Virtual Body Frame
            # w_body = R_body^T * w_world
            # w_body = R_b.transpose() @ w_world
            
            # Desired angular velocity (usually 0 for stabilization)
            w_desired_body = self.desired_CoM_states.angular_velocity 
            w_error_body = w_desired_body - w_world
    
            # F. COMPUTE DAMPING TORQUE (In Body Frame)
            # Damping must be applied to body velocity 
            tau_damping_body = self.CoM_k_damping * w_error_body
            
            # G. COMBINE AND ROTATE TO WORLD
            # Total torque in body frame
            tau_total_body = tau_stiffness_body + tau_damping_body
            nc_imp = R_b @ tau_total_body   
            return np.concatenate((nc_imp, -1.0*fc_imp), axis=0)

    def CaclOrientationImpedant(self,R_b, R_d, K_r):
        """
        Computes the rotational spring torque based on the quaternion method 
        in "Posture and balance control..." (Ott et al. 2011), Ref [26].
        
        This implementation uses pydrake's math classes.
        
        Implements: tau_r = -2 * (delta*I + epsilon_hat) @ K_r @ epsilon
        
        Args:
            R_b (pydrake.math.RotationMatrix): Current trunk orientation
            R_d (pydrake.math.RotationMatrix): Desired trunk orientation
            K_r (np.ndarray): 3x3 stiffness gain matrix
            
        Returns:
            np.ndarray: 3x1 rotational spring torque (tau_r)
        """
        # 1. Calculate the error rotation matrix
        # R_err = R_d^T @ R_b (as defined in the paper)
        # In pydrake, R_d.transpose() is the same as R_d.inverse()
        R_err_matrix = R_d.transpose() @ R_b  
        # print("R err",R_err_matrix)
        # print("R err rpy",RollPitchYaw(R_err_matrix).vector()*180.0/np.pi)
        # 2. Convert to quaternion to get delta and epsilon
        # pydrake's ToQuaternion() method is robust.
        q_err = R_err_matrix.ToQuaternion()   
        # q_err = 1e-04

        # print("Q err",q_err)    
        # Get the scalar part (delta) and vector part (epsilon)
        delta = q_err.w()       # scalar w
        epsilon = q_err.xyz()   # 3D vector [x, y, z]
        threshold = 1e-03
        # delta[abs(delta) <= threshold] = 0.0
        epsilon[abs(epsilon) <= threshold] = 0.0
        epsilon_hat = VectorToSkewSymmetric(epsilon)
        I = np.eye(3)
        term1 = (delta * I + epsilon_hat)
        term2 = K_r * epsilon
        tau_r = -2.0*(term1 @ term2)
        return tau_r
  
    def CalcSwingLegWrenchImpedance(self):
        position_error = self.desired_sw_leg_states.position - self.current_sw_leg_states.position
        velocity_error = self.desired_sw_leg_states.linear_velocity - self.current_sw_leg_states.linear_velocity
        position_threshold = 1e-04
        position_error[abs(position_error) <= position_threshold] = 0.0
        velocity_threshold = 2e-03
        velocity_error[abs(velocity_error) <= velocity_threshold] = 0.0
        f_sw_imp = self.sw_kp * position_error + self.sw_kd * velocity_error

        R_current = self.current_sw_leg_states.GetOrientationInRotationMatrix()
        # print("sw desired orientation",self.desired_sw_leg_states.orientation)
        R_desired = self.desired_sw_leg_states.GetOrientationInRotationMatrix()
        # print("R sw current",self.current_sw_leg_states.orientation*180.0/np.pi)
        stifness_term = self.CaclOrientationImpedant(
            R_b=R_current,
            R_d=R_desired,
            K_r=self.sw_k_stiffness)
        damping_term = self.sw_k_damping * (np.array([0.0,0.0,self.desired_sw_leg_states.angular_velocity[2]]) - self.current_sw_leg_states.angular_velocity)
        n_sw_imp = stifness_term + damping_term
        wrench_sw_imp = np.concatenate((n_sw_imp, -1.0*f_sw_imp), axis=0)
        return wrench_sw_imp
    
    def StanceLegStabilizeController(self,K, D_lin:float, D_ang:float):
        D = np.concatenate((np.ones(3)*D_lin,
                            np.ones(3)*D_ang))
        match self.leg_phase:
            case [1,0]:
                wrench_l = self.CalcStanceLegStabilizeWrench(K=K, D=D,
                                                             stance_body_name="L_foot_contact")
                wrench_stabilize = np.concatenate((wrench_l,np.zeros(6)))
            case [0,1]:
                wrench_r = self.CalcStanceLegStabilizeWrench(K=K, D=D,
                                                             stance_body_name="R_foot_contact")
                wrench_stabilize = np.concatenate((np.zeros(6),wrench_r))
            case [1,1]:
                wrench_l = self.CalcStanceLegStabilizeWrench(K=K, D=D,
                                                             stance_body_name="L_foot_contact")
                wrench_r = self.CalcStanceLegStabilizeWrench(K=K, D=D,
                                                             stance_body_name="R_foot_contact")
                wrench_stabilize = np.concatenate((wrench_l,wrench_r))
        return wrench_stabilize

    def CalcStanceLegStabilizeWrench(self,K,D,stance_body_name:str):
        stance_frame = self.plant.GetBodyByName(stance_body_name).body_frame()
        spatial_velocity = stance_frame.CalcSpatialVelocity(
            context = self.plant_context,
            frame_M = self.plant.world_frame(), 
            frame_E = stance_frame
        )
        if stance_body_name == "L_foot_contact":
            self.current_lfoot_states.orientation[2] = self.desired_rfoot_states.orientation[2] # Only stabilize yaw, allow roll and pitch to be determined by optimization and impedance.
            R_current = self.current_lfoot_states.GetOrientationInRotationMatrix()
            R_desired = self.desired_lfoot_states.GetOrientationInRotationMatrix()
        elif stance_body_name == "R_foot_contact":
            self.current_rfoot_states.orientation[2] = self.desired_lfoot_states.orientation[2] # Only stabilize yaw, allow roll and pitch to be determined by optimization and impedance.
            R_current = self.current_rfoot_states.GetOrientationInRotationMatrix()
            R_desired = self.desired_rfoot_states.GetOrientationInRotationMatrix()
        stifness_term = self.CaclOrientationImpedant(
            R_b=R_current,
            R_d=R_desired,
            K_r=np.ones(3)*K)
        # print("stiffness term", stifness_term)
        
        w_rel = spatial_velocity.rotational()    # Angular velocity (rad/s)
        v_rel = spatial_velocity.translational() # Linear velocity (m/s)
        angular_vel = w_rel.copy()
        linear_vel = v_rel.copy()
        twist = np.concatenate((angular_vel,linear_vel))
        threshold = 1e-03
        twist[np.abs(twist) < threshold] = 0.0
        wrench = -1.0*D*twist
        # print("damp term",wrench)
        wrench[0:3] += stifness_term
        return wrench

    def CalcTorqueFromInverseDynamics(self,known_vdot,feet_wrenches):
        external_forces = MultibodyForces(self.plant)
        contact_frames = ["L_foot_contact", "R_foot_contact"]
        contact_forces_x = [feet_wrenches[3], feet_wrenches[9]]  # x-component
        contact_forces_y = [feet_wrenches[4], feet_wrenches[10]]  # y-component
        contact_forces_z = [feet_wrenches[5], feet_wrenches[11]]  # z-component
        moment_x = [feet_wrenches[0], feet_wrenches[6]]  # moment x-component
        moment_y = [feet_wrenches[1], feet_wrenches[7]]  # moment y-component
        moment_z = [feet_wrenches[2], feet_wrenches[8]]  # moment z-component
        for i in range(len(contact_frames)):
            contact_frame = self.plant.GetBodyByName(contact_frames[i])
            F_Bo_W = SpatialForce(tau=-1.0*np.array([moment_x[i],
                                                     moment_y[i],
                                                     moment_z[i]]),
                                  f=1.0*np.array([contact_forces_x[i],
                                                  contact_forces_y[i],
                                                  contact_forces_z[i]]))
            contact_frame.AddInForceInWorld(self.plant_context, F_Bo_W, external_forces)
        torque_id = self.plant.CalcInverseDynamics(context=self.plant_context,
                                                known_vdot=known_vdot,
                                                external_forces=external_forces)
        return torque_id

    def CalcFz(self,contact_forces_Z):
        fz_L = 0.0
        fz_R = 0.0
        for i in range(len(contact_forces_Z)):
            if i < 4:
                fz_L += contact_forces_Z[i]
            else:
                fz_R += contact_forces_Z[i]
        return [-1.0*fz_L,-1.0*fz_R]
    
    def CalcOutput_actuation_torques(self, context, output):
        torque_output = self.total_torque.copy()
        torque_output[0] = self.total_torque[2]
        torque_output[2] = self.total_torque[0]
        torque_output[6] = self.total_torque[8]
        torque_output[8] = self.total_torque[6]
        # torque_output = np.zeros(12) # for testing without torque
        output.SetFromVector(torque_output)

    def CalcOutput_desired_foot_wrenches(self, context, output):    
        output.SetFromVector(self.desired_foot_wrenchs)

    def CalcOutput_fz_boundaries(self, context, output):
        output.SetFromVector(np.concatenate((self.fz_boundary, [self.f_imp[5]]), axis=0))

    def CalcOutput_walking_phase(self, context, output):
        # print("walking_phase",self.walking_phase)
        output.SetFromVector(np.array([self.walking_phase]))
    