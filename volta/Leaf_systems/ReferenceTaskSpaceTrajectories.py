import numpy as np
from pydrake.all import BasicVector, LeafSystem
from pydrake.math import RollPitchYaw

from volta.planning.offline_planning import (
    InitP2PTrajectories,
    InitSwingLegTrajectory,
    LIPJointTrajectory,
)
from volta.robot.kinematics import CalcForwardKinematics, GetRelativeVelocity
from volta.robot.states import States, WalkStates
from volta.utils import IsInsideRectangle


class ReferenceTaskSpaceTrajectories(LeafSystem):
    def __init__(self,
                 model_series, 
                 model_name,
                 step_length=0.1, 
                 step_height=0.05, 
                 step_time=5.0, 
                 z_com=0.7,
                 desire_torso_high=0.4, 
                 y_0=0.125,
                 init_CoM_position=np.zeros(3),
                 plant=None,
                 dt=0.01):
        super().__init__()  
        self.dt = dt
        self.plant = plant
        self.plant_context = plant.CreateDefaultContext()
        self.current_CoM_position = init_CoM_position
        self.GaitPlanner = LIPJointTrajectory(model_series=model_series,
                                 model_name=model_name,
                                 step_length=step_length,
                                 step_height=step_height,
                                 step_time=step_time,
                                 z_com=z_com,
                                 desire_torso_high=desire_torso_high,
                                 y_0=y_0)
        self.GaitPlanner.CalcTrajectory()

        self.L_stance = False
        self.walk_state = WalkStates.init
        self.DefineVariables()
        self.DefineIOPort()
        self.DeclarePeriodicUnrestrictedUpdateEvent(period_sec=self.dt                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   ,
                                                    offset_sec=0.0,
                                                    update=self.Update)
        
    def DefineIOPort(self):
        self.robot_states_feedback = self.DeclareVectorInputPort(
            "Robot states feedback",
            BasicVector(37)
        )
        self.DeclareVectorOutputPort(name="Desired CoM trajectory",
                                     size=9,
                                     calc=self.Output_desire_CoM_trajectory)
        self.DeclareVectorOutputPort(name="Desired swing leg trajectory",
                                     size=6,
                                     calc=self.Output_desire_sw_leg_trajectory)
        self.DeclareVectorOutputPort(name="Reference leg",
                                     size=2,
                                     calc=self.Output_reference_leg)
        self.DeclareVectorOutputPort(name="Next stance leg",
                                     size=2,
                                     calc=self.Output_next_stance_leg)
        
    def DefineVariables(self):
        self.generalized_position = np.zeros(19)
        self.current_CoM_state_in_stance = States("current_CoM_state_in_stance")
        self.current_CoM_state_in_stance.position = self.current_CoM_position.copy()
        self.current_sw_leg_state_in_stance = States("current_sw_leg_state_in_stance")
        self.desired_CoM_states = States("desired_CoM_states")
        self.desired_CoM_states.position = self.current_CoM_position.copy()
        self.CoM_traj_current_time = 0.0
        self.total_time = 3.0
        self.calc_CoM_traj = True
        self.CoM_traj = InitP2PTrajectories(p_start=self.current_CoM_position, 
                                    p_final=np.array([self.current_CoM_position[0],
                                                        self.current_CoM_position[1]+0.09,
                                                        self.current_CoM_position[2]-0.0]), 
                                    v_start=np.zeros(3), 
                                    v_final=np.zeros(3), 
                                    total_time=self.total_time)
        self.desired_swing_leg_states = States("desired_swing_leg_states")
        self.desired_swing_leg_states.position = np.array([0.0,-0.125*2.0,0.0])
        self.calc_sw_leg_traj = True
        self.sw_traj_current_time = 0.0
        self.step_time = 3.0
        foot_placements = [0.0, -0.125*2.0, 0.0, 
                        self.GaitPlanner.step_length*1.0, -0.125*2.0, 0.0]
        self.swing_leg_traj = InitSwingLegTrajectory(foot_placements=foot_placements,
                                                    step_time=self.step_time,
                                                    z_max=0.06)
        self.sw_leg_trajectory_position_offset = np.array([0.0, 0.125*2.0, 0.0,])
        self.reference_leg = [1,0] # [L leg, R leg]
        self.next_stance_leg = [1,1]
        self.do_onece = True
        self.auto_update_stance_leg = True 
        self.com_ind = 0
        self.flip = 1

    def UpdateVariables(self, context):
        robot_state = self.robot_states_feedback.Eval(context)
        self.generalized_position = robot_state[0:19]
        # print("generalized position in update:", self.generalized_position)
        self.plant.SetPositions(self.plant_context, self.generalized_position)

    def Update(self, context, state):
        global_time = context.get_time()
        print("-"*50)
        print("RTS time: ",global_time)
        self.UpdateVariables(context=context)
        match self.walk_state:
            case WalkStates.init:
                if global_time > 3.0:
                    if self.sw_traj_current_time < self.step_time:
                        desired_x_sw = []
                        desired_xd_sw = []
                        for i in range(len(self.swing_leg_traj)):
                            desired_x_sw.append(self.swing_leg_traj[i].value(self.sw_traj_current_time)[0][0])
                            desired_xd_sw.append(self.swing_leg_traj[i].EvalDerivative(self.sw_traj_current_time, 1)[0][0])
                        self.desired_swing_leg_states.position = desired_x_sw
                        self.desired_swing_leg_states.linear_velocity = desired_xd_sw
                        self.sw_traj_current_time += self.dt
                    else:
                        self.auto_update_stance_leg = False
                        self.next_stance_leg = [1,1]
                        if self.do_onece:
                            self.UpdateCoMStateInStance(self.reference_leg,
                                                        self.current_CoM_state_in_stance)
                            self.CoM_traj = InitP2PTrajectories(p_start=self.current_CoM_state_in_stance.position, 
                                                                p_final=np.array([self.GaitPlanner.step_length*0.5,
                                                                                -0.125,
                                                                                0.3]), 
                                                                v_start=np.zeros(3), 
                                                                v_final=np.array([0.0,0.0,0.0])*self.GaitPlanner.st_V_c[0,:], 
                                                                total_time=self.total_time)
                            self.calc_CoM_traj = True
                            self.CoM_traj_current_time = 0.0
                            self.do_onece = False

                if self.auto_update_stance_leg:
                    self.UpdateLegState()
                if self.calc_CoM_traj:
                    if self.CoM_traj_current_time<self.total_time:
                        desired_x_c = []
                        desired_xd_c = []
                        desired_xdd_c = []
                        for i in range(len(self.CoM_traj)):
                            desired_x_c.append(self.CoM_traj[i].value(self.CoM_traj_current_time)[0][0])
                            desired_xd_c.append(self.CoM_traj[i].EvalDerivative(self.CoM_traj_current_time, 1)[0][0])
                            desired_xdd_c.append(self.CoM_traj[i].EvalDerivative(self.CoM_traj_current_time, 2)[0][0])
                        self.desired_CoM_states.position = np.array(desired_x_c)
                        self.desired_CoM_states.linear_velocity = np.array(desired_xd_c)
                        self.desired_CoM_states.linear_acceleration = np.array(desired_xdd_c)     
                        self.CoM_traj_current_time += self.dt
                    else:
                        self.calc_CoM_traj = False
                if global_time >= 9.0:
                    self.walk_state = WalkStates.stand
                    self.reference_leg = [0,1]
                    self.UpdateCoMStateInStance(self.reference_leg,self.current_CoM_state_in_stance)
                    # self.desired_CoM_states.position = self.current_CoM_state_in_stance.position.copy()
                    print("RTS desired com position",self.desired_CoM_states.position)
            
            case WalkStates.stand:
                self.UpdateCoMStateInStance(self.reference_leg,
                                            self.current_CoM_state_in_stance)
                self.UpdateSwingLegStateInStance(self.reference_leg,
                                                 self.current_sw_leg_state_in_stance)
                self.sw_leg_trajectory_position_offset = self.current_sw_leg_state_in_stance.position.copy()
                foot_placements = np.concatenate((self.current_sw_leg_state_in_stance.position,
                                                  np.array([self.GaitPlanner.step_length*1.0, 0.125*2.0, 0.0])))
                self.swing_leg_traj = InitSwingLegTrajectory(foot_placements=foot_placements,
                                                            step_time=self.step_time,
                                                            z_max=0.06)
                
                print("sw leg offset:", self.sw_leg_trajectory_position_offset)
                self.walk_state = WalkStates.walk

            case WalkStates.walk:
                if self.com_ind < self.GaitPlanner.st_P_c.shape[0]:
                    self.desired_CoM_states.position            = self.GaitPlanner.st_P_c[self.com_ind,:]
                    self.desired_CoM_states.linear_velocity     = self.GaitPlanner.st_V_c[self.com_ind,:]
                    self.desired_CoM_states.linear_acceleration = self.GaitPlanner.st_Vd_c[self.com_ind,:]

                    self.desired_swing_leg_states.position          = self.GaitPlanner.sw0_P_sw[self.com_ind,:] + self.sw_leg_trajectory_position_offset
                    self.desired_swing_leg_states.linear_velocity   = self.GaitPlanner.sw0_V_sw[self.com_ind,:]
                    self.com_ind += 1
                else:
                    if self.reference_leg == [0,1]:
                        self.reference_leg = [1,0]
                    elif self.reference_leg == [1,0]:
                        self.reference_leg = [0,1]
                    self.UpdateSwingLegStateInStance(self.reference_leg,
                                                 self.current_sw_leg_state_in_stance)
                    self.sw_leg_trajectory_position_offset = self.current_sw_leg_state_in_stance.position.copy()
                    # self.flip = self.flip*-1
                    self.GaitPlanner.st_P_c[:,1] = -1.0*self.GaitPlanner.st_P_c[:,1]
                    self.GaitPlanner.st_V_c[:,1] = -1.0*self.GaitPlanner.st_V_c[:,1]
                    self.GaitPlanner.st_Vd_c[:,1] = -1.0*self.GaitPlanner.st_Vd_c[:,1]
                    self.com_ind = 0
                    # self.desired_CoM_states.linear_velocity     = np.zeros(3)
                    # self.desired_CoM_states.linear_acceleration = np.zeros(3)

    def UpdateCoMStateInStance(self,reference_leg,com_state:States):
        W_P_com = self.plant.CalcCenterOfMassPositionInWorld(self.plant_context)
        if reference_leg[0] == 1:  # left leg is stance leg
            stance_frame_name = "L_foot_contact"
        elif reference_leg[1] == 1:  # right leg is stance leg
            stance_frame_name = "R_foot_contact"
        stance_frame = self.plant.GetFrameByName(stance_frame_name)
        st_P_com = stance_frame.CalcPoseInWorld(self.plant_context).inverse().multiply(W_P_com)
        base_frame = self.plant.GetFrameByName("torso") 
        W_T_torso = base_frame.CalcPoseInWorld(self.plant_context)
        R_torso = W_T_torso.rotation()
        # torso_orientation = RollPitchYaw(R_torso).vector()
        com_state.position = st_P_com
        # self.CoM_satate_in_stance.orientation = torso_orientation
        # self.CoM_satate_in_stance.CalcLinearVelocity(self.dt)
        # self.CoM_satate_in_stance.linear_velocity = GetCoMLinearVelocityViaMomentum(self.plant, 
                                                                                    # self.plant_context)
        # self.CoM_satate_in_stance.angular_velocity = self.current_robot_state[19:22]
        # self.CoM_linear_velocity_via_momentum = GetCoMLinearVelocityViaMomentum(self.plant, self.plant_context)
    
    def UpdateSwingLegStateInStance(self,reference_leg,sw_leg_states:States):
        if reference_leg[0] == 1:  # left leg is stance leg
            stance_frame_name = "L_foot_contact"
            swing_frame_name = "R_foot_contact"
        elif reference_leg[1] == 1:  # right leg is stance leg
            stance_frame_name = "R_foot_contact"
            swing_frame_name = "L_foot_contact"
        
        sw_leg_position_in_st = CalcForwardKinematics(plant=self.plant,
                                                                    generalize_positions=self.generalized_position,
                                                                    base_frame_name=stance_frame_name,
                                                                    end_effector_frame_name=swing_frame_name)
        sw_frame = self.plant.GetFrameByName(swing_frame_name) 
        W_T_sw = sw_frame.CalcPoseInWorld(self.plant_context)
        R_sw = W_T_sw.rotation() 
        swing_leg_orientation = RollPitchYaw(R_sw).vector()
        angular_vel,linear_vel = GetRelativeVelocity(plant=self.plant,
                                                      context=self.plant_context,
                                                      stance_body_name=stance_frame_name,
                                                      swing_body_name=swing_frame_name)
        sw_leg_states.position = sw_leg_position_in_st
        # self.swing_leg_state_in_stance.orientation = swing_leg_orientation
        # self.swing_leg_state_in_stance.linear_velocity = linear_vel
        # self.swing_leg_state_in_stance.angular_velocity = angular_vel
        
    def EvaluateSwLegTrajectory(self):
        if self.sw_traj_current_time < self.step_time:
            desired_x_sw = []
            desired_xd_sw = []
            for i in range(len(self.swing_leg_traj)):
                desired_x_sw.append(self.swing_leg_traj[i].value(self.sw_traj_current_time)[0][0])
                desired_xd_sw.append(self.swing_leg_traj[i].EvalDerivative(self.sw_traj_current_time, 1)[0][0])
            self.desired_swing_leg_states.position = desired_x_sw
            self.desired_swing_leg_states.linear_velocity = desired_xd_sw
            self.sw_traj_current_time += self.dt
            return 0
        else:
            return 1
        
    def UpdateLegState(self):
        W_P_com = self.plant.CalcCenterOfMassPositionInWorld(self.plant_context)
        L_foot_contact = self.plant.GetFrameByName("L_foot_contact")
        L_P_com = L_foot_contact.CalcPoseInWorld(self.plant_context).inverse().multiply(W_P_com)
        R_foot_contact = self.plant.GetFrameByName("R_foot_contact")
        R_P_com = R_foot_contact.CalcPoseInWorld(self.plant_context).inverse().multiply(W_P_com)
        offset = 2 # [mm]
        L_foot_rect = np.array([-110+offset,-49+offset,
                                110-offset, 49-offset]) * 1e-3
        R_foot_rect = np.array([-110+offset,-49+offset,
                                110-offset, 49-offset]) * 1e-3
        if IsInsideRectangle(point=L_P_com[0:2], rect=L_foot_rect):
            self.next_stance_leg = [1,0]
        elif IsInsideRectangle(point=R_P_com[0:2], rect=R_foot_rect):
            self.next_stance_leg = [0,1]
        else:
            self.next_stance_leg = [1,1]

    def Output_desire_CoM_trajectory(self, context, output):
        # dummy_out = np.concatenate((self.current_CoM_position,np.zeros(6)))
        # output.SetFromVector(dummy_out)
        output.SetFromVector(self.desired_CoM_states.GetLinearStateVector(acc=True))

    def Output_desire_sw_leg_trajectory(self, context, output):
        # dummy_out = np.concatenate((np.array([0.0,-0.125*2.0,0.0]),np.zeros(3)))
        # output.SetFromVector(dummy_out)
        output.SetFromVector(self.desired_swing_leg_states.GetLinearStateVector(acc=False)) 
    
    def Output_reference_leg(self, context, output):
        output.SetFromVector(self.reference_leg)

    def Output_next_stance_leg(self, context, output):
        output.SetFromVector(self.next_stance_leg)