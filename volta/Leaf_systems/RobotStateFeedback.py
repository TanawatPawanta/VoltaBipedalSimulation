import numpy as np
from pydrake.math import RollPitchYaw
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import BasicVector, LeafSystem

from volta.robot.kinematics import (
    CalcCoMPositionFromStanceLeg,
    CalcForwardKinematics,
    GetCoMLinearVelocityViaMomentum,
    GetRelativeVelocity,
)
from volta.robot.states import States


class RobotStateFeedback(LeafSystem):
    def __init__(self, plant:MultibodyPlant, dt, generalized_position):
        super().__init__()
        self.plant = plant
        self.plant_context = plant.CreateDefaultContext()
        self.dt = dt
        self.current_robot_state = np.concatenate((generalized_position,
                                                   np.zeros(18)))
        self.plant.SetPositions(self.plant_context, generalized_position) 
        self.DefineVariables()
        self.DefineIOPort()

        # Declare periodic events 
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=self.dt,  
            offset_sec=0.0,  
            update=self.DiscreteUpdate)
        
    def DefineVariables(self):
        self.reference_leg = [1,0]  # [left_leg, right_leg]
        self.CoM_states_in_stance = States("CoM_state_in_stance")
        self.swing_leg_state_in_stance = States("swing_leg_state_in_stance")
        self.CoM_states_in_stance.position = CalcCoMPositionFromStanceLeg(plant=self.plant,
                                                                        plant_context=self.plant_context,
                                                                        stance_leg="L_foot_contact")
        self.generalized_position = self.current_robot_state[0:19]
        self.generalized_velocity = self.current_robot_state[19:37]
        self.sw_linear_velocity_via_euler = np.zeros(3)
        self.com_linear_acc_in_W = np.zeros(3)
    def DefineIOPort(self):
        # Declare an output port 
        self.input_port_0 = self.DeclareVectorInputPort(
            "Robot states feedback",
            BasicVector(37)  
        )
        self.input_port_1 = self.DeclareVectorInputPort(
            "Reference leg",
            BasicVector(2)  
        )
        self.DeclareVectorOutputPort(
            "CoM state in stance",
            BasicVector(12),
            self.CalcOutput_0
        )
        self.DeclareVectorOutputPort(
            "Swing leg state in stance",
            BasicVector(12),
            self.CalcOutput_1
        )
        self.DeclareVectorOutputPort(
            "CoM acceleration in W",
            BasicVector(3),
            self.CalcOutput_2
        )
    def UpdateVariables(self,context):
        self.current_robot_state = self.input_port_0.Eval(context)
        self.generalized_position = self.current_robot_state[0:19]
        self.generalized_velocity = self.current_robot_state[19:37]
        self.reference_leg = self.input_port_1.Eval(context)
        self.plant.SetPositions(self.plant_context, self.generalized_position) 
        self.plant.SetVelocities(self.plant_context, self.generalized_velocity)
    
    def DiscreteUpdate(self, context, discrete_state):
        # Update discrete variables
        # print("RSF reference leg",self.reference_leg)
        self.UpdateVariables(context)
        self.UpdateCoMStateInStance(self.reference_leg)
        self.UpdateSwingLegStateInStance(self.reference_leg)
        print("generalized position:", self.generalized_position)
        print("curr com position in stance:", self.CoM_states_in_stance.position)
        self.com_linear_acc_in_W = self.plant.CalcCenterOfMassTranslationalAccelerationInWorld(context=self.plant_context)
    def UpdateCoMStateInStance(self,reference_leg):
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
        torso_orientation = RollPitchYaw(R_torso).vector()
        self.CoM_states_in_stance.position = st_P_com
        self.CoM_states_in_stance.orientation = torso_orientation
        self.CoM_states_in_stance.linear_velocity = GetCoMLinearVelocityViaMomentum(self.plant, 
                                                                                    self.plant_context)
        self.CoM_states_in_stance.angular_velocity = self.current_robot_state[19:22]
    
    def UpdateSwingLegStateInStance(self,reference_leg):
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
        self.swing_leg_state_in_stance.position = sw_leg_position_in_st
        self.swing_leg_state_in_stance.orientation = swing_leg_orientation
        self.swing_leg_state_in_stance.linear_velocity = linear_vel
        self.swing_leg_state_in_stance.angular_velocity = angular_vel
        self.sw_linear_velocity_via_euler = self.swing_leg_state_in_stance.CalcLinearVelocity(self.dt)
    
    def CalcOutput_0(self, context, output):
        out = self.CoM_states_in_stance.GetStateVector()
        output.SetFromVector(out)

    def CalcOutput_1(self, context, output):
        out = self.swing_leg_state_in_stance.GetStateVector()
        output.SetFromVector(out)

    def CalcOutput_2(self, context, output):
        out = self.com_linear_acc_in_W
        output.SetFromVector(out)