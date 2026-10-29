import numpy as np
from pydrake.math import RollPitchYaw
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import BasicVector, LeafSystem


class TaskSpaceFeedback(LeafSystem):
    """
    A LeafSystem that takes the robot's generalized states (q, v) and outputs 
    the Cartesian states (pose and twist) of the CoM, Left Foot, and Right Foot.
    """
    def __init__(self, plant: MultibodyPlant):
        super().__init__()
        self.plant = plant
        self.plant_context = self.plant.CreateDefaultContext()
        
        # Cache the frames to avoid looking them up in the loop
        self.base_frame = self.plant.GetFrameByName("torso") 
        self.lfoot_frame = self.plant.GetFrameByName("L_foot_contact")
        self.rfoot_frame = self.plant.GetFrameByName("R_foot_contact")
        
        # =====================================================================
        # INPUT PORTS
        # =====================================================================
        # Expects a vector of size 37: [q (19), v (18)]
        self.robot_states_port = self.DeclareVectorInputPort(
            "robot_states", BasicVector(37)
        )
        
        # =====================================================================
        # OUTPUT PORTS
        # Format for all outputs: [roll, pitch, yaw, x, y, z, wx, wy, wz, vx, vy, vz] (Size 12)
        # =====================================================================
        self.DeclareVectorOutputPort("com_states", BasicVector(12), self.CalcCoMStates)
        self.DeclareVectorOutputPort("lfoot_states", BasicVector(12), self.CalcLFootStates)
        self.DeclareVectorOutputPort("rfoot_states", BasicVector(12), self.CalcRFootStates)

    def _UpdatePlantContext(self, context):
        """Helper function to update the internal plant context from the input port."""
        robot_states = self.robot_states_port.Eval(context)
        # print(f"Received robot states: {robot_states}")
        q = robot_states[0:19]
        v = robot_states[19:37]
        
        self.plant.SetPositions(self.plant_context, q)
        self.plant.SetVelocities(self.plant_context, v)
        
        return robot_states

    def CalcCoMStates(self, context, output):
        robot_states = self._UpdatePlantContext(context)
        
        # 1. Position & Linear Velocity
        position = self.plant.CalcCenterOfMassPositionInWorld(self.plant_context)
        linear_velocity = self.plant.CalcCenterOfMassTranslationalVelocityInWorld(self.plant_context)
        
        # 2. Orientation (Tracking the torso frame)
        W_T_torso = self.base_frame.CalcPoseInWorld(self.plant_context)
        orientation = RollPitchYaw(W_T_torso.rotation()).vector()
        
        # 3. Angular Velocity (from generalized velocities)
        angular_velocity = robot_states[19:22]
        
        # Concatenate into size 12 vector
        com_state_vector = np.concatenate((orientation, position, angular_velocity, linear_velocity))
        output.SetFromVector(com_state_vector)

    def _CalcFootStates(self, frame, output):
        """Helper to calculate states for a specific foot frame."""
        # 1. Pose
        W_T_foot = frame.CalcPoseInWorld(self.plant_context)
        orientation = RollPitchYaw(W_T_foot.rotation()).vector()
        position = W_T_foot.translation()
        
        # 2. Twist / Spatial Velocity
        spatial_velocity = frame.CalcRelativeSpatialVelocityInWorld(
            self.plant_context, self.plant.world_frame()
        )
        angular_vel = spatial_velocity.rotational().copy()
        linear_vel = spatial_velocity.translational().copy()
        
        # Apply noise threshold (kept from your original code)
        threshold = 1e-03
        angular_vel[np.abs(angular_vel) < threshold] = 0.0
        linear_vel[np.abs(linear_vel) < threshold] = 0.0
        
        # Concatenate into size 12 vector
        foot_state_vector = np.concatenate((orientation, position, angular_vel, linear_vel))
        output.SetFromVector(foot_state_vector)

    def CalcLFootStates(self, context, output):
        self._UpdatePlantContext(context)
        self._CalcFootStates(self.lfoot_frame, output)

    def CalcRFootStates(self, context, output):
        self._UpdatePlantContext(context)
        self._CalcFootStates(self.rfoot_frame, output)