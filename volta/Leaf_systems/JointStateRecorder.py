import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem


class JointStateRecorder(LeafSystem):
    def __init__(self, plant,init_com_position=np.zeros(3)):
        super().__init__()
        self.plant = plant
        self.plant_context = plant.CreateDefaultContext()
        self.num_positions = self.plant.num_positions()
        self.CoM_posotion = init_com_position
        self.CoM_posotion_m1 = init_com_position
        self.DefinedVariables()
        # Declare input port to receive the plant state vector
        self.plant_state_input_port = self.DeclareVectorInputPort(
            "plant_state",
            BasicVector(self.plant.num_multibody_states())
        )
        # Declare separate output ports for positions and velocities
        self.DeclareVectorOutputPort(
            "body_positions",
            BasicVector(7),
            self.CalcBodyPositionsOutput
        )
        self.DeclareVectorOutputPort(
            "body_velocities",
            BasicVector(6),
            self.CalcBodyVelocitiesOutput
        )
        self.DeclareVectorOutputPort(
            "joint_positions",
            BasicVector(self.joint_num),
            self.CalcJointPositionsOutput
        )
        self.DeclareVectorOutputPort(
            "joint_velocities",
            BasicVector(self.joint_num),
            self.CalcJointVelocitiesOutput
        )
        self.DeclareVectorOutputPort(
            "joint_states",
            BasicVector(self.joint_num*2),
            self.CalcJointStatesOutput
        )
        self.DeclareVectorOutputPort(
            "CoM Linear Velocity",
            BasicVector(3),
            self.CalcCoMLinearVelOutput
        )  
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=self.dt,  
            offset_sec=0.0,  
            update=self.DiscreteUpdate)
        
    def DiscreteUpdate(self, context, discrete_state):
        state = self.plant_state_input_port.Eval(context)
        positions = state[:self.num_positions]
        # print("positions(JSR):", positions)
        self.plant.SetPositions(
            self.plant_context,
            positions
        )
        self.CoM_posotion = self.plant.CalcCenterOfMassPositionInWorld(self.plant_context)
        # print("CoM position(JSR):", self.CoM_posotion)
        # print("CoM position m1(JSR):", self.CoM_posotion_m1)
        self.CoM_velocity = (self.CoM_posotion - self.CoM_posotion_m1) / self.dt
        # print("CoM velocity(JSR):", self.CoM_velocity)
        self.CoM_posotion_m1 = self.CoM_posotion

    def DefinedVariables(self):
        self.dt = 0.01
        self.time_data = []
        self.joint_num = 12
        self.CoM_velocity = np.zeros(3)

    def CalcBodyPositionsOutput(self, context, output):
        state = self.plant_state_input_port.Eval(context)
        positions = state[:self.num_positions]
        if self.num_positions == 12: # fixed base
            output.SetFromVector(np.zeros(7))
        else:   # floating base
            output.SetFromVector(positions[:7])
    
    def CalcBodyVelocitiesOutput(self, context, output):
        state = self.plant_state_input_port.Eval(context)
        velocities = state[self.num_positions:]
        if self.num_positions == 12: # fixed base
            output.SetFromVector(np.zeros(6))
        else:   # floating base
            output.SetFromVector(velocities[:6])
        
    def CalcJointPositionsOutput(self, context, output):
        state = self.plant_state_input_port.Eval(context)
        positions = state[:self.num_positions]
        if self.num_positions == 12: # fixed base
            output.SetFromVector(positions)
        else:   # floating base
            output.SetFromVector(positions[7:])

    def CalcJointVelocitiesOutput(self, context, output):
        state = self.plant_state_input_port.Eval(context)
        velocities = state[self.num_positions:]
        if self.num_positions == 12: # fixed base
            output.SetFromVector(velocities)
        else:   # floating base
            output.SetFromVector(velocities[6:])

    def CalcJointStatesOutput(self, context, output):
        state = self.plant_state_input_port.Eval(context)
        positions = np.array([state[:self.num_positions]])
        velocities = np.array([state[self.num_positions:]])
        if self.num_positions == 12: # fixed base
            out = np.concatenate((positions,velocities),axis=1)
            output.SetFromVector(out[0])
        else:   # floating base
            out = np.concatenate((np.array([positions[0,7:]]), np.array([velocities[0,6:]])),axis=1)
            output.SetFromVector(out[0])
    
    def CalcCoMLinearVelOutput(self, context, output):
        output.SetFromVector(self.CoM_velocity)
    
