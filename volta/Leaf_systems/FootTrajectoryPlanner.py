import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem

from volta.planning.foot_trajectory_generator import FootTrajectoryGenerator


class FootTrajectoryPlanner(LeafSystem):
    def __init__(self, dt, footsteps_planner, initial, params):
        super().__init__()
        self.dt = dt
        self.foot_trajectory_gen = FootTrajectoryGenerator(initial=initial,
                                                            footstep_planner=footsteps_planner,
                                                            params=params)
        self.timestep = 0

        # Declare an output port 
        # self.input_port_0 = self.DeclareVectorInputPort(
        #     "input_0",
        #     BasicVector(12)  
        # )
        self.DeclareVectorOutputPort(
            "Foot Trajectory",
            BasicVector(12),
            self.CalcOutput_0
        )
        # Declare periodic events
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=self.dt,  
            offset_sec=0.0,  
            update=self.DiscreteUpdate)
        

    def DiscreteUpdate(self, context, discrete_state):
        # Update discrete variables
        pass 

    def CalcOutput_0(self, context, output):
        out = np.zeros([1,12])
        output.SetFromVector(out[0])