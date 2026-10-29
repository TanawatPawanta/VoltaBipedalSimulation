import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem


class SamplingU(LeafSystem):
    def __init__(self, dt):
        super().__init__()
        self.dt = dt
        self.sampling_data = np.zeros(12)

        # Declare an output port 
        self.input_port_0 = self.DeclareVectorInputPort(
            "input_0",
            BasicVector(12)  
        )
        self.DeclareVectorOutputPort(
            "output_0",
            BasicVector(12),
            self.CalcOutput_0
        )
        # Declare periodic events
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=self.dt,  
            offset_sec=0.0,  
            update=self.DiscreteUpdate)

    def DiscreteUpdate(self, context, discrete_state):
        self.sampling_data = self.input_port_0.Eval(context)
        

    def CalcOutput_0(self, context, output):
        output.SetFromVector(self.sampling_data)