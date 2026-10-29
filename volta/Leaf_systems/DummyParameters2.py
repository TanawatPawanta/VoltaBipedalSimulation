import numpy as np
from pydrake.all import LeafSystem


class DummyParameters2(LeafSystem):
    def __init__(self):
        super().__init__()  # Don't forget to initialize the base class.

        self.DeclareVectorOutputPort(name="dummy torque",
                                     size=12,
                                     calc=self.Output_gen_command)

    def Output_gen_command(self, context, output):
        out = np.array([-0.73156, -0.07558,  0.91371,  3.31031,  0.18023,  0.11699,
       -0.64257,  0.00117, -0.68   , -3.36963, -0.16604,  0.11864])
        output.SetFromVector(out)

    

