from pydrake.systems.framework import BasicVector, LeafSystem


class Sum(LeafSystem):
    def __init__(self, port_shape:int):
        super().__init__()
        self.port_shape = port_shape
        # Declare an output port 
        self.input_port_0 = self.DeclareVectorInputPort(
            "input A",
            BasicVector(self.port_shape)  
        )
        self.input_port_1 = self.DeclareVectorInputPort(
            "input B",
            BasicVector(self.port_shape)  
        )
        self.DeclareVectorOutputPort(
            "output A+B",
            BasicVector(self.port_shape),
            self.CalcOutput_0
        )

    def CalcOutput_0(self, context, output):
        out = self.input_port_0.Eval(context) + self.input_port_1.Eval(context)
        output.SetFromVector(out)