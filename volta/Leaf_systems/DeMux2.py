from pydrake.systems.framework import BasicVector, LeafSystem


class DeMux2(LeafSystem):
    def __init__(self, input_size):
        super().__init__()
        self.input_size = input_size
        self.half_ind = int(self.input_size*0.5)

        # Declare an output port 
        self.input_port_1 = self.DeclareVectorInputPort(
            "Vector in",
            BasicVector(self.input_size)  
        )
        self.DeclareVectorOutputPort(
            "Vector out 1",
            BasicVector(self.half_ind),
            self.CalcOutput_1
        )
        self.DeclareVectorOutputPort(
            "Vector out 2",
            BasicVector(self.half_ind),
            self.CalcOutput_2
        )

    def CalcOutput_1(self, context, output):
        vec_in = self.input_port_1.Eval(context)
        out = vec_in[0:self.half_ind]
        output.SetFromVector(out)

    def CalcOutput_2(self, context, output):
        vec_in = self.input_port_1.Eval(context)
        out = vec_in[self.half_ind:]
        output.SetFromVector(out)