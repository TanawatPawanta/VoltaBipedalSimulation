import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem


class RateTransition(LeafSystem):
    def __init__(self, dt):
        super().__init__()
        self.dt = dt
        self.qd_desire_m1 = np.zeros(12)    # x_m1 mean x[n-1] 
        self.qd_feedback_m1 = np.zeros(12)

        self.qd_desire = np.zeros(12)
        self.qd_feedback = np.zeros(12)

        self.qdd_desire = np.zeros(12)
        self.qdd_estimate = np.zeros(12)

        self.qd_reference_port = self.DeclareVectorInputPort(
            "qd reference",
            BasicVector(12)
        )
        self.qd_u_port = self.DeclareVectorInputPort(
            "qd u",
            BasicVector(12)  
        )
        self.qd_feedback_port = self.DeclareVectorInputPort(
            "qd feedback",
            BasicVector(12)  
        )
        self.DeclareVectorOutputPort(
            "estimate",
            BasicVector(24),
            self.CalcEstimate
        )
        self.DeclareVectorOutputPort(
            "desire",
            BasicVector(24),
            self.CalcDesire
        )
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec= self.dt,  
            offset_sec=0.0,  
            update=self.DiscreteUpdate)
    
    def DiscreteUpdate(self,context,discrete_state):
        # Update [n]
        self.qd_desire = self.qd_reference_port.Eval(context) + self.qd_u_port.Eval(context)
        self.qd_feedback = self.qd_feedback_port.Eval(context)
        
        # Use backward euler for update qdd
        self.qdd_desire = (self.qd_desire - self.qd_desire_m1)/self.dt
        self.qdd_estimate = (self.qd_feedback - self.qd_feedback_m1)/self.dt

        # Update [n-1]
        self.qd_desire_m1 = self.qd_desire
        self.qd_feedback_m1 = self.qd_feedback
        pass

    def CalcEstimate(self,context, output):
        # self.qd_feedback = self.qd_feedback_port.Eval(context)
        out = np.concatenate((np.array([self.qd_feedback]), np.array([self.qdd_estimate])),axis=1)
        output.SetFromVector(out[0])

    def CalcDesire(self,context, output):
        # self.qd_desire = self.qd_reference_port.Eval(context) + self.qd_u_port.Eval(context)
        out = np.concatenate((np.array([self.qd_desire]), np.array([self.qdd_desire])),axis=1)
        output.SetFromVector(out[0])