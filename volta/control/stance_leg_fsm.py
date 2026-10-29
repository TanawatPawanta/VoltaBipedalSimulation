"""State machine for the stance-leg / walking phase."""
from statemachine import State, StateMachine


class StanceLegStatemachine(StateMachine):
    """A state machine to manage the stance leg during walking.

    States:
        L_stance: Left leg is the stance leg.
        R_stance: Right leg is the stance leg.

    Transitions:
        to_R_stance: Transition from L_stance to R_stance.
        to_L_stance: Transition from R_stance to L_stance.
    """
    # States
    double_stance = State("double_stance", initial=True)
    L_stance = State("L_stance")
    R_stance = State("R_stance")
    stance_transition= State("stance_transition")    
    # Transitions
    to_R_stance_from_transition = stance_transition.to(R_stance)
    to_L_stance_from_transition = stance_transition.to(L_stance)
    to_double_stance_from_transition = stance_transition.to(double_stance)
    to_stance_transition_from_L = L_stance.to(stance_transition)
    to_stance_transition_from_R = R_stance.to(stance_transition)
    to_stance_transition_from_double = double_stance.to(stance_transition)
    def __init__(self,dt=0.01,transition_duration=0.5):
        super().__init__()
        self.current_leg_state = "double_stance"
        self.next_leg_state = "double_stance"
        self.transition_rate = 0.0
        self.transition_duration = transition_duration # seconds
        self.transition_local_time = 0.0
        self.dt = dt
        self.leg_phase = [1,1]  # [L_leg_phase, R_leg_phase], 1 for stance, 0 for swing

    def SetNextLegState(self, next_leg_state: str):
        self.next_leg_state = next_leg_state
        self._ProcessNextlegState()
    
    def _ProcessNextlegState(self):
        # print("Current leg state:", self.current_leg_state)
        # print("Next leg state:", self.next_leg_state)
        # print("Current state:", self.current_state.id)
        # print("States change:",self.current_state.id != self.next_leg_state)
        if self.current_state.id != self.next_leg_state:
            match self.current_state:
                case self.L_stance:
                    # print("Transitioning from L stance")
                    self.to_stance_transition_from_L()
                case self.R_stance:
                    # print("Transitioning from R stance")
                    self.to_stance_transition_from_R()
                case self.double_stance:
                    # print("Transitioning from double stance")
                    self.to_stance_transition_from_double()
    
    @stance_transition.enter
    def enter_stance_transition(self):
        self.current_leg_state = "transition"
        self.transition_rate = 0.0

    @stance_transition.exit
    def exit_stance_transition(self):
        pass

    @L_stance.enter
    def enter_L_stance(self):
        self.current_leg_state = "L_stance"
        self.leg_phase = [1,0]
        # print("Entered L_stance")

    @R_stance.enter
    def enter_R_stance(self):
        self.current_leg_state = "R_stance"
        self.leg_phase = [0,1]
        # print("Entered R_stance")

    @double_stance.enter
    def enter_double_stance(self):
        self.current_leg_state = "double_stance"
        self.leg_phase = [1,1]
        # print("Entered double_stance")

    def UpdateStanceLegStateMachine(self):
        if self.current_state == self.stance_transition:
            self.transition_local_time += self.dt
            self.transition_rate = self.transition_local_time / self.transition_duration
            # print("Transition local time ", self.transition_local_time)
            # print("Transition rate ", self.transition_rate)
            if self.transition_rate >= 1.0:
                match self.next_leg_state:
                    case "L_stance":
                        self.to_L_stance_from_transition()
                    case "R_stance":
                        self.to_R_stance_from_transition()
                    case "double_stance":
                        self.to_double_stance_from_transition()
                self.transition_local_time = 0.0
                self.transition_rate = 0.0
                self.current_leg_state = self.next_leg_state

    def GetCurrentStateName(self)->str:
        return self.current_state.id
    
    def GetTransitionRate(self)->float:
        return self.transition_rate

    def GetlegPhase(self)->list:
        return self.leg_phase
    
    def GetCurrentLegState(self)->str:
        return self.current_leg_state