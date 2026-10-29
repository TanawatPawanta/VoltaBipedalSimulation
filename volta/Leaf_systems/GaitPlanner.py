import copy
import time

import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import BasicVector, LeafSystem
from pydrake.trajectories import PiecewisePolynomial

from volta.planning.foot_trajectory_generator import FootTrajectoryGenerator
from volta.planning.footsteps_planner import FootstepsPlanner
from volta.planning.ismpc import Ismpc
from volta.robot.states import States


class GaitPlanner(LeafSystem):
    """IS-MPC gait planner: CoM/ZMP plan, swing-foot trajectories and contact schedule.

    Runs every `params['mpc_time_step']` s on the footstep plan from `footsteps_planner`.
    Inputs:  0 CoM state [12], 1 ZMP [3]
    Outputs: 0 desired CoM [18], 1 desired lfoot [12], 2 desired rfoot [12],
             3 contact flags [left, right], 4 desired ZMP [3], 5 ZMP velocity (MPC input u) [3]
    12-vectors are [rpy, xyz, angular vel, linear vel]; 18-vectors append [angular acc, linear acc].
    """
    def __init__(self, plant:MultibodyPlant, footsteps_planner:FootstepsPlanner, initial_states:dict, params:dict, enable:bool=True):
        super().__init__()
        self.enable = enable
        self.dt = params['mpc_time_step']
        self.plant = plant
        self.plant_context = self.plant.CreateDefaultContext()
        self.params = params
        self.footsteps_planner = footsteps_planner
        self.mpc = Ismpc(
                        initial=initial_states, 
                        footstep_planner=self.footsteps_planner, 
                        params=params
                        )
        self.timesteps = 0
        self.current_states = copy.deepcopy(initial_states)
        self.desired_com_state = States(name="desired com state")
        self.desired_com_state.position[2] = self.params['h']
        self.desired_lfoot_state = States(name="lfoot state")
        self.desired_lfoot_state.position = np.array([0.0,0.125,0.0])
        self.desired_rfoot_state = States(name="rfoot state")
        self.desired_rfoot_state.position = np.array([0.0,-0.125,0.0])
        self.desired_zmp_state = States(name="desired zmp")
        self.contact = np.array([1,1]) # ds,ls
        self.foot_trajectory_gen = FootTrajectoryGenerator(initial=initial_states,
                                                            footstep_planner=footsteps_planner,
                                                            params=params)
        self.previous_contact = [1,1]
        self.orientation_traj_start_orientation = np.zeros(3)
        self.orientation_traj_start_time = 0.0
        self.orientation_traj_start_orientation = self.desired_com_state.orientation
        target_orientation = self.desired_lfoot_state.orientation if self.contact[0] == 1 else self.desired_rfoot_state.orientation
        step_index = self.footsteps_planner.get_step_index_at_time(self.timesteps)
        self.trajectory_current_time = 0.0
        self.CoM_traj = self.InitP2PTrajectories(p_start=self.current_states['com']['pos'], 
                                       p_final=np.array([self.current_states['com']['pos'][0],
                                                         self.current_states['com']['pos'][1]+0.05,
                                                         self.current_states['com']['pos'][2]-0.0]), 
                                       v_start=np.zeros(3), 
                                       v_final=np.zeros(3), 
                                       total_time=3.0)
        self.com_ori_trajs = self.InitP2PTrajectories(p_start=self.orientation_traj_start_orientation,
                                                            p_final=target_orientation,
                                                            v_start=np.zeros(3),
                                                            v_final=np.zeros(3),
                                                            total_time=self.params['ss_duration']*self.dt)

        # Declare an output port 
        # self.robot_states_feedback_port = self.DeclareVectorInputPort(
        #     "Robot states feedback(q,qd)",
        #     BasicVector(37)  
        # )
        self.CoM_feedback_port = self.DeclareVectorInputPort(
            "CoM states feedback",
            BasicVector(12)  
        )
        self.zmp_feedback_port = self.DeclareVectorInputPort(
            "ZMP",
            BasicVector(3)  
        )
        self.DeclareVectorOutputPort(
            "CoM Trajectory",
            BasicVector(18),
            self.CalcOutput_0
        )
        self.DeclareVectorOutputPort(
            "L Foot Trajectory",
            BasicVector(12),
            self.CalcOutput_1
        )
        self.DeclareVectorOutputPort(
            "R Foot Trajectory",
            BasicVector(12),
            self.CalcOutput_2
        )
        self.DeclareVectorOutputPort(
            "Contact",
            BasicVector(2),
            self.CalcOutput_3
        )
        self.DeclareVectorOutputPort(
            "Desired ZMP",
            BasicVector(3),
            self.CalcOutput_4
        )       
        self.DeclareVectorOutputPort(
            "ZMP velocity",
            BasicVector(3),
            self.CalcOutput_5
        )        
        # Declare periodic events
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=self.dt,  
            offset_sec=0.0,  
            update=self.DiscreteUpdate)
        
    def UpdateState(self,context):
        # robot_states = self.robot_states_feedback_port.Eval(context)
        current_CoM_state = self.CoM_feedback_port.Eval(context)
        self.current_states['com']['pos'] = current_CoM_state[3:6]
        self.current_states['com']['vel'] = current_CoM_state[9:12]

        zmp = self.zmp_feedback_port.Eval(context)
        # generalized_position = robot_states[0:19]
        # generalized_velocity = robot_states[19:37]
        # self.plant.SetPositions(self.plant_context, generalized_position) 
        # self.plant.SetVelocities(self.plant_context, generalized_velocity)
        # self.current_states['com']['pos']=self.plant.CalcCenterOfMassPositionInWorld(context=self.plant_context)
        # self.current_states['com']['vel']=self.plant.CalcCenterOfMassTranslationalVelocityInWorld(context=self.plant_context)
        self.current_states['zmp']['pos']=zmp

    def DiscreteUpdate(self, context, discrete_state):
        # Update discrete variables
        self.UpdateState(context=context)
        # print("="*50)
        # print(f"mpc timestep {self.timesteps}")
        # print("com pos",self.current_states['com']['pos'])
        # print("com vel",self.current_states['com']['vel'])
        # print("zmp pos",self.current_states['zmp']['pos'])
        # start_time = time.time()
        if self.enable:
            
            lip_state, contact = self.mpc.solve(self.current_states, self.timesteps)
            self.desired_com_state.position = lip_state['com']['pos']
            self.desired_com_state.position[2] = self.params['h']
            self.desired_com_state.linear_velocity = lip_state['com']['vel']
            self.desired_com_state.linear_acceleration = lip_state['com']['acc']
            self.desired_com_state.linear_velocity[2] = 0.0
            self.desired_com_state.linear_acceleration[2] = 0.0
            self.desired_zmp_state.position = lip_state['zmp']['pos']
            self.desired_zmp_state.linear_velocity = lip_state['zmp']['vel']

            if contact == 'ds':
                self.contact = [1,1]
            else:
                if contact == 'lfoot':
                    self.contact = [1,0]
                else:
                    self.contact = [0,1]
            feet_traj = self.foot_trajectory_gen.generate_feet_trajectories_at_time(time=self.timesteps)
            self.desired_lfoot_state.orientation        = feet_traj['lfoot']['pos'][0:3]
            self.desired_lfoot_state.position           = feet_traj['lfoot']['pos'][3:]
            self.desired_lfoot_state.angular_velocity   = feet_traj['lfoot']['vel'][0:3]
            self.desired_lfoot_state.linear_velocity    = feet_traj['lfoot']['vel'][3:]

            self.desired_rfoot_state.orientation        = feet_traj['rfoot']['pos'][0:3]
            self.desired_rfoot_state.position           = feet_traj['rfoot']['pos'][3:]
            self.desired_rfoot_state.angular_velocity   = feet_traj['rfoot']['vel'][0:3]
            self.desired_rfoot_state.linear_velocity    = feet_traj['rfoot']['vel'][3:]

            # On contact change
            if not np.array_equal(self.contact, self.previous_contact):
                self.orientation_traj_start_time = self.timesteps
                self.orientation_traj_start_orientation = self.desired_com_state.orientation
                target_orientation = self.desired_lfoot_state.orientation if self.contact[0] == 1 else self.desired_rfoot_state.orientation
                step_index = self.footsteps_planner.get_step_index_at_time(self.timesteps)
                ss_duration = self.footsteps_planner.plan[step_index]['ss_duration']
                self.com_ori_trajs = self.InitP2PTrajectories(p_start=self.orientation_traj_start_orientation,
                                                        p_final=target_orientation,
                                                        v_start=np.zeros(3),
                                                        v_final=np.zeros(3),
                                                        total_time=ss_duration*self.dt)

            # If in single support, interpolate, otherwise hold
            if self.contact[0] != self.contact[1]: # single support
                time_in_traj = (self.timesteps - self.orientation_traj_start_time)*self.dt
                self.desired_com_state.orientation[2]       = self.com_ori_trajs[2].value(time_in_traj)[0][0]
                self.desired_com_state.angular_velocity[2]  = self.com_ori_trajs[2].EvalDerivative(time_in_traj, 1)[0][0]
            else: # double support
                self.desired_com_state.angular_velocity = np.zeros(3)
                self.desired_com_state.angular_acceleration = np.zeros(3)
            
            self.previous_contact = self.contact
            self.timesteps += 1
            end_time = time.time()
        # else:
        #     if self.trajectory_current_time<3.0:
        #         desired_x_c = []
        #         desired_xd_c = []
        #         desired_xdd_c = []
        #         for i in range(len(self.CoM_traj)):
        #             desired_x_c.append(self.CoM_traj[i].value(self.trajectory_current_time)[0][0])
        #             desired_xd_c.append(self.CoM_traj[i].EvalDerivative(self.trajectory_current_time, 1)[0][0])
        #             desired_xdd_c.append(self.CoM_traj[i].EvalDerivative(self.trajectory_current_time, 2)[0][0])
        #         self.desired_com_state.position = desired_x_c
        #         self.desired_com_state.position[2] = self.params['h']
        #         self.desired_com_state.linear_velocity = lip_state['com']['vel']
        #         self.desired_com_state.linear_acceleration = lip_state['com']['acc']
        #         self.desired_com_state.linear_velocity[2] = 0.0
        #         self.desired_com_state.linear_acceleration[2] = 0.0
        #         self.desired_zmp_state.position = lip_state['zmp']['pos']
        #         self.desired_zmp_state.linear_velocity = lip_state['zmp']['vel']
        #         self.trajectory_current_time += self.dt
        # print(f"contact {self.contact}")
        # print(f"desired lfoot state {self.desired_lfoot_state.GetStateVector(acc=False)}")
        # print(f"desired rfoot state {self.desired_rfoot_state.GetStateVector(acc=False)}")

        # print(f"Time consumed in gait planner: {end_time-start_time}")      

    def CalcOutput_0(self, context, output):
        # output.SetFromVector(self.desired_com_state.GetLinearStateVector(acc=True))
        output.SetFromVector(self.desired_com_state.GetStateVector(acc=True))

    def CalcOutput_1(self, context, output):
        output.SetFromVector(self.desired_lfoot_state.GetStateVector(acc=False))

    def CalcOutput_2(self, context, output):
        output.SetFromVector(self.desired_rfoot_state.GetStateVector(acc=False))

    def CalcOutput_3(self, context, output):
        output.SetFromVector(self.contact)

    def CalcOutput_4(self, context, output):
        output.SetFromVector(self.desired_zmp_state.position)

    def CalcOutput_5(self, context, output):
        output.SetFromVector(self.desired_zmp_state.linear_velocity)
    
    def InitP2PTrajectories(self,p_start, p_final, v_start, v_final, total_time):
        time = np.array([0.0, total_time])
        trajs = []
        for i in range(len(p_start)):
            P = np.array([[p_start[i],p_final[i]]])
            V = np.array([[v_start[i],v_final[i]]])
            trajs.append(PiecewisePolynomial.CubicHermite(time, P, V))
        return trajs