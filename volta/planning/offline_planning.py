"""Offline LIP CoM / joint-trajectory planning (earlier pipeline)."""
import matplotlib.pyplot as plt
import numpy as np
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput
from pydrake.trajectories import PiecewisePolynomial

from volta.Leaf_systems.CubicTrajectory3D import CubicTrajectory3D
from volta.Leaf_systems.DummyParameters2 import DummyParameters2
from volta.Leaf_systems.LIPMGaitGeneration import LIPMGaitGeneration
from volta.robot.kinematics import (
    CalcInverseDiffKinematics,
    CalcLegInverseKinematics,
    InitialCalcLegSpatialJacobianMatrix,
    SolveIKWithComPositionConstraint,
)
from volta.utils import DefinePlant, RigidTransformToMatrix


class LIPJointTrajectory:
    def __init__(self, model_series:str, model_name:str, step_length:float, step_time:float,step_height:float, z_com:float,desire_torso_high:float, y_0:float):
        self.model_series = model_series
        self.model_name = model_name
        self.plant, self.robot_index = DefinePlant(model_name=model_name,
                                                   is_fixed=True,
                                                   fixed_frame="torso")
        self.step_length = step_length
        self.step_time = step_time
        self.step_height = step_height
        self.z_com = z_com
        self.y_0 = y_0
        self.desire_torso_height = desire_torso_high
        self.st_P_c = None
        self.st_V_c = None
        self.st_Vd_c = None
        self.sw0_P_sw = None
        self.sw0_V_sw = None
        # Walking joint trajectory
        self.L_leg_joints_positions = None
        self.R_leg_joints_positions = None
        self.L_leg_joints_velocities = None
        self.R_leg_joints_velocities = None
        # Prepare to walk joints trajectory
        self.L_leg_init_joints_positions = None
        self.R_leg_init_joints_positions = None
        self.L_leg_init_joints_velocities = None
        self.R_leg_init_joints_velocities = None
        self.manipulability_L = None
        self.manipulability_R = None

    def SetParameters(self, step_length:float, step_time:float, z_com:float, y_0:float):
        self.step_length = step_length
        self.step_time = step_time
        self.z_com = z_com
        self.y_0 = y_0
        print("Parameters updated !")

    def GetJointsTrajectory(self,phase:str="walking",is_left:bool=True):
        match phase:
            case "walking":
                if is_left:
                    if self.L_leg_joints_positions is None or self.L_leg_joints_velocities is None:
                        raise ValueError("Left leg trajectory has not been calculated yet. Please call CalcTrajectory() first.")
                    return self.L_leg_joints_positions, self.L_leg_joints_velocities
                else:
                    if self.R_leg_joints_positions is None or self.R_leg_joints_velocities is None:
                        raise ValueError("Right leg trajectory has not been calculated yet. Please call CalcTrajectory() first.")
                    return self.R_leg_joints_positions, self.R_leg_joints_velocities
            case "init":
                if is_left:
                    if self.L_leg_init_joints_positions is None or self.L_leg_init_joints_velocities is None:
                        raise ValueError("Left leg trajectory has not been calculated yet. Please call CalcTrajectory() first.")
                    return self.L_leg_init_joints_positions, self.L_leg_init_joints_velocities
                else:
                    if self.R_leg_init_joints_positions is None or self.R_leg_init_joints_velocities is None:
                        raise ValueError("Right leg trajectory has not been calculated yet. Please call CalcTrajectory() first.")
                    return self.R_leg_init_joints_positions, self.R_leg_init_joints_velocities
    
    def ListParameters(self):
        print("LIP Joint Trajectory Parameters:")
        print("Model Series: ", self.model_series)
        print("Model Name: ", self.model_name)
        print("Step Length [m]: ", self.step_length)
        print("Step Height [m]: ", self.step_height)
        print("Step Time [s]: ", self.step_time)    
        print("Z CoM [m]: ", self.z_com)
        print("torso high [m]:",self.desire_torso_height)
        print("Y_0 [m]: ", self.y_0)

    def DisplayJointTrajectories(self, phase:str="walking", is_left:bool=True):
        """Display joints trajectory 

        Args:
            phase (str, optional): "init", "walking" or "full". Defaults to "walking".
            is_left (bool, optional): Dispaly left or right leg. Defaults to True.
        """
        if self.model_series == "Hanuman":
            name = ["J_hip_y","J_hip_r","J_hip_p","J_knee","J_ankle_p","J_ankle_r"]
        else:
            name = ["J_hip_p","J_hip_r","J_hip_y","J_knee","J_ankle_p","J_ankle_r"]
        fig, axs = plt.subplots(3, 2, figsize=(14, 10))
        for idx, ax in enumerate(axs.flat):
            temp = name[idx]
            match phase:
                case "init":
                    temp2 = "Init"
                    if is_left:
                        ax.plot(self.L_leg_init_joints_positions[:, idx], label=f"L {temp} q")
                        ax.plot(self.L_leg_init_joints_velocities[:, idx], label=f"L {temp} qd")

                    else:
                        ax.plot(self.R_leg_init_joints_positions[:, idx], label=f"R {temp} q") 
                        ax.plot(self.R_leg_init_joints_velocities[:, idx], label=f"R {temp} qd")

                case "walking":
                    temp2 = "Walking"
                    if is_left:
                        if self.model_series == "Hanuman":
                            if idx == 1 or idx ==5:
                                ax.plot(np.concatenate((self.L_leg_joints_positions[:, idx],
                                                        -1.0*self.R_leg_joints_positions[:, idx])), label=f"L {temp} q")
                                ax.plot(np.concatenate((self.L_leg_joints_velocities[:, idx],
                                                        -1.0*self.R_leg_joints_velocities[:, idx])), label=f"L {temp} qd")
                            else:
                                ax.plot(np.concatenate((self.L_leg_joints_positions[:, idx],
                                                        self.R_leg_joints_positions[:, idx])), label=f"L {temp} q")  
                                ax.plot(np.concatenate((self.L_leg_joints_velocities[:, idx],
                                                        self.R_leg_joints_velocities[:, idx])), label=f"L {temp} qd") 
                        else:                             
                            ax.plot(np.concatenate((self.L_leg_joints_positions[:, idx],
                                                    self.R_leg_joints_positions[:, idx])), label=f"L {temp} q") 
                            ax.plot(np.concatenate((self.L_leg_joints_velocities[:, idx],
                                                    -1.0*self.R_leg_joints_velocities[:, idx])), label=f"L {temp} qd")
                    else:
                        if self.model_series == "Hanuman":
                            if idx == 1 or idx ==5:
                                ax.plot(np.concatenate((self.R_leg_joints_positions[:, idx],
                                                        -1.0*self.L_leg_joints_positions[:, idx])), label=f"R {temp} q")
                                ax.plot(np.concatenate((self.R_leg_joints_velocities[:, idx],
                                                        -1.0*self.L_leg_joints_velocities[:, idx])), label=f"R {temp} qd")
                            else:
                                ax.plot(np.concatenate((self.R_leg_joints_positions[:, idx],
                                                        self.L_leg_joints_positions[:, idx])), label=f"R {temp} q")  
                                ax.plot(np.concatenate((self.R_leg_joints_velocities[:, idx],
                                                        self.L_leg_joints_velocities[:, idx])), label=f"R {temp} qd") 
                        else:                             
                            ax.plot(np.concatenate((self.R_leg_joints_positions[:, idx],
                                                    self.L_leg_joints_positions[:, idx])), label=f"R {temp} q") 
                            ax.plot(np.concatenate((self.R_leg_joints_velocities[:, idx],
                                                    -1.0*self.L_leg_joints_velocities[:, idx])), label=f"R {temp} qd")
                
                case "full":
                    temp2 = "Full"
                    if is_left:
                        if self.model_series == "Hanuman":
                            if idx == 1 or idx ==5:
                                ax.plot(np.concatenate((self.L_leg_init_joints_positions[:, idx],
                                                        self.L_leg_joints_positions[:, idx],
                                                        -1.0*self.R_leg_joints_positions[:, idx])), label=f"L {temp} q")
                                ax.plot(np.concatenate((self.L_leg_init_joints_velocities[:, idx],
                                                        self.L_leg_joints_velocities[:, idx],
                                                        -1.0*self.R_leg_joints_velocities[:, idx])), label=f"L {temp} qd")
                            else:
                                ax.plot(np.concatenate((self.L_leg_init_joints_positions[:, idx],
                                                        self.L_leg_joints_positions[:, idx],
                                                        self.R_leg_joints_positions[:, idx])), label=f"L {temp} q")  
                                ax.plot(np.concatenate((self.L_leg_init_joints_velocities[:, idx],
                                                        self.L_leg_joints_velocities[:, idx],
                                                        self.R_leg_joints_velocities[:, idx])), label=f"L {temp} qd") 
                        else:                             
                            ax.plot(np.concatenate((self.L_leg_init_joints_positions[:, idx],
                                                    self.L_leg_joints_positions[:, idx],
                                                    -1.0*self.R_leg_joints_positions[:, idx])), label=f"L {temp} q")
                            ax.plot(np.concatenate((self.L_leg_init_joints_velocities[:, idx],
                                                    self.L_leg_joints_velocities[:, idx],
                                                    -1.0*self.R_leg_joints_velocities[:, idx])), label=f"L {temp} qd")
                    else:
                        if self.model_series == "Hanuman":
                            if idx == 1 or idx ==5:
                                ax.plot(np.concatenate((self.R_leg_init_joints_positions[:, idx],
                                                        self.R_leg_joints_positions[:, idx],
                                                        -1.0*self.L_leg_joints_positions[:, idx])), label=f"R {temp} q")
                                ax.plot(np.concatenate((self.R_leg_init_joints_velocities[:, idx],
                                                        self.R_leg_joints_velocities[:, idx],
                                                        -1.0*self.L_leg_joints_velocities[:, idx])), label=f"R {temp} qd")
                            else:
                                ax.plot(np.concatenate((self.R_leg_init_joints_positions[:, idx],
                                                        self.R_leg_joints_positions[:, idx],
                                                        self.L_leg_joints_positions[:, idx])), label=f"R {temp} q")  
                                ax.plot(np.concatenate((self.R_leg_init_joints_velocities[:, idx],
                                                        self.R_leg_joints_velocities[:, idx],
                                                        self.L_leg_joints_velocities[:, idx])), label=f"R {temp} qd") 
                        else:                             
                            ax.plot(np.concatenate((self.R_leg_init_joints_positions[:, idx],
                                                    self.R_leg_joints_positions[:, idx],
                                                    -1.0*self.L_leg_joints_positions[:, idx])), label=f"R {temp} q")
                            ax.plot(np.concatenate((self.R_leg_init_joints_velocities[:, idx],
                                                    self.R_leg_joints_velocities[:, idx],
                                                    -1.0*self.L_leg_joints_velocities[:, idx])), label=f"R {temp} qd")

            ax.set_xlabel("Time [s]")
            ax.set_ylabel("value")
            ax.set_title(f"{temp} Trajectory")
            ax.legend()
            ax.grid(True)
        plt.tight_layout()
        if is_left:
            fig.suptitle(f"{self.model_series} Left Leg {temp2} Joint Trajectories", fontsize=16)
        else:
            fig.suptitle(f"{self.model_series} Right Leg {temp2} Joint Trajectories", fontsize=16)
        plt.subplots_adjust(top=0.95)
        plt.show()

    def DisplayManipulability(self):
        plt.plot(self.manipulability_L, label="Left Leg Manipulability")
        plt.plot(self.manipulability_R, label="Right Leg Manipulability")
        plt.xlabel("Time Step")
        plt.ylabel("Manipulability")
        plt.title("Manipulability of Left and Right Legs")
        plt.legend()
        plt.grid()
        plt.show()

    def CalcTrajectory(self,initial_joints_position=np.zeros(12)):
        self.st_P_c, self.st_V_c, self.st_Vd_c, t0_swing, t_swing = CalcNstepsCoMTrajectory(num_step=1,
                                                                    step_length=self.step_length,
                                                                    step_time=self.step_time,
                                                                    z_com=self.z_com,
                                                                    y_0=self.y_0,
                                                                    desire_robot_height=self.desire_torso_height)
        print("t0_swing",t0_swing)
        print("t swing",t_swing)
        self.sw0_P_sw, self.sw0_V_sw = CalcSwingLegTrajectory(swing_high=self.step_height,
                                                    step_length=self.step_length,
                                                    step_time=self.step_time,
                                                    t0_swing=t0_swing,
                                                    t_swing=t_swing)
        # self.L_leg_joints_positions, self.R_leg_joints_positions, self.L_leg_joints_velocities, self.R_leg_joints_velocities, self.manipulability_L, self.manipulability_R = CalcJointsTrajectory(model_series=self.model_series,   
        #                                                                                                                                                                                             model_name=self.model_name,
        #                                                                                                                                                                                             st_P_c=self.st_P_c,
        #                                                                                                                                                                                             st_V_c=self.st_V_c,
        #                                                                                                                                                                                             sw0_P_sw=self.sw0_P_sw,
        #                                                                                                                                                                                                 sw0_V_sw=self.sw0_V_sw,
        #                                                                                                                                                                                             step_length=self.step_length,
        #                                                                                                                                                                                             y_0=self.y_0)
        # self.L_leg_init_joints_positions, self.L_leg_init_joints_velocities, self.R_leg_init_joints_positions, self.R_leg_init_joints_velocities = CalcInitialJointTrajectory(model_series=self.model_series,
        #                                                                                                                                                                                            model_name=self.model_name,
        #                                                                                                                                                                                            step_length=self.step_length,
        #                                                                                                                                                                                            target_height=self.desire_torso_height,
        #                                                                                                                                                                                            y_0=self.y_0,
        #                                                                                                                                                                                            q_initial=initial_joints_position,
        #                                                                                                                                                                                            q_final=np.concatenate((self.L_leg_joints_positions[0], self.R_leg_joints_positions[0])),
        #                                                                                                                                                                                        qd_final=np.concatenate((self.L_leg_joints_velocities[0], self.R_leg_joints_velocities[0])))
        # print("Joints trajectory calculation success!!")

    def SaveTrajectory(self, path:str, file_name:str):
        full_path = path + "/" + file_name + ".npz"
        np.savez(full_path, 
                 T = self.step_time,
                 L_leg_init_joints_positions=self.L_leg_init_joints_positions,
                 R_leg_init_joints_positions=self.R_leg_init_joints_positions,
                 L_leg_init_joints_velocities=self.L_leg_init_joints_velocities,
                 R_leg_init_joints_velocities=self.R_leg_init_joints_velocities,
                 L_leg_joints_positions=self.L_leg_joints_positions,
                 R_leg_joints_positions=self.R_leg_joints_positions,
                 L_leg_joints_velocities=self.L_leg_joints_velocities,
                 R_leg_joints_velocities=self.R_leg_joints_velocities)
        print(f"Trajectory saved to {full_path}")

def CalcNstepsCoMTrajectory(num_step:int, 
                            step_length:float, 
                            step_time:float, 
                            z_com:float, 
                            y_0:float,
                            desire_robot_height=0.4):
    builder = DiagramBuilder()
    system = LIPMGaitGeneration(g = 9.807, z_com = z_com)
    LIP = builder.AddSystem(system)
    logger = LogVectorOutput(system.get_output_port(0), builder)
    acc_logger = LogVectorOutput(system.get_output_port(2), builder)
    diagram = builder.Build()
    start_swing = 0.03
    y_0_offset = 0.0
    y_0 = y_0 + y_0_offset
    [x_0, xd_0, y_0, yd_0, T_ss] = LIP.CalInitialCondition(step_time, y_0, step_length)
    simulator = Simulator(diagram)

    CoM_x = np.array([])
    CoM_y = np.array([])
    CoM_xd = np.array([])
    CoM_yd = np.array([])
    foot_holdx = 0.0
    foot_holdy = 0.0
    foot_center = []

    for i in range(num_step):
        new_context = diagram.CreateDefaultContext()
        simulator.reset_context(new_context)
        new_context.SetContinuousState([x_0, xd_0, y_0, yd_0])
        simulator.Initialize()
        simulator.AdvanceTo(T_ss-0.01)
        log = logger.FindLog(new_context)
        acc_data = acc_logger.FindLog(new_context).data()
        CoM_x = np.concatenate((CoM_x, log.data()[0]+foot_holdx))
        CoM_y = np.concatenate((CoM_y, log.data()[2]+foot_holdy))
        CoM_xd = np.concatenate((CoM_xd, log.data()[1]))
        CoM_yd = np.concatenate((CoM_yd, log.data()[3]))
        CoM_xdd = acc_data[0]
        CoM_ydd = acc_data[1]
        foot_holdx = foot_holdx + step_length
        foot_holdy = foot_holdy + 2.0*log.data()[2][-1]
        x_0 = x_0
        xd_0 = xd_0
        y_0 = -y_0
        yd_0 = -yd_0

    CoM_y = CoM_y-y_0_offset
    comX = np.delete(CoM_x, [1, 2, 3])
    comY = np.delete(CoM_y, [1, 2, 3])
    comZ = np.ones([comX.size])*(desire_robot_height)
    st_P_c = np.array([comX,comY,comZ]).T
    comXd = np.delete(CoM_xd, [1, 2, 3])
    comYd = np.delete(CoM_yd, [1, 2, 3])
    comZd = np.zeros([comX.size])
    st_V_c = np.array([comXd,comYd,comZd]).T
    comXdd = np.delete(CoM_xdd, [1, 2, 3])
    comYdd = np.delete(CoM_ydd, [1, 2, 3])
    comZdd = np.zeros([comX.size])  
    st_Vd_c = np.array([comXdd,comYdd,comZdd]).T 
    walk_phase = []
    for i in (comY):
        # print(i)
        if (i+y_0_offset < start_swing): #or (i-y_0_offset > y_0*2 - start_swing):
            walk_phase.append(0) # single support
        else:
            walk_phase.append(1) # double support
    walk_phase = np.array(walk_phase) 
    t_swing = np.count_nonzero(walk_phase == 0)*0.01/num_step
    t0_swing = np.where(walk_phase == 0)[0][0]*0.01  
    return st_P_c, st_V_c, st_Vd_c, t0_swing, t_swing

def CalcSwingLegTrajectory(swing_high, 
                           step_length, 
                           step_time, 
                           t0_swing, 
                           t_swing): 
    builder = DiagramBuilder()
    dummy_system = builder.AddNamedSystem("dummy param",DummyParameters2(T0_swing=t0_swing,T_swing=t_swing,step_lenght=step_length))
    traj = CubicTrajectory3D()
    traj_system = builder.AddNamedSystem("traj",traj)
    traj_system.z_max = swing_high

    for i in range(3):
        builder.Connect(dummy_system.get_output_port(i),traj_system.get_input_port(i))
    traj_logger = LogVectorOutput(traj_system.get_output_port(0), builder)
    traj_vel_logger = LogVectorOutput(traj_system.get_output_port(1), builder) 
    diagram = builder.Build()
    simulator = Simulator(diagram)
    # Run the simulation.
    context = diagram.CreateDefaultContext()
    simulator.reset_context(context)
    simulator.Initialize()
    simulator.AdvanceTo(step_time)

    log = traj_logger.FindLog(context)
    log_vel = traj_vel_logger.FindLog(context)
    swX = log.data()[0]
    swY = log.data()[1]
    swZ = log.data()[2]
    sw0_P_sw = np.array([swX,swY,swZ]).T
    swXd = log_vel.data()[0]
    swYd = log_vel.data()[1]
    swZd = log_vel.data()[2]
    sw0_V_sw = np.array([swXd,swYd,swZd]).T
    return sw0_P_sw, sw0_V_sw

def CalcInitialJointTrajectory(model_series:str, 
                               model_name:str, 
                               step_length:float, 
                               target_height:float, 
                               y_0:float,
                               q_initial,
                               q_final,
                               qd_final):
    """Calculate the initial joint trajectory for the robot.

    Args:
        model_series (str): The model series of the robot.
        model_name (str): The model name of the robot.
        step_length (float): The length of the step.
        target_height (float): The target height of the robot.
        y_0 (float): The initial y position of the robot.
        q_initial (_type_): The initial joint positions.
        q_final (_type_): The final joint positions.
        qd_final (_type_): The final joint velocities.

    Returns:
        _type_: q_L, qd_L, q_R, qd_R
    """
    CoM_offset = 0.01
    if model_series == "Hanuman":
        L1 = 0.054
        L2 = 0.03
        L3 = 0.1605
        L4 = 0.166
        L5 = 0.049
        link_lengths = [L1,L2,L3,L4,L5]
        CoM_P_sw_viapoints0 = [y_0*2.0 + CoM_offset, 0.0, -0.35, 1]
        CoM_P_sw_viapoints1 = [y_0*2.0 + CoM_offset, (step_length)*0.5, -0.35+0.05, 1]
        CoM_P_sw_viapoints2 = [y_0*2.0 + CoM_offset, (step_length), -0.35, 1]
        viapoints = [CoM_P_sw_viapoints0, CoM_P_sw_viapoints1, CoM_P_sw_viapoints2]    
        q_halfstep = []
        body_R_foot = np.array([
                                [0,-1, 0],
                                [0, 0, 1],
                                [-1,0, 0],
                                [0, 0, 0]])
        for i in viapoints:
            body_P_Rfoot = np.array([i]).T
            body_T_Rfoot = np.concatenate((body_R_foot,body_P_Rfoot),axis=1)
            th_R = CalcLegInverseKinematics(tform=body_T_Rfoot,link_lengths=link_lengths,is_left=False)
            q_halfstep.append(th_R)
        q_halfstep = np.array(q_halfstep).T
        body_P_Lfoot_0 = np.array([[CoM_offset, 0.0, -0.35, 1]]).T
        body_T_Lfoot_0 = np.concatenate((body_R_foot,body_P_Lfoot_0),axis=1)

        q_L_init = q_initial[:6]
        q_L_0 = CalcLegInverseKinematics(tform=body_T_Lfoot_0,
                                         link_lengths=link_lengths,
                                         is_left=True)
        q_L_1 = q_L_0
        q_L_2 = q_L_0
        q_L_3 = q_final[:6]

        q_R_init = q_initial[6:]
        q_R_0 = q_halfstep[:,0]
        q_R_1 = q_halfstep[:,1]
        q_R_2 = q_halfstep[:,2]
        q_R_3 = q_final[6:]

    elif model_series == "Volta":
        CoM_P_sw_viapoints0 = np.array([0.0, 
                                        -(y_0*2.0), 
                                        0.0])
        CoM_P_sw_viapoints1 = np.array([(step_length)*0.5, 
                                        -(y_0*2.0), 
                                        0.05])
        CoM_P_sw_viapoints2 = np.array([step_length, 
                                        -(y_0*2.0), 
                                        0.0])
        viapoints = [CoM_P_sw_viapoints0, CoM_P_sw_viapoints1, CoM_P_sw_viapoints2]
    
        q_halfstep = []
        plant, robot_ind = DefinePlant(model_name=model_name,
                                    is_fixed=True,
                                    fixed_frame="torso") 
        for i in viapoints:
            # Inverse pose kinematics
            # is_success, th_R = SolveInverseKinematics(model_series=model_series,
            #                             plant=plant,
            #                             is_left=False,
            #                             base_P_e=i,
            #                             base_R_e=RollPitchYaw(0, 0, 0).ToRotationMatrix(),
            #                             initial_guess=None)
            is_success,q = SolveIKWithComPositionConstraint(model_series="Volta",
                                                            plant=plant,
                                                            is_L_stance=True,
                                                            st_P_com=np.array([0.0, -CoM_offset, target_height]),
                                                            st_R_base=RotationMatrix.Identity(),
                                                            st_P_sw=i,
                                                            st_R_sw=RotationMatrix.Identity())
            q_halfstep.append(q)
        q_halfstep = np.array(q_halfstep).T

        q_L_init = q_initial[:6]
        # is_success, q_L_0 = SolveInverseKinematics(model_series=model_series,
        #                                 plant=plant,
        #                                 is_left=True,
        #                                 base_P_e=np.array([0.0, -CoM_offset, -target_height]),
        #                                 base_R_e=RollPitchYaw(0, 0, 0).ToRotationMatrix(),
        #                                 initial_guess=None)
        q_L_0 = q_halfstep[0:6,0]
        q_L_1 = q_halfstep[0:6,1]
        q_L_2 = q_halfstep[0:6,2]
        q_L_3 = q_final[:6]

        q_R_init = q_initial[6:]
        q_R_0 = q_halfstep[6:,0]
        q_R_1 = q_halfstep[6:,1]
        q_R_2 = q_halfstep[6:,2]
        q_R_3 = q_final[6:]

    
    q_L_viapoints = np.array([q_L_init,
                           q_L_0,
                           q_L_1,
                           q_L_2,
                           q_L_3]).T

    q_R_viapoints = np.array([ q_R_0,
                            q_R_1,
                            q_R_2,
                            q_R_3]).T
    qd = np.zeros(6)
    qd_L_end = qd_final[:6]
    qd_R_end = qd_final[6:]
    Times = [0.0, 2.0, 3.0, 4.0, 5.0]
    qd_L_viapoints = np.concatenate((np.zeros([4,6]),np.array([qd_L_end]))).T
    qL_trajs = InitMultipleViapointTrajectories(P= q_L_viapoints,
                                                     V= qd_L_viapoints,
                                                     Times= Times,
                                                     n= 6)

    qR_trajs = InitContinuousSecondDerivativesTrajectories(P= q_R_viapoints,
                                                            V_i= qd,
                                                            V_f= qd_R_end,
                                                            Times= Times[1:],
                                                            n= 6)
    qR_trajs_0 = InitP2PTrajectories(p_start=q_R_init,
                                        p_final=q_R_0,
                                        v_start=qd,
                                        v_final=qd,
                                        total_time=Times[1])
    Q_L = []
    Qd_L = []
    Q_R = []
    Qd_R = []
    CoM_P = []
    curr_time = 0.0
    dt = 0.01
    while curr_time <= Times[-1]:
        temp_qL = []
        temp_qdL = []
        temp_qR = []
        temp_qdR = []
        if curr_time <= Times[1]:
            temp_traj_R = qR_trajs_0
        else:
            temp_traj_R = qR_trajs
        for i in range(len(qL_trajs)):
            temp_qL.append(qL_trajs[i].value(curr_time)[0][0])
            temp_qdL.append(qL_trajs[i].EvalDerivative(curr_time, 1)[0][0])
            
            temp_qR.append(temp_traj_R[i].value(curr_time)[0][0])
            temp_qdR.append(temp_traj_R[i].EvalDerivative(curr_time, 1)[0][0])
        
        Q_L.append(temp_qL)
        Qd_L.append(temp_qdL)
        Q_R.append(temp_qR)
        Qd_R.append(temp_qdR)
        curr_time += dt
    q_L = np.array(Q_L)
    qd_L = np.array(Qd_L)
    q_R = np.array(Q_R)
    qd_R = np.array(Qd_R)
    return q_L, qd_L, q_R, qd_R

def InitP2PTrajectories(p_start, p_final, v_start, v_final, total_time):
    time = np.array([0.0, total_time])
    trajs = []
    for i in range(len(p_start)):
        P = np.array([[p_start[i],p_final[i]]])
        V = np.array([[v_start[i],v_final[i]]])
        trajs.append(PiecewisePolynomial.CubicHermite(time, P, V))
    return trajs

def InitMultipleViapointTrajectories(P, V, Times, n):
    trajs = []
    for i in range(n):
        trajs.append(PiecewisePolynomial.CubicHermite(np.array(Times), 
                                                      np.array([P[i,:]]), 
                                                      np.array([V[i,:]])))
    return trajs

def InitContinuousSecondDerivativesTrajectories(P, V_i, V_f, Times:list,n):
    trajs = []
    for i in range(n):
        trajs.append(PiecewisePolynomial.CubicWithContinuousSecondDerivatives(np.array(Times),
                                                                              np.array([P[i,:]]),
                                                                              np.array([[V_i[i]]]),
                                                                              np.array([[V_f[i]]])))
    return trajs

def CalcJointsTrajectory(model_series:str,model_name:str, st_P_c, st_V_c, sw0_P_sw, sw0_V_sw, step_length, y_0): 
    st_P_sw0 = np.array([-step_length, y_0*2.0, 0.0])
    Q_L = []
    Q_R = []
    Qd_L = []
    Qd_R = []
    Manipulability_L = []
    Manipulability_R = []   
    initial_guess_L = None
    initial_guess_R = None
    initial_guess = np.zeros(12)
    for ind in range(st_P_c.shape[0]):
        # Invese pose kinematics
        if model_series == "Hanuman":
            plant, robot_ind = InitialCalcLegSpatialJacobianMatrix(model_name="humanoid_L_leg")
            st_R_c = RotationMatrix.MakeZRotation(-np.pi/2.0) 
            R = RotationMatrix.MakeZRotation(np.pi/2.0) 
        else:
            plant, robot_ind = DefinePlant(model_name=model_name,
                                        is_fixed=True,
                                        fixed_frame="torso")
            st_R_c = RotationMatrix.MakeZRotation(0.0) 
            R = RotationMatrix.MakeZRotation(0.0)   
        st_T_c = RigidTransform(st_R_c, st_P_c[ind])
        c_T_st = st_T_c.inverse()   # Right foot

        st_P_sw = st_P_sw0 + sw0_P_sw[ind]
        st_R_sw = RotationMatrix.MakeZRotation(0.0)
        st_T_sw = RigidTransform(st_R_sw, st_P_sw)
        c_T_sw = c_T_st @ st_T_sw   # Left foot
        if model_series == "Hanuman":
            L1 = 0.054
            L2 = 0.03
            L3 = 0.1605
            L4 = 0.166
            L5 = 0.049
            link_lengths = [L1,L2,L3,L4,L5]
            temp_R = RotationMatrix.MakeZRotation(np.pi/2.0) @ RotationMatrix.MakeYRotation(np.pi/2.0)
            q_L = CalcLegInverseKinematics(tform=RigidTransformToMatrix(RigidTransform(temp_R ,c_T_sw.translation())),
                                            link_lengths=link_lengths,
                                            is_left=True)
            q_R = CalcLegInverseKinematics(tform=RigidTransformToMatrix(RigidTransform(temp_R ,c_T_st.translation())),
                                            link_lengths=link_lengths,
                                            is_left=False)
        else:
            # is_L_success,q_L = SolveInverseKinematics(model_series= model_series,
            #                                             plant=plant,
            #                                             is_left=True,
            #                                             base_P_e=c_T_sw.translation(),
            #                                             base_R_e=RollPitchYaw(0, 0, 0).ToRotationMatrix(),
            #                                             initial_guess=initial_guess_L)
            # initial_guess_L = np.concatenate((q_L, np.zeros(6)))
            # is_R_success,q_R = SolveInverseKinematics(model_series=model_series,
            #                                             plant=plant,
            #                                             is_left=False,
            #                                             base_P_e=c_T_st.translation(),
            #                                             base_R_e=RollPitchYaw(0, 0, 0).ToRotationMatrix(),
            #                                             initial_guess=initial_guess_R)
            # initial_guess_R = np.concatenate(( np.zeros(6),q_R))
            # print(st_P_c[ind])
            # print(st_P_sw)
            is_success,q = SolveIKWithComPositionConstraint(model_series="Volta",
                                                            plant=plant,
                                                            is_L_stance=False,
                                                            st_P_com=st_P_c[ind],
                                                            st_R_base=RotationMatrix.Identity(),
                                                            st_P_sw=st_P_sw,
                                                            st_R_sw=RotationMatrix.Identity(),
                                                            q_initial_guess=initial_guess
                            )
            if is_success == False:
                print(is_success)
            initial_guess = q
        # Inverse differential kinematics
        c_R_st = c_T_st.rotation()
        st_R_sw0 = RotationMatrix.MakeZRotation(0.0)
        c_v_st = c_R_st @ st_V_c[ind]
        c_v_st = -1.0*c_v_st
        c_v_sw = c_v_st + c_R_st @ st_R_sw0 @ sw0_V_sw[ind]
        w = np.zeros([3,1])
        
        v_sw = R @ np.array([c_v_sw]).T 
        twist_sw = np.concatenate((w,v_sw),axis=0)
        v_st = R @ np.array([c_v_st]).T 
        twist_st = np.concatenate((w,v_st),axis=0)
        # Calculate joint velocities
        if model_series == "Hanuman":
            joint_angles_1 =  q_L
            joint_angles_2 = q_R
        else:
            # joint_angles_1 =  np.array(np.concatenate((q_L, np.zeros(6))))
            # joint_angles_2 = np.concatenate((np.zeros(6),q_R))
            joint_angles_1 = q
            joint_angles_2 = q
        qd_L, manipulability_L = CalcInverseDiffKinematics(is_left=True, 
                                                           plant=plant,
                                                            robot_instant_index=robot_ind,
                                                            joint_angles=joint_angles_1,
                                                            twist=twist_sw)
        qd_R, manipulability_R = CalcInverseDiffKinematics(is_left=False, 
                                                           plant=plant,
                                                           robot_instant_index=robot_ind,
                                                           joint_angles=joint_angles_2,
                                                           twist=twist_st)
        Manipulability_L.append(manipulability_L)
        Manipulability_R.append(manipulability_R)
        Q_L.append(q[0:6])
        Q_R.append(q[6:])
        Qd_L.append(qd_L)
        Qd_R.append(qd_R)
    # Convert lists to numpy arrays
    L_leg_joints_positions = np.array(Q_L)
    R_leg_joints_positions = np.array(Q_R)
    L_leg_joints_velocities = np.array(Qd_L)
    R_leg_joints_velocities = np.array(Qd_R)
    Manipulability_L = np.array(Manipulability_L)
    Manipulability_R = np.array(Manipulability_R)

    return L_leg_joints_positions, R_leg_joints_positions, L_leg_joints_velocities, R_leg_joints_velocities, Manipulability_L, Manipulability_R

def InitSwingLegTrajectory(foot_placements,
                           step_time:float=3.5,
                           z_max:float=0.05):
    '''
    Docstring for InitSwingLegTrajectory
    
    :param foot_placements: [0.0, 0.0, 0.0, step_lenght*2.0, 0.0,0.0]
    :param step_time: Description
    :param z_max: Description
    '''
    time_xy = np.array([0.0, step_time])
    time_z = np.array([0.0, step_time*0.5, step_time])
    
    px = np.array([[foot_placements[0],foot_placements[3]]])
    py = np.array([[foot_placements[1],foot_placements[4]]])
    pz = np.array([[foot_placements[2],z_max,foot_placements[5]]])
    vx = np.array([[0.0, 0.0]])
    vy = np.array([[0.0, 0.0]])
    vz = np.array([[0.0, 0.0, 0.0]])
    trajectory_x = PiecewisePolynomial.CubicHermite(time_xy, px, vx)
    trajectory_y = PiecewisePolynomial.CubicHermite(time_xy, py.reshape(1, -1), vy)
    trajectory_z = PiecewisePolynomial.CubicHermite(time_z, pz.reshape(1, -1), vz)
    return [trajectory_x,trajectory_y,trajectory_z]


# Deprecated misspelled names, kept so older notebooks keep working.
CalcSwingLegtrajectory = CalcSwingLegTrajectory
