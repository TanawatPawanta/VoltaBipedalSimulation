"""Inverse dynamics and gravity compensation helpers."""
import os

import numpy as np
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import MultibodyForces

from volta.paths import MODELS_DIR


def InitialCalcInverseDynamics()->list[MultibodyPlant]:
    """Construct MultibodyPlant for dynamics calculation. 
    Make the humanoid like 12-DOF serial manipulator.

    Returns:
        list[MultibodyPlant]: [L_fixed,R_fixed]
    """
    Plants = []
    urdf_file_names = ["humanoid_L_fixed.urdf","humanoid_R_fixed.urdf"]
    fixed_frames = ["L_foot_contact", "R_foot_contact"]
    for i in range(2):
        plant = MultibodyPlant(time_step=0.0)
        parser = Parser(plant)
        dir = MODELS_DIR
        robot_urdf = urdf_file_names[i]
        robot_urdf_path = os.path.join(dir,robot_urdf)
        robot_instant_index = parser.AddModels(robot_urdf_path)
        robot_fixed_frame = plant.GetFrameByName(fixed_frames[i])
        init_pos = [0.0,0.0,0.0] 
        init_orien = np.asarray([0, 0, 0])
        X_WRobot = RigidTransform(
            RollPitchYaw(init_orien * np.pi / 180), p=init_pos)
        plant.WeldFrames(plant.world_frame(),robot_fixed_frame, X_WRobot)
        plant.Finalize()
        Plants.append(plant)
    return Plants

def InitialFloatingBasePlant()->MultibodyPlant:
    plant = MultibodyPlant(time_step=0.0)
    parser = Parser(plant)
    dir = MODELS_DIR
    robot_urdf = "humanoid_half_body.urdf"
    robot_urdf_path = os.path.join(dir,robot_urdf)
    parser.AddModels(robot_urdf_path)
    plant.Finalize()
    return plant

def CalcGravityCompensation(plants:list[MultibodyPlant], q:list,is_L_stance:bool)->list:
    pass

def CalcInverseDynamics(plants:list[MultibodyPlant], q:list, qd:list, qdd:list ,is_L_stance:bool)->list:
    """
    Compute torque by tau_id = M(q)v_dot + C(q,q_dot)v + tau_app + J*F 
    Args:
        plants (list): plants of fixed base robot 
        q (list): joints position
        qd (list): joints velocity
        qdd (list): joint acceleration
        is_L_stance (bool): 

    Returns:
        list: joints torque
    """
    q_L = q[0:6]
    q_R = q[6:]
    qd_L = qd[0:6]   
    qd_R = qd[6:]
    qdd_L = qdd[0:6]
    qdd_R = qdd[6:]
    if is_L_stance:
        plant = plants[0]
        pesent_q_L = -1*np.array(q_L[::-1])
        pesent_q_R = np.array(q_R)
        q = np.concatenate((pesent_q_L,pesent_q_R))
        pesent_qd_L = -1*np.array(qd_L[::-1])
        pesent_qd_R = np.array(qd_R)
        qd = np.concatenate((pesent_qd_L,pesent_qd_R))
        pesent_qdd_L = -1*np.array(qdd_L[::-1])
        pesent_qdd_R = np.array(qdd_R)
        qdd = np.concatenate((pesent_qdd_L,pesent_qdd_R))
        context = plant.CreateDefaultContext()
        plant.SetPositions(context, q)
        plant.SetVelocities(context, qd)
        external_forces = MultibodyForces(plant)  # All zero by default
        # tau_id = plant.CalcInverseDynamics(context, qdd, external_forces)
        # tau_id_L = -tau_id[0:6]
        # tau__id_R = tau_id[6:]
        plant.SetVelocities(context, np.zeros(12))
        tau_g = plant.CalcGravityGeneralizedForces(context=context)
        tau_g_L = -1.0*(-np.array(tau_g[0:6])[::-1])
        tau_g_R = -np.array(tau_g[6:12]) 
    else:
        plant = plants[1]
        pesent_q_L = np.array(q_L)
        pesent_q_R = -1*np.array(q_R[::-1])
        q = np.concatenate((pesent_q_R,pesent_q_L))
        pesent_qd_L = np.array(qd_L)
        pesent_qd_R = -1*np.array(qd_R[::-1])
        qd = np.concatenate((pesent_qd_R,pesent_qd_L))
        pesent_qdd_L = np.array(qdd_L)
        pesent_qdd_R = -1*np.array(qdd_R[::-1])
        qdd = np.concatenate((pesent_qdd_R,pesent_qdd_L))
        context = plant.CreateDefaultContext()
        plant.SetPositions(context, q)
        plant.SetVelocities(context, qd)
        external_forces = MultibodyForces(plant)  # All zero by default
        # tau_id = plant.CalcInverseDynamics(context, qdd, external_forces)
        # tau_id_R = -tau_id[0:6]
        # tau_id_L = tau_id[6:]
        plant.SetVelocities(context, np.zeros(12))
        tau_g = plant.CalcGravityGeneralizedForces(context=context)
        # tau_g_R = -1.0*(-np.array(tau_g[0:6])[::-1])
        tau_g_R = -1.0*(-np.array(tau_g[0:6])[::-1])
        tau_g_L = -np.array(tau_g[6:12]) 
        
        tau_L =  tau_g_L
        tau_R = tau_g_R
    return np.concatenate((tau_L,tau_R))


# Deprecated misspelled names, kept so older notebooks keep working.
InitialCalcInversDynamic = InitialCalcInverseDynamics
CalcInversDynamic = CalcInverseDynamics
