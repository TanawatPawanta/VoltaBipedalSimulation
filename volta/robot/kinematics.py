"""Forward / inverse / differential kinematics of the Volta legs on Drake MultibodyPlants."""
import os

import numpy as np
from numpy import ndarray
from numpy.linalg import inv
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.inverse_kinematics import ComPositionConstraint, InverseKinematics
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.solvers import Solve

from volta.paths import MODELS_DIR


def CalcLegForwardKinematics(joint_angles, link_lengths, forward_axis, is_left):
    """
    Compute px, py, and pz based on the given joint angles and link lengths.
    Ref: Kinematics and Dynamics of a New 16 DOF Humanoid Biped Robot with Active Toe Joint (2017)

    Parameters:
        joint_angles (list): Joint angles [theta1, theta2, ..., theta6] in radians.
        link_lengths (list): Link lengths [l1, l2, ..., l5] in m.

    Returns:
        np.array: [px, py, pz].T - The computed sole positions wrt body.
    """
    theta1, theta2, theta3, theta4, theta5, theta6= joint_angles
    L1, L2, L3, L4, L5 = link_lengths
    if is_left:
        L1 = -L1
    # print("L1: ", L1)
    # print("input L: ",link_lengths)
    # print("input theta: ", joint_angles)
    # Define shorthand for sin and cos
    sin = np.sin
    cos = np.cos
    c1 = cos(theta1)
    c2 = cos(theta2)
    c26 = cos(theta2 + theta6)
    c3 = cos(theta3)
    c34 = cos(theta3 + theta4)
    c345 = cos(theta3 + theta4 + theta5)
    c4 = cos(theta4)
    c6 = cos(theta6)
    s1 = sin(theta1)
    s2 = sin(theta2)
    s26 = sin(theta2 + theta6)
    s3 = sin(theta3)
    s34 = sin(theta3 + theta4)
    s345 = sin(theta3 + theta4 + theta5)

    R = L3*c3 + L4*c34
    r = L5*c6*c345
    px = L3*s3 + L4*s34 + L5*c6*s345 
    py = L1 + R*s2 + r*s26
    pz = R*c2 + r*c26 + L2
    if forward_axis == 'x':
        P = np.array([px*c1 - py*s1, px*s1 + py*c1, -pz]).T
    elif forward_axis == 'y':
        P = np.array([(px*s1 + py*c1), (px*c1 - py*s1), -pz]).T
    for i in range(len(P)):
        if abs(P[i]) < 1e-03:
            P[i] = 0.0
    return P

def InitialCalcLegSpatialJacobianMatrix(model_name:str):
    plant = MultibodyPlant(time_step=0.0)
    parser = Parser(plant)
    dir = MODELS_DIR
    robot_urdf = f"{model_name}.urdf"
    robot_urdf_path = os.path.join(dir,robot_urdf)
    robot_instant_index = parser.AddModels(robot_urdf_path)
    robot_body_frame = plant.GetFrameByName("torso")
    init_pos = [0.0, 0.0, 0.0] 
    init_orien = np.asarray([0.0, 0.0, 0.0])
    X_floorRobot = RigidTransform(
        RollPitchYaw(init_orien * np.pi / 180), p=init_pos)
    plant.WeldFrames(plant.world_frame(),robot_body_frame, X_floorRobot)
    plant.Finalize()
    return plant, robot_instant_index

def CalcLegSpatialJacobianMatrix(is_left, plant, robot_instant_index, joint_angles:list, p_BoBp_B = np.zeros(3)):   
    """_summary_

    Args:
        is_left (bool): _description_
        plant (_type_): _description_
        robot_instant_index (_type_): _description_
        joint_angles (list): _description_

    Returns:
        _type_: Jacobian matrix (angular and translational)
    """
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, robot_instant_index[0], np.array(joint_angles))
    if plant.num_positions() >= 12:
        if is_left:
            end_effector_frame = plant.GetFrameByName("L_foot_contact")
        else:
            end_effector_frame = plant.GetFrameByName("R_foot_contact")
    elif plant.num_positions() == 6:
        end_effector_frame = plant.GetFrameByName("L_foot_contact")
    else:
        print("CalcLegSpatialJacobianMatrix: plant's position is not 12 or 6")

    with_respect_to = JacobianWrtVariable.kQDot
    # p_BoBp_B = np.zeros(3)

    base_J_e = plant.CalcJacobianSpatialVelocity(
        context=context,
        with_respect_to=with_respect_to,
        frame_B=end_effector_frame,     # Jacobian for the end-effector
        p_BoBp_B=p_BoBp_B,              # At origin of end-effector frame
        frame_A=plant.world_frame(),    # Compute velocity relative to world
        frame_E=plant.world_frame(),    # Expressed in world frame
    )

    threshold = 1e-05
    base_J_e[abs(base_J_e) <= threshold] = 0.0
    return base_J_e

def CalcSpatialJacobianMatrix(plant,  
                              robot_instant_index, 
                              base_frame_name:str,
                              end_effector_name:str,
                              joint_angles:list, 
                              p_BoBp_B = np.zeros(3)):   
    """Calc Jacobian matrix (base_J_eff) from base frame to end-effector frame 
    express in world frame. When V = base_J_eff x q_d

    Args:
        is_left (bool): _description_
        plant (_type_): _description_
        robot_instant_index (_type_): _description_
        joint_angles (list): _description_

    Returns:
        _type_: Jacobian matrix (angular and translational)
    """
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, robot_instant_index[0], np.array(joint_angles))
    base_frame = plant.GetFrameByName(base_frame_name)
    end_effector_frame = plant.GetFrameByName(end_effector_name)

    with_respect_to = JacobianWrtVariable.kV
    # p_BoBp_B = np.zeros(3)

    base_J_e = plant.CalcJacobianSpatialVelocity(
        context=context,
        with_respect_to=with_respect_to,
        frame_B=end_effector_frame,     # Jacobian for the end-effector
        p_BoBp_B=p_BoBp_B,              # At origin of end-effector frame
        frame_A=plant.world_frame(),    # Compute velocity relative to world
        frame_E=plant.world_frame(),    # Expressed in world frame
    )

    threshold = 1e-04
    base_J_e[abs(base_J_e) <= threshold] = 0.0
    return base_J_e

def CalcForwardDiffKinematics(is_left, plant, robot_instant_index, joint_angles:list, joint_velocities:list):
    """Calculate the forward velocity kinematics of the robot leg using the spatial Jacobian matrix.

    Args:
        plant (_type_): plant that stores the robot model.
        robot_instant_index (_type_): robot instant index in the plant.
        joint_angles (list): joint angles in radians.
        joint_velocities (list): joint velocities in radians per sec.

    Returns:
        3x1 vector, 3x1 vector: angular velocity and linear velocity of the end-effector frame expressed in base frame.
    """
    base_J_e = CalcLegSpatialJacobianMatrix(is_left=is_left, plant=plant,
                                 robot_instant_index=robot_instant_index,
                                 joint_angles=joint_angles)
    twist = base_J_e @ np.array([joint_velocities]).T
    W = twist[:3,0]
    V = twist[3:,0]
    return W,V

def CalcInverseDiffKinematics(is_left, plant, robot_instant_index, joint_angles:list, twist:list):
    """Calculate the inverse velocity kinematics of the robot leg using the spatial Jacobian matrix.

    Args:
        plant (_type_): plant that stores the robot model.
        robot_instant_index (_type_): robot instant index in the plant.
        joint_angles (list): joint angles in radians.
        twist (list): twist vector of interested frame express in base frame in the form of [wx, wy, wz, vx, vy, vz].

    Returns:
        _type_: joint velocities in radians per second.
    """
    base_J_e = CalcLegSpatialJacobianMatrix(is_left=is_left, plant=plant,
                                 robot_instant_index=robot_instant_index,
                                 joint_angles=joint_angles)
    if plant.num_positions() == 12:
        if is_left:
            base_J_e = base_J_e[:,0:6]
        else:
            base_J_e = base_J_e[:,6:12]
    manipulability = np.linalg.det(base_J_e @ base_J_e.T)
    # print(joint_angles)
    # print("shape of J: ",base_J_e.shape)
    qd = inv(base_J_e) @ twist 
    threshold = 1e-05
    qd[abs(qd) <= threshold] = 0.0 
    return qd[:,0], manipulability

def CheckJointLimits(joint_angles:list):
    """
    Check if the joint angles are within the specified joint limits and return saturated values if exceeded.

    Parameters:
    joint_angles (list or np.ndarray): joint angles in rad [q1, q2, q3, q4, q5, q6].
    joint_limits (list of tuples): Joint limits in rad as [(min1, max1), (min2, max2), ...].

    Returns:
    tuple:
        - bool: True if all joint angles are within limits, False otherwise.
        - np.ndarray: Saturated joint angles.
    """
    joint_limits = [(-3.1416, 3.1416), 
                    (-0.5236, 0.5236), 
                    (-1.5708, 2.0944), 
                    (-2.2689, 0.0000), 
                    (-1.7453, 1.3963), 
                    (-0.3491, 0.3491)]
    
    joint_angles = np.array(joint_angles)
    saturated_angles = np.copy(joint_angles)
    out_of_bounds = False

    for i, (angle, (lower, upper)) in enumerate(zip(joint_angles, joint_limits)):
        if angle < lower:
            saturated_angles[i] = lower
            out_of_bounds = True
        elif angle > upper:
            saturated_angles[i] = upper
            out_of_bounds = True

    return out_of_bounds, saturated_angles

def CalcLegInverseKinematics(tform, link_lengths, is_left):
    """
    Compute inverse kinematics to find joint angles from a transformation matrix and robot dimensions.
    
    Parameters:
        tform (numpy.ndarray): 4x4 transformation matrix (numpy array) of enfeffectro respect w/ base(y axis point forward!).
        L1, L2, L3, L4, L5 (float): Robot link lengths.
        is_left (bool): Flag to determine if the calculation is for the left leg.

    Returns:
        numpy.ndarray: 1x6 Joint angles [th1, th2, th3, th4, th5, th6].
    """
    L1, L2, L3, L4, L5 = link_lengths
    # Adjust L1 if it's the left leg
    if not is_left:
        L1 = -L1

    # Perform offsets
    tform = tform + np.array([[0, 0, 0, L1],
                              [0, 0, 0, 0],
                              [0, 0, 0, L2],
                              [0, 0, 0, 0]])

    # Extract position and orientation
    R = tform[:3, :3]
    p = tform[:3, 3]

    # Inverse rotation matrix
    Rp = R.T
    n, s, a = Rp[:, 0], Rp[:, 1], Rp[:, 2]
    p = -np.dot(Rp, p)

    # Compute joint angles
    cos4 = ((p[0] + L5) ** 2 + p[1] ** 2 + p[2] ** 2 - L3 ** 2 - L4 ** 2) / (2 * L3 * L4)
    cos4 = np.clip(cos4, -1.0, 1.0)  # Clip to handle numerical precision issues

    th4 = np.arctan2(np.sqrt(1 - cos4 ** 2), cos4)

    temp = (p[0] + L5) ** 2 + p[1] ** 2
    temp = max(temp, 0.0)
    th5 = np.arctan2(-p[2], np.sqrt(temp)) - np.arctan2(L3 * np.sin(th4), L3 * np.cos(th4) + L4)

    th6 = np.arctan2(p[1], -p[0] - L5)

    temp = 1 - (np.sin(th6) * a[0] + np.cos(th6) * a[1]) ** 2
    temp = max(temp, 0.0)
    th2 = np.arctan2(-np.sqrt(temp), np.sin(th6) * a[0] + np.cos(th6) * a[1])
    th2 += np.pi / 2.0

    th1 = np.arctan2(-np.sin(th6) * s[0] - np.cos(th6) * s[1],
                     -np.sin(th6) * n[0] - np.cos(th6) * n[1])

    th345 = np.arctan2(a[2], np.cos(th6) * a[0] - np.sin(th6) * a[1])
    th345 -= np.pi
    th3 = th345 - th4 - th5

    # Pack joint angles
    th = np.array([-th1, -th2, -th3, -th4, -th5, th6])
    for i in range(len(th)):
        if abs(th[i]) < 1.5e-04: # encoder resolution is 1.5e-04 rad/bit
            th[i] = 0.0
    return th

def SolveInverseKinematics(model_series:str,
                           plant:MultibodyPlant, 
                           is_left:bool,
                           base_P_e:ndarray,
                           base_R_e:RotationMatrix,
                           initial_guess:list=None):

    IK_solver = InverseKinematics(plant)

    if is_left:
        end_effector_frame = plant.GetFrameByName("L_foot_contact")
    else:
        end_effector_frame = plant.GetFrameByName("R_foot_contact")
    base_frame = plant.GetFrameByName("torso")
    desired_position = base_P_e  
    desired_orientation = base_R_e

    # Add position and orientation constraints
    IK_solver.AddPositionConstraint(
        frameB=end_effector_frame,
        p_BQ=np.zeros(3),     # point in end-effector frame
        frameA=base_frame,
        p_AQ_lower=desired_position-0.0001,
        p_AQ_upper=desired_position+0.0001
    )
    IK_solver.AddOrientationConstraint(
        frameAbar=base_frame,
        R_AbarA=desired_orientation,
        frameBbar=end_effector_frame,
        R_BbarB=desired_orientation,
        theta_bound=0.0001  # tolerance in radians
    )
    q_variables = IK_solver.q()

    if model_series == "Hanuman":
        joint_limits = [(-3.1416, 3.1416), 
                        (-0.5236, 0.5236), 
                        (-1.5708, 2.0944), 
                        (-2.2689, 0.0000), 
                        (-1.7453, 1.3963), 
                        (-0.3491, 0.3491),
                        (-3.1416, 3.1416), 
                        (-0.5236, 0.5236), 
                        (-1.5708, 2.0944), 
                        (-2.2689, 0.0000), 
                        (-1.7453, 1.3963), 
                        (-0.3491, 0.3491)]
        
    elif model_series == "Volta":
        joint_limits = [(-1.5708, 1.5708), 
                        (-0.6, 0.6), 
                        (-0.7854, 0.7854), 
                        (-2.35, 0.0), 
                        (-1.5708, 1.5708),
                        (-1.5708, 1.5708),

                        (-1.5708, 1.5708), 
                        (-0.6, 0.6), 
                        (-0.7854, 0.7854),  
                        (0.0, 2.35), 
                        (-1.5708, 1.5708),
                        (-1.5708, 1.5708)
                        ]
    for i in range(plant.num_positions()):
        IK_solver.prog().AddBoundingBoxConstraint(joint_limits[i][0], joint_limits[i][1], q_variables[i])
    if initial_guess is not None:
        initial_guess = initial_guess
    else:
        initial_guess = np.ones(plant.num_positions())*0.1
    result = Solve(IK_solver.prog(),initial_guess)

    is_success = False
    if result.is_success():
        q_solution = result.GetSolution(q_variables)
        is_success = True
    else:
        q_solution = np.zeros(plant.num_positions())
  
    if is_left:
        q_solution = q_solution[0:6]
    else:
        q_solution = q_solution[6:]

    threshold = 1.5e-03
    q_solution[abs(q_solution) <= threshold] = 0.0 
    return is_success,q_solution

def SolveIKWithComPositionConstraint(model_series:str, 
                                    plant:MultibodyPlant, 
                                    is_L_stance:bool,
                                    st_P_com:ndarray,
                                    st_R_base:RotationMatrix,
                                    st_P_sw:ndarray,
                                    st_R_sw:RotationMatrix,
                                    q_initial_guess:list=None):
    IK_solver = InverseKinematics(plant)

    if is_L_stance:
        stance_leg = plant.GetFrameByName("L_foot_contact")
        swing_leg = plant.GetFrameByName("R_foot_contact")
    else:
        stance_leg = plant.GetFrameByName("R_foot_contact")
        swing_leg = plant.GetFrameByName("L_foot_contact")
    base_frame = plant.GetFrameByName("torso")

    # Add position and orientation constraints
    IK_solver.AddPositionConstraint(
        frameB=swing_leg,
        p_BQ=np.zeros(3),     # point in end-effector frame
        frameA=stance_leg,
        p_AQ_lower=st_P_sw-0.0001,
        p_AQ_upper=st_P_sw+0.0001
    )
    IK_solver.AddOrientationConstraint(
        frameAbar=base_frame,
        R_AbarA=st_R_base,
        frameBbar=stance_leg,
        R_BbarB=st_R_base,
        theta_bound=0.0001  # tolerance in radians
    )
    IK_solver.AddOrientationConstraint(
        frameAbar=stance_leg,
        R_AbarA=st_R_sw,
        frameBbar=swing_leg,
        R_BbarB=st_R_sw,
        theta_bound=0.0001  # tolerance in radians
    )
    q_variables = IK_solver.q()
   
    if model_series == "Hanuman":
        joint_limits = [(-3.1416, 3.1416), 
                        (-0.5236, 0.5236), 
                        (-1.5708, 2.0944), 
                        (-2.2689, 0.0000), 
                        (-1.7453, 1.3963), 
                        (-0.3491, 0.3491),
                        (-3.1416, 3.1416), 
                        (-0.5236, 0.5236), 
                        (-1.5708, 2.0944), 
                        (-2.2689, 0.0000), 
                        (-1.7453, 1.3963), 
                        (-0.3491, 0.3491)]
        
    elif model_series == "Volta":
        joint_limits = [(-1.5708, 1.5708), 
                        (-0.6, 0.6), 
                        (-0.7854, 0.7854), 
                        (-2.35, 0.0), 
                        (-1.0, 1.0), 
                        (-1.5708, 1.5708),

                        (-1.5708, 1.5708), 
                        (-0.6, 0.6), 
                        (-0.7854, 0.7854),  
                        (0.0, 2.35), 
                        (-1.0, 1.0),
                        (-1.5708, 1.5708)
                        ]
    
    for i in range(plant.num_positions()):
        IK_solver.prog().AddBoundingBoxConstraint(joint_limits[i][0], joint_limits[i][1], q_variables[i])
    # Create a ComPositionConstraint
    # This constrains the center of mass position relative to a reference frame
    com_constraint = ComPositionConstraint(plant=plant,
                                          model_instances=None,
                                          expressed_frame=stance_leg,
                                          plant_context=plant.CreateDefaultContext())
    # Create a decision variable for the desired center of mass position
    # This is p_EC in the constraint equation: p_EScm(q) - p_EC = 0
    desired_com_position = IK_solver.prog().NewContinuousVariables(3, "com_position")
    # Add the constraint to the optimization program
    # This enforces that the actual CoM position equals the desired position
    IK_solver.prog().AddConstraint(
        constraint=com_constraint,
        vars=np.concatenate([IK_solver.q(), desired_com_position])
    )
    # Set the desired center of mass position 
    IK_solver.prog().AddLinearEqualityConstraint(
        Aeq=np.eye(3),
        beq=st_P_com,
        vars=desired_com_position
    )
    if q_initial_guess is not None:
        q_initial_guess = q_initial_guess
    else:
        q_initial_guess = np.ones(plant.num_positions())*0.1
    plant.SetPositions(context=plant.CreateDefaultContext(),
                       q=q_initial_guess)
    P_W_com = plant.CalcCenterOfMassPositionInWorld(context=plant.CreateDefaultContext())
    T_W_st = stance_leg.CalcPoseInWorld(context=plant.CreateDefaultContext())
    P_st_com = T_W_st.inverse() @ P_W_com
    initial_guess = np.concatenate((q_initial_guess,P_st_com)) 
    result = Solve(IK_solver.prog(),initial_guess)
    is_success = False
    if result.is_success():
        q_solution = result.GetSolution(q_variables)
        is_success = True
    else:
        q_solution = np.zeros(plant.num_positions())
    threshold = 1.5e-03
    q_solution[abs(q_solution) <= threshold] = 0.0 
    return is_success,q_solution

def CalcForwardKinematics(plant, generalize_positions,base_frame_name="torso", end_effector_frame_name="L_foot_contact"):
    """
    Calculate the forward kinematics of the robot leg.

    Args:
        plant (MultibodyPlant): The plant containing the robot model.
        genaeralize_positions (list): Generalized positions including base pose(if floating base) and joint angles.
        base_frame_name (str): Name of the base frame.
        end_effector_frame_name (str): Name of the end-effector frame.

    Returns:
        np.ndarray: The position of the end-effector frame in the world frame.
    """
    context = plant.CreateDefaultContext()
    plant.SetPositions(context,np.array(generalize_positions))
    base_frame = plant.GetFrameByName(base_frame_name)
    world_T_base = base_frame.CalcPoseInWorld(context)
    end_effector_frame = plant.GetFrameByName(end_effector_frame_name)
    world_T_end_effector = end_effector_frame.CalcPoseInWorld(context)
    base_T_end_effector = world_T_base.inverse() @ world_T_end_effector
    return base_T_end_effector.translation()  # Return only the translation part

def CalcPositionVectorFromCoM(plant:MultibodyPlant, context, target_frame_name:str):
    W_P_com = plant.CalcCenterOfMassPositionInWorld(context)
    target_frame = plant.GetFrameByName(target_frame_name) 
    W_P_target = target_frame.CalcPoseInWorld(context).translation()
    W_P_com_target = W_P_target - W_P_com
    return W_P_com_target

def CalcContactPointPositionsInCoM(plant, context, contact_frame_names):
    positions = []
    for frame_name in contact_frame_names:
        positions.append(CalcPositionVectorFromCoM(
            plant=plant,
            context=context,
            target_frame_name=frame_name
        ))
    return np.array(positions)

def CalcCoMPositionFromStanceLeg(plant, plant_context, stance_leg:str):
        '''
        Docstring for CalcCoMPositionFromStanceLeg
        
        :param plant: Description
        :param plant_context: Description
        :param stance_leg: Description
        :type stance_leg: str "L_foot_contact" or "R_foot_contact"
        '''
        W_P_com = plant.CalcCenterOfMassPositionInWorld(plant_context)
        foot_contact = plant.GetFrameByName(stance_leg)
        current_CoM_position_in_st = foot_contact.CalcPoseInWorld(plant_context).inverse().multiply(W_P_com)
        return current_CoM_position_in_st

def GetRelativeVelocity(plant, 
                        context, 
                        stance_body_name, 
                        swing_body_name):
    """
    Calculates the spatial velocity of the swing body relative to the stance body.
    
    Args:
        plant: The MultibodyPlant object.
        context: The Context object storing the current state (q, v).
        stance_body_name: String name of the stance body (e.g., "right_foot").
        swing_body_name: String name of the swing body (e.g., "left_foot").
        
    Returns:
        v_rel: Linear velocity (3x1 numpy array) of Swing relative to Stance.
        w_rel: Angular velocity (3x1 numpy array) of Swing relative to Stance.
    """
    
    # 1. Get the Body Frames
    # Note: Ensure you access the body by its specific model instance if you have multiple robots
    stance_frame = plant.GetBodyByName(stance_body_name).body_frame()
    swing_frame = plant.GetBodyByName(swing_body_name).body_frame()

    # 2. Compute Relative Spatial Velocity
    # "Velocity of Swing (B) measured in Stance (A), expressed in Stance (E)"
    # This automatically handles the subtraction: V_rel = V_swing - V_stance
    spatial_velocity = swing_frame.CalcSpatialVelocity(
        context = context,
        frame_M = stance_frame, 
        frame_E = stance_frame
    )

    # 3. Extract Linear and Angular components
    # Drake SpatialVelocity is stored as [angular (w), linear (v)]
    w_rel = spatial_velocity.rotational()    # Angular velocity (rad/s)
    v_rel = spatial_velocity.translational() # Linear velocity (m/s)
    angular_vel = w_rel.copy()
    linear_vel = v_rel.copy()
    threshold = 1e-03
    angular_vel[np.abs(angular_vel) < threshold] = 0.0
    linear_vel[np.abs(linear_vel) < threshold] = 0.0
    return angular_vel, linear_vel

def GetCoMLinearVelocityViaMomentum(plant, context):
    """
    Computes CoM linear velocity using system momentum.
    """
    # 1. Calculate Total Mass
    # (Usually constant, can be cached, but shown here for completeness)
    total_mass = plant.CalcTotalMass(context)
    
    # 2. Calculate Spatial Momentum in World Frame
    # Returns a SpatialMomentum object containing [angular (L), linear (p)]
    spatial_momentum = plant.CalcSpatialMomentumInWorldAboutPoint(context,
                                                                  np.zeros(3))
    
    # 3. Extract Linear Momentum (p)
    linear_momentum = spatial_momentum.translational()
    
    # 4. Compute CoM Velocity (v = p / m)
    v_com = linear_momentum / total_mass
    
    return v_com


# Deprecated misspelled names, kept so older notebooks keep working.
CalcfForwaredKinematics = CalcForwardKinematics
SolveIKWithComPositionConstrain = SolveIKWithComPositionConstraint
