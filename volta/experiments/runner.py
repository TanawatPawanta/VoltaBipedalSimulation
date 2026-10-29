"""Build, run and post-process one closed-loop walking experiment.

Signal flow of the diagram built by `_build_diagram`:

    plant state ─► state noiser ─┬─► TaskSpaceFeedback ──► StateEstimator ─► GaitPlanner (IS-MPC)
                                 │                        ▲    (CoM, ZMP)        │ desired CoM / feet / contact
                                 ├─► ContactForceSensor ──┘ (ZMP)                ▼
                                 └────────────────────────────────────► PassivityBasedWBC2 ─► joint torques ─► plant
    DisturbanceGenerator ─► plant applied spatial force (push experiments)

Usage:
    from volta.experiments.runner import run_experiment
    run_experiment("1a", save_graph=True)
"""
import os

import matplotlib.pyplot as plt
import numpy as np
from pydrake.geometry import Box, Rgba, StartMeshcat
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput, ZeroOrderHold
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig

from volta.Leaf_systems.ContactForceSensor import ContactForceSensor
from volta.Leaf_systems.DisturbanceGenerator import DisturbanceGenerator
from volta.Leaf_systems.GaitPlanner import GaitPlanner
from volta.Leaf_systems.NoiseGenerator import RobotStateNoiseGenerator
from volta.Leaf_systems.PassivityBasedWBC2 import PassivityBasedWBC2
from volta.Leaf_systems.StateEstimator import StateEstimator
from volta.Leaf_systems.TaskSpaceFeedback import TaskSpaceFeedback
from volta.Leaf_systems.WalkRecorder import WalkRecorder
from volta.experiments.configs import GetConfigByName
from volta.paths import RESULTS_DATA_DIR, RESULTS_PICS_DIR
from volta.planning.footsteps_planner import FootstepsPlanner
from volta.plotting import PlotCoMZMP, PlotFootsteps, PlotTracking, PlotTrackingCompare
from volta.robot.kinematics import CalcForwardKinematics, SolveIKWithComPositionConstraint, SolveInverseKinematics
from volta.utils import CreateLoggers, DefinePlant

RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RESET = "\033[0m"

MODEL_NAME = "Volta_rev5"
COM_HEIGHT = 0.35            # [m] nominal CoM height (LIP model height)
FIXED_BASE_OFFSET = 0.5      # [m] torso lift when running in fixed-base mode

WBC_GAINS = dict(
    CoM_kp=np.array([4000.0, 4000.0, 4000.0]),
    CoM_kd=np.ones(3) * 300.0,
    CoM_k_stiffness=np.ones(3) * 20.0,
    CoM_k_damping=np.ones(3) * 1.5,
    sw_kp=np.ones(3) * 400.0,
    sw_kd=np.ones(3) * 20.0,
    sw_k_stiffness=np.ones(3) * 10.0,
    sw_k_damping=np.ones(3) * 0.7,
)

# Standard deviations of the measurement noise added to the robot state.
STATE_NOISE_SIGMAS = dict(
    q_base_ori_sig=0.00,    # clean orientation for now
    q_base_pos_sig=0.003,
    q_joint_sig=0.0001,
    v_base_ang_sig=0.00,
    v_base_lin_sig=0.005,
    v_joint_sig=0.005,
)
NO_NOISE_SIGMAS = {k: 0.00 for k in STATE_NOISE_SIGMAS}


def GaitParams(com_height: float = COM_HEIGHT) -> dict:
    """IS-MPC / footstep-planner parameters (durations are in MPC steps)."""
    params = {
        'g': 9.81,
        'h': com_height,
        'foot_size': 0.08,
        'step_height': 0.03,
        'ss_duration': 70,
        'ds_duration': 30,
        'mpc_time_step': 0.01,
        'first_swing': 'rfoot',
        'first_stance': 'lfoot',
        'N': 100,
    }
    params['eta'] = np.sqrt(params['g'] / params['h'])
    return params


def InitialTaskStates(com_height: float = COM_HEIGHT) -> dict:
    """Initial feet / CoM / ZMP states for the gait planner and state estimator."""
    return {
        'lfoot': {'pos': np.array([0, 0, 0, 0, 0.125, 0]), 'vel': np.zeros(6), 'acc': np.zeros(6)},
        'rfoot': {'pos': np.array([0, 0, 0, 0, -0.125, 0]), 'vel': np.zeros(6), 'acc': np.zeros(6)},
        'com':   {'pos': np.array([0, 0, com_height]), 'vel': np.zeros(6), 'acc': np.zeros(3)},
        'zmp':   {'pos': np.zeros(3), 'vel': np.zeros(3), 'acc': np.zeros(3)},
    }


def FootstepsViz(meshcat, footstep_plan, alpha=0.5, z_offset=0.002):
    """
    Visualizes a list of footsteps in Meshcat.

    Args:
        meshcat: The Meshcat instance.
        footstep_plan: List of dicts with 'index', 'pos', 'ang', and 'foot_id'.
        alpha: Transparency value (0.0 to 1.0).
        z_offset: Tiny height increase to prevent flickering (Z-fighting).
    """
    foot_length = 0.23
    foot_width = 0.11
    foot_thickness = 0.002
    color_left = Rgba(1, 0, 0, alpha)   # Red
    color_right = Rgba(0, 0, 1, alpha)  # Blue

    for step in footstep_plan:
        path = f"footstep_plan/step_{step['index']}"
        color = color_left if step['foot_id'] == 'lfoot' else color_right
        meshcat.SetObject(path, Box(foot_length, foot_width, foot_thickness), color)
        pose = RigidTransform(RollPitchYaw(step['ang']), step['pos'] + np.array([0, 0, z_offset]))
        meshcat.SetTransform(path, pose)


def _initial_configuration(set_initial_pose_by: str, fixed_base: bool):
    """Solve the initial joint positions q and the torso position in the world.

    set_initial_pose_by: "q" all joints zero, "h" torso height, "c" CoM position (default).
    """
    plant_fixed_base, _ = DefinePlant(model_name=MODEL_NAME, is_fixed=True, fixed_frame="torso")
    lift = FIXED_BASE_OFFSET * fixed_base
    match set_initial_pose_by:
        case "q":
            q = np.zeros(12)
            torso_positions = CalcForwardKinematics(plant=plant_fixed_base,
                                                      generalize_positions=q,
                                                      base_frame_name="L_foot_contact",
                                                      end_effector_frame_name="torso")
            print(f"Torso height: {torso_positions[2]} m")
            init_pos = [0.0, 0.0, torso_positions[2] + lift]
        case "h":
            desire_torso_height = 0.5  # [m]
            _, left_q = SolveInverseKinematics(model_series="Volta",
                                               plant=plant_fixed_base,
                                               is_left=True,
                                               base_P_e=np.array([0.0, 0.125, -1.0 * desire_torso_height]),
                                               base_R_e=RollPitchYaw(0, 0, 0).ToRotationMatrix())
            _, right_q = SolveInverseKinematics(model_series="Volta",
                                                plant=plant_fixed_base,
                                                is_left=False,
                                                base_P_e=np.array([0.0, -0.125, -1.0 * desire_torso_height]),
                                                base_R_e=RollPitchYaw(0, 0, 0).ToRotationMatrix())
            init_pos = [0.0, 0.0, desire_torso_height + lift]
            q = np.concatenate((left_q, right_q))
        case "c":
            is_success, q = SolveIKWithComPositionConstraint(model_series="Volta",
                                                            plant=plant_fixed_base,
                                                            is_L_stance=True,
                                                            st_P_com=np.array([0.0, -0.125, COM_HEIGHT]),
                                                            st_R_base=RotationMatrix.Identity(),
                                                            st_P_sw=np.array([0.0, -0.25, 0.0]),
                                                            st_R_sw=RotationMatrix.Identity())
            torso_positions = CalcForwardKinematics(plant=plant_fixed_base,
                                                      generalize_positions=q,
                                                      base_frame_name="L_foot_contact",
                                                      end_effector_frame_name="torso")
            init_pos = [torso_positions[0], torso_positions[1] + 0.125, torso_positions[2] + lift]
            print(is_success)
        case _:
            raise ValueError(f"Unknown set_initial_pose_by: {set_initial_pose_by!r}")
    return q, init_pos


def _add_plant(builder, exp_config, sim_time_step, gravity, init_pos, init_orien, fixed_base):
    """Add the robot and floor to a MultibodyPlant and set the robot's default pose."""
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    plant: MultibodyPlant = plant
    parser = Parser(plant)
    plant.mutable_gravity_field().set_gravity_vector(gravity)

    robot_instance = parser.AddModelsFromString(exp_config.robot_urdf, "urdf")
    if exp_config.multi_terrain:
        parser.AddModelsFromString(exp_config.multi_terrain_sdf, "sdf")
        floor_frame = plant.GetFrameByName("floor_0_frame")
    else:
        parser.AddModelsFromString(exp_config.floor_sdf, "sdf")
        floor_frame = plant.GetFrameByName("floor_top_center")
    plant.WeldFrames(plant.world_frame(), floor_frame, RigidTransform(RollPitchYaw([0, 0, 0]), p=[0.0, 0, -0.0]))

    X_floorRobot = RigidTransform(RollPitchYaw(init_orien * np.pi / 180), p=init_pos)
    if fixed_base:
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("torso"), X_floorRobot)
        print("Simulation in Fixed based mode")
    else:
        print("Simulation in Floating based mode")
    plant.Finalize()

    plant_context = plant.CreateDefaultContext()
    X_Worldfloor = floor_frame.CalcPoseInWorld(plant_context)
    plant.SetDefaultFloatingBaseBodyPose(plant.GetBodyByName("torso"), X_Worldfloor.multiply(X_floorRobot))
    return plant, plant_context, robot_instance


def _build_diagram(exp_config, meshcat, q, init_pos, planner, params, initial, *,
                   sim_time_step, gravity, fixed_base, force_viz, enable_gait_planner, enable_noise):
    """Assemble the closed-loop diagram. Returns (diagram, plant, robot_instance, systems, loggers)."""
    builder = DiagramBuilder()
    init_orien = np.asarray([0, 0, 0])  # body frame orientation [deg], fixed xyz axes
    plant, plant_context, robot_instance = _add_plant(
        builder, exp_config, sim_time_step, gravity, init_pos, init_orien, fixed_base)

    vis_config = VisualizationConfig()
    vis_config.publish_contacts = force_viz
    # Disable Meshcat mouse spring: it claims applied_spatial_force, which the pusher needs
    vis_config.mouse_interaction_stiffness = None
    ApplyVisualizationConfig(vis_config, builder, meshcat=meshcat)

    # --- Systems ---
    contact_force_sensor = builder.AddNamedSystem("Contact force sensor",
                                                  ContactForceSensor(dt=0.01, plant=plant))
    # Controller-side model of the robot (floating base, no floor).
    plant_floating_base, _ = DefinePlant(model_name=MODEL_NAME, is_fixed=False, fixed_frame=None)
    walk_recorder = builder.AddNamedSystem("Walk_recorder",
                                           WalkRecorder(meshcat=meshcat,
                                                        plant=plant,
                                                        context=plant_context,
                                                        model_instances=robot_instance))
    gait_planner = builder.AddNamedSystem("ISMPC",
                                          GaitPlanner(plant=plant_floating_base,
                                                      footsteps_planner=planner,
                                                      initial_states=initial,
                                                      params=params,
                                                      enable=enable_gait_planner))
    wbc = builder.AddNamedSystem("Passivity based WBC",
                                 PassivityBasedWBC2(plant=plant_floating_base,
                                                    **WBC_GAINS,
                                                    initial_joint_position=q,
                                                    dt=0.001))
    taskspace_feedback = builder.AddNamedSystem("Taskspace feedback",
                                                TaskSpaceFeedback(plant=plant_floating_base))
    groundtruth_feedback = builder.AddNamedSystem("Groundtruth taskspace feedback",
                                                  TaskSpaceFeedback(plant=plant_floating_base))
    sigmas = STATE_NOISE_SIGMAS if enable_noise else NO_NOISE_SIGMAS
    state_noiser = builder.AddSystem(RobotStateNoiseGenerator(num_joints=12, **sigmas))
    zmp_buffer = builder.AddSystem(ZeroOrderHold(period_sec=0.001, vector_size=3))
    state_estimator = builder.AddNamedSystem("state estimator",
                                             StateEstimator(dt=0.001, initial=initial, mpc=gait_planner.mpc))
    pusher = builder.AddSystem(DisturbanceGenerator(plant,
                                                    body_name="torso",
                                                    force_xyz=exp_config.forces,
                                                    tau=exp_config.tau,
                                                    start_time=exp_config.apply_wrenches_t,
                                                    duration=0.1))

    # --- Wiring ---
    clean_state_port = plant.get_state_output_port(robot_instance[0])
    builder.Connect(clean_state_port, state_noiser.get_input_port(0))
    noisy_state_port = state_noiser.get_output_port(0)

    # Measurements: raw CoM / feet from the noisy state, ground-truth CoM from the clean state.
    builder.Connect(noisy_state_port, taskspace_feedback.get_input_port(0))
    builder.Connect(clean_state_port, groundtruth_feedback.get_input_port(0))
    # Contact forces and raw ZMP.
    builder.Connect(plant.get_contact_results_output_port(), contact_force_sensor.get_input_port(0))
    builder.Connect(noisy_state_port, contact_force_sensor.get_input_port(1))

    # Estimator inputs: 0 raw CoM, 1 measured ZMP, 2 u = planned ZMP velocity (planner -> buffer).
    builder.Connect(taskspace_feedback.get_output_port(0), state_estimator.get_input_port(0))
    builder.Connect(contact_force_sensor.get_output_port(1), state_estimator.get_input_port(1))
    builder.Connect(gait_planner.get_output_port(5), zmp_buffer.get_input_port())
    builder.Connect(zmp_buffer.get_output_port(), state_estimator.get_input_port(2))

    # Planner runs on the filtered CoM and ZMP.
    builder.Connect(state_estimator.get_output_port(0), gait_planner.get_input_port(0))
    builder.Connect(state_estimator.get_output_port(1), gait_planner.get_input_port(1))

    # WBC inputs: 0 robot state, 1-4 planner references (CoM, lfoot, rfoot, contact),
    # 5 filtered CoM, 6-7 measured feet, 8 CoP in foot frame.
    builder.Connect(noisy_state_port, wbc.get_input_port(0))
    for port_idx in range(4):
        builder.Connect(gait_planner.get_output_port(port_idx), wbc.get_input_port(port_idx + 1))
    builder.Connect(state_estimator.get_output_port(0), wbc.get_input_port(5))
    for port_idx in [1, 2]:
        builder.Connect(taskspace_feedback.get_output_port(port_idx), wbc.get_input_port(port_idx + 5))
    builder.Connect(contact_force_sensor.get_output_port(2), wbc.get_input_port(8))
    builder.Connect(wbc.get_output_port(0), plant.get_actuation_input_port(robot_instance[0]))

    builder.Connect(pusher.get_output_port(), plant.get_applied_spatial_force_input_port())

    # --- Loggers ---
    loggers = {}
    for system, ports, names in [
        (wbc, [0, 1, 2, 3], ["torque_command", "desired_foot_wrenches", "fz_boundaries", "walking_phase"]),
        (taskspace_feedback, [0, 1, 2], ["CoM_state", "lfoot_state", "rfoot_state"]),
        (state_estimator, [0, 1], ["filtered_CoM_state", "filtered_ZMP"]),
        (groundtruth_feedback, [0], ["groundtruth_CoM_state"]),
        (contact_force_sensor, [0, 1, 2, 3], ["GRF", "ZMP", "ZMP_in_foot_frame", "total_contact_wrenches"]),
        (gait_planner, [0, 1, 2, 3, 4], ["COM_traj", "L_foot_traj", "R_foot_traj", "contact", "desired_zmp"]),
    ]:
        created = CreateLoggers(builder=builder, system=system, port_num=ports,
                                name=[f"{n}_logger" for n in names])
        loggers.update(zip(names, created))
    loggers["net_torque"] = LogVectorOutput(plant.get_net_actuation_output_port(robot_instance[0]), builder)
    loggers["net_torque"].set_name("net_torque_logger")

    diagram = builder.Build()
    diagram.set_name("Robot simulation")
    systems = dict(walk_recorder=walk_recorder, gait_planner=gait_planner, wbc=wbc)
    return diagram, plant, robot_instance, systems, loggers


def _simulate(diagram, plant, robot_instance, walk_recorder, meshcat, q, init_pos, *,
              sim_time, fixed_base, realtime_rate):
    """Set the initial state and advance the simulation. Returns (simulator, context, result string)."""
    simulator = Simulator(diagram)
    simulator.reset_context(diagram.CreateDefaultContext())
    context = simulator.get_mutable_context()
    plant_context = plant.GetMyContextFromRoot(context)
    walk_recorder.root_context = context

    body_quaternion_wxyz = [1.0, 0.0, 0.0, 0.0]
    if fixed_base:
        initial_positions = np.asarray(q)
    else:
        initial_positions = np.concatenate((body_quaternion_wxyz, init_pos, q))
    plant.SetPositions(plant_context, robot_instance[0], initial_positions)

    simulator.Initialize()
    simulator.set_target_realtime_rate(realtime_rate)
    meshcat.StartRecording()
    try:
        simulator.AdvanceTo(sim_time)
        result = f"{GREEN}SUCCEEDED{RESET}"
    except Exception as e:
        print("Simulation error:", e)
        result = f"{RED}FAILED{RESET}"
    meshcat.PublishRecording()
    return simulator, context, result


def _walking_phase(contact_data):
    """Contact flags [left, right] -> phase code: 0 left stance, 1 right stance, 2 double stance."""
    walking_phase = []
    for i in range(contact_data.shape[1]):
        if contact_data[0, i] == 1 and contact_data[1, i] == 0:
            walking_phase.append(0)
        elif contact_data[0, i] == 0 and contact_data[1, i] == 1:
            walking_phase.append(1)
        elif contact_data[0, i] == 1 and contact_data[1, i] == 1:
            walking_phase.append(2)
    return np.array(walking_phase)


def _save_results(path, time, logs, walking_phase):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez_compressed(
        path,
        timestamp       = time,
        desired_zmp     = logs["desired_zmp"],
        desired_com     = logs["COM_traj"],
        desired_lfoot   = logs["L_foot_traj"],
        desired_rfoot   = logs["R_foot_traj"],
        walking_phase   = walking_phase,
        raw_ZMP         = logs["ZMP"],
        raw_com         = logs["CoM_state"],
        raw_lfoot       = logs["lfoot_state"],
        raw_rfoot       = logs["rfoot_state"],
        groundtruth_CoM = logs["groundtruth_CoM_state"],
        filtered_CoM    = logs["filtered_CoM_state"],
        filtered_ZMP    = logs["filtered_ZMP"],
        GRF             = logs["GRF"],
        torque_command  = logs["torque_command"],
        desired_foot_wrenches = logs["desired_foot_wrenches"],
    )


def _plot_results(exp_config, plan, params, time, logs, walking_phase, *,
                  sim_time, sim_time_step, total_sim_time, save_path, show_plot):
    """All standard figures of one run (saved to `save_path` when given)."""
    name = exp_config.exp_name[0:6]
    plot_interval = (0.005, sim_time)  # [s]
    desired_com = logs["COM_traj"]
    desired_lfoot = logs["L_foot_traj"]
    desired_rfoot = logs["R_foot_traj"]
    desired_zmp = logs["desired_zmp"]
    actual_com = logs["CoM_state"]
    actual_lfoot = logs["lfoot_state"]
    actual_rfoot = logs["rfoot_state"]
    filtered_com = logs["filtered_CoM_state"]
    filtered_zmp = logs["filtered_ZMP"]
    groundtruth_com = logs["groundtruth_CoM_state"]
    raw_zmp = logs["ZMP"]
    common = dict(plot_interval=plot_interval, save_folder=save_path)

    # Estimator quality
    PlotTrackingCompare(time_log=time[::10], ref_data=groundtruth_com[3:6, ::10],
                        act_data=[actual_com[3:6, ::10], filtered_com[3:6, ::10]],
                        phase_codes=walking_phase[::10], act_labels=["Raw CoM position", "Filtered CoM position"],
                        act_colors=None, dim_labels=['X', 'Y', 'Z'], unit='m', title="Filtered CoM Position",
                        show_error=False, show_plot=show_plot, **common)
    PlotTrackingCompare(time_log=time[::10], ref_data=groundtruth_com[9:12, ::10],
                        act_data=[actual_com[9:12, ::10], filtered_com[9:12, ::10]],
                        phase_codes=walking_phase[::10], act_labels=["Raw CoM position", "Filtered CoM position"],
                        act_colors=None, dim_labels=['X', 'Y', 'Z'], unit='m/s', title="Filtered CoM Velocity",
                        show_error=False, show_plot=show_plot, **common)
    PlotCoMZMP(time_log=time[::100], CoM_data=filtered_zmp[:, ::100], ZMP_data=raw_zmp[:, ::100],
               title="Filtered ZMP", phase_codes=walking_phase[::100], show_error=False,
               labels=["Raw ZMP", "Filtered ZMP"], show_plot=show_plot, **common)

    # CoM tracking
    PlotTracking(time_log=time, unit="rad", ref_data=desired_com[0:3, :], act_data=actual_com[0:3, :],
                 title=f"{name}: CoM Orientation Tracking", phase_codes=walking_phase,
                 show_error=False, show_plot=show_plot, **common)
    PlotTracking(time_log=time[::10], ref_data=desired_com[3:6, ::10], act_data=filtered_com[3:6, ::10],
                 title=f"{name}: CoM Position Tracking", phase_codes=walking_phase[::10],
                 show_error=True, show_plot=show_plot, **common)
    PlotCoMZMP(time_log=time[::10], CoM_data=desired_com[3:6, ::10], ZMP_data=desired_zmp[0:3, ::10],
               title=f"{name}: Planned ZMP-CoM", phase_codes=walking_phase[::10], show_error=False,
               labels=["Desired ZMP", "Desired CoM"], show_plot=show_plot, **common)

    # Reference velocity tracking (v_ref is shifted by `offset` samples = 2 s of standing at the start)
    ts = np.linspace(0, total_sim_time, int(total_sim_time / sim_time_step))
    com_vel = np.array([filtered_com[9, :ts.shape[0]],
                        filtered_com[10, :ts.shape[0]],
                        filtered_com[8, :ts.shape[0]]])
    offset = 20000
    v_data = np.array([exp_config.VRefCommand(t) for t in ts[0:-offset]])
    v_ref = np.hstack((np.ones([3, offset]) * 0.0, v_data.T))
    PlotTracking(time_log=ts, ref_data=v_ref, dim_labels=["Vx [m/s]", "Vy [m/s]", "Omega [rad/s]"], unit=None,
                 act_data=com_vel, title=f"{name}: Reference Velocity Tracking", phase_codes=walking_phase,
                 show_error=False, labels=["CoM velocity", "Reference velocity"], **common)

    # Feet tracking
    for label, desired, actual in [("Left", desired_lfoot, actual_lfoot), ("Right", desired_rfoot, actual_rfoot)]:
        PlotTracking(time_log=time[::10], unit="rad", ref_data=desired[0:3, ::10], act_data=actual[0:3, ::10],
                     title=f"{name}: {label} Foot Orientation Tracking", phase_codes=walking_phase[::10],
                     show_error=False, show_plot=show_plot, **common)
        PlotTracking(time_log=time[::10], ref_data=desired[3:6, ::10], act_data=actual[3:6, ::10],
                     title=f"{name}: {label} Foot Position Tracking", phase_codes=walking_phase[::10],
                     show_error=True, show_plot=show_plot, **common)

    PlotTracking(time_log=time[::10], unit="m/s^2", ref_data=desired_com[15:18, ::10],
                 act_data=desired_com[15:18, ::10], title=f"{name}: Planned CoM acceleration",
                 phase_codes=walking_phase[::10], show_error=False, show_plot=show_plot, **common)

    # ZMP tracking
    PlotCoMZMP(time_log=time[::100], CoM_data=desired_zmp[:, ::100], ZMP_data=filtered_zmp[:, ::100] + 0.000,
               title=f"{name}: ZMP Tracking", phase_codes=walking_phase[::100], show_error=False,
               labels=["Actual ZMP", "Desired ZMP"], show_plot=show_plot, **common)
    PlotCoMZMP(time_log=time[::100], ZMP_data=filtered_zmp[:, ::100] + 0.000, CoM_data=filtered_com[3:6, ::100],
               title=f"{name}: Actual ZMP-CoM", phase_codes=walking_phase[::100],
               plot_interval=(0.011, sim_time), save_folder=save_path, show_error=False,
               labels=["Actual ZMP", "Actual CoM"], show_plot=show_plot)

    # Top views over the footstep plan
    actual_com_xy = [filtered_com[3, 500::100], filtered_com[4, 500::100], "Actual CoM"]
    actual_zmp_xy = [filtered_zmp[0, 500::100], filtered_zmp[1, 500::100], "Actual ZMP"]
    desired_com_xy = [desired_com[3, 500::100], desired_com[4, 500::100], "Desired CoM"]
    desired_zmp_xy = [desired_zmp[0, 500::100], desired_zmp[1, 500::100], "Desired ZMP"]
    footsteps = dict(steps=plan, save_folder=save_path, support_size=[params['foot_size'], params['foot_size']])
    PlotFootsteps(plot_1=actual_com_xy, plot_2=actual_zmp_xy, plot_3=desired_com_xy, plot_4=desired_zmp_xy,
                  title=f"{name}: Tracking Performance", **footsteps)
    PlotFootsteps(plot_2=desired_zmp_xy, plot_1=actual_com_xy,
                  title=f"{name}: Actual CoM and Desired ZMP", **footsteps)
    PlotFootsteps(plot_1=actual_zmp_xy, plot_2=desired_zmp_xy,
                  title=f"{name}: ZMP Tracking Performance", **footsteps)
    PlotFootsteps(plot_1=desired_com_xy, plot_2=desired_zmp_xy,
                  title=f"{name}: Planned CoM and ZMP", **footsteps)
    plt.close('all')


def run_experiment(experiment_name: str = "1a", save_graph: bool = False, show_plot: bool = False, *,
                   fixed_base: bool = False,
                   gravity_enable: bool = True,
                   force_viz: bool = False,
                   enable_gait_planner: bool = True,
                   enable_noise: bool = True,
                   set_initial_pose_by: str = "c",
                   realtime_rate: float = 0.15):
    """Run one experiment from `configs.py` end to end.

    With save_graph=True the logs go to M_result/Final/Data/<exp>[_noise].npz and the
    figures to M_result/Final/Pics/<exp_name>[_noise]/. Returns a colored SUCCEEDED/FAILED string.
    """
    print(f"Running experiment: {experiment_name}")
    meshcat = StartMeshcat()
    port = str(meshcat.web_url())[-4:]
    print(f"Sharing URL: tanawatp@tanawat-ROG.local:{port}")
    print(f"Download URL: {meshcat.web_url()}/download")

    exp_config = GetConfigByName(experiment_name)
    exp_config.DisplayConfig()
    sim_time_step = exp_config.sim_timestep  # [s]
    sim_time = exp_config.sim_time           # [s]
    gravity = [0, 0, -9.81 * gravity_enable]
    noise_suffix = "_noise" if enable_noise else ""
    save_path = f"{RESULTS_PICS_DIR}/{exp_config.exp_name}{noise_suffix}" if save_graph else None

    q, init_pos = _initial_configuration(set_initial_pose_by, fixed_base)
    params = GaitParams(COM_HEIGHT)
    initial = InitialTaskStates(COM_HEIGHT)

    planner = FootstepsPlanner(params=params)
    plan = planner.PlanFootsteps([0.0, 0.125, 0.0], exp_config.VRefCommand,
                                 np.clip(sim_time - 2.0, min=1.0), params['first_stance'])
    meshcat.Delete()
    meshcat.DeleteAddedControls()
    FootstepsViz(meshcat, plan, alpha=0.3, z_offset=0.0001)

    diagram, plant, robot_instance, systems, loggers = _build_diagram(
        exp_config, meshcat, q, init_pos, planner, params, initial,
        sim_time_step=sim_time_step, gravity=gravity, fixed_base=fixed_base, force_viz=force_viz,
        enable_gait_planner=enable_gait_planner, enable_noise=enable_noise)
    simulator, context, result = _simulate(
        diagram, plant, robot_instance, systems["walk_recorder"], meshcat, q, init_pos,
        sim_time=sim_time, fixed_base=fixed_base, realtime_rate=realtime_rate)

    logs = {name: logger.FindLog(context).data() for name, logger in loggers.items()}
    time = loggers["CoM_state"].FindLog(context).sample_times()
    walking_phase = _walking_phase(logs["contact"])

    if save_graph:
        _save_results(os.path.join(RESULTS_DATA_DIR, f"{exp_config.exp_name[0:6]}{noise_suffix}.npz"),
                      time, logs, walking_phase)
    _plot_results(exp_config, plan, params, time, logs, walking_phase,
                  sim_time=sim_time, sim_time_step=sim_time_step,
                  total_sim_time=simulator.get_mutable_context().get_time(),
                  save_path=save_path, show_plot=show_plot)
    return result


# Name used by existing notebooks.
RunExperimentV2 = run_experiment
