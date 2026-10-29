"""Everything a simulation notebook usually needs, in one import.

    from volta.prelude import *

Library code should import what it uses explicitly instead.
"""
# Standard / third-party
import copy
import math
import os
import re
import sys

import casadi as cs
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import numpy as np
import pydrake.solvers as solvers
from scipy.spatial.transform import Rotation as R

# pydrake
from pydrake.common import temp_directory
from pydrake.geometry import Box, Meshcat, Rgba, SceneGraphConfig, Sphere, StartMeshcat
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.math import SpatialForce
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.multibody.tree import MultibodyForces
from pydrake.solvers import Solve
from pydrake.symbolic import Jacobian, Variable, cos, sin
from pydrake.systems.analysis import Simulator
from pydrake.systems.controllers import PidController
from pydrake.systems.framework import BasicVector, DiagramBuilder, LeafSystem
from pydrake.systems.primitives import LogVectorOutput, ZeroOrderHold
from pydrake.trajectories import PiecewisePolynomial
from pydrake.visualization import (
    AddDefaultVisualization,
    ApplyVisualizationConfig,
    ModelVisualizer,
    VisualizationConfig,
)

from volta.paths import MODELS_DIR, PROJECT_ROOT, RESULT_ROOT, RESULTS_DATA_DIR, RESULTS_PICS_DIR

# Drake LeafSystems
from volta.Leaf_systems.ContactForceSensor import ContactForceSensor
from volta.Leaf_systems.CubicTrajectory3D import CubicTrajectory3D
from volta.Leaf_systems.DeMux2 import DeMux2
from volta.Leaf_systems.DisturbanceGenerator import DisturbanceGenerator
from volta.Leaf_systems.DummyParameters import DummyParameters
from volta.Leaf_systems.DummyParameters2 import DummyParameters2
from volta.Leaf_systems.GaitPlanner import GaitPlanner
from volta.Leaf_systems.GravityCompensation import GravityCompensation
from volta.Leaf_systems.InverseDynamicsPDController import InverseDynamicsPDController
from volta.Leaf_systems.JointStateRecorder import JointStateRecorder
from volta.Leaf_systems.LIPMGaitGeneration import LIPMGaitGeneration
from volta.Leaf_systems.MITController import MITController
from volta.Leaf_systems.NoiseGenerator import RobotStateNoisGenerator, RobotStateNoiseGenerator
from volta.Leaf_systems.PassivityBasedWBC2 import PassivityBasedWBC2
from volta.Leaf_systems.PathTracer import PathTracer
from volta.Leaf_systems.RateTransition import RateTransition
from volta.Leaf_systems.ReferenceJointTrajectories import ReferenceJointTrajectories
from volta.Leaf_systems.ReferenceTaskSpaceTrajectories import ReferenceTaskSpaceTrajectories
from volta.Leaf_systems.RobotStateFeedback import RobotStateFeedback
from volta.Leaf_systems.SamplingU import SamplingU
from volta.Leaf_systems.StateEstimator import StateEstimator
from volta.Leaf_systems.Sum import Sum
from volta.Leaf_systems.TaskSpaceFeedback import TaskSpaceFeedback
from volta.Leaf_systems.WalkRecorder import WalkRecorder

# Functions and helpers
from volta.experiments.configs import *
from volta.experiments.runner import *
from volta.experiments.velocity_profiles import *
from volta.planning.footsteps_planner import *
from volta.planning.ismpc import *
from volta.planning.offline_planning import *
from volta.plotting import *
from volta.robot.dynamics import *
from volta.robot.kinematics import *
from volta.test_objects import *
from volta.utils import *
