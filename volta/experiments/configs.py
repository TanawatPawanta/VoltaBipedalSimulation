"""Experiment definitions. Look one up with GetConfigByName("1a").

Groups (keys are "exp_<id>"):
    exp_1  0a standing; 1a-1g walking patterns (straight, diagonal, circular, rotate, L, combined, fwd/back)
    exp_2  soft / softer floor (stiffness)
    exp_3  low floor dissipation
    exp_4  mixed-stiffness / uneven tiled terrain
    exp_5  joint damping
    exp_6  walking speed
    exp_7  external push on the torso
"""
from volta.experiments.scene_descriptions import floor_sdf, multi_terrain_sdf, volta_urdf
from volta.experiments.velocity_profiles import GetVelocityProfile
from volta.paths import MODELS_DIR


class ExperimentConfig:
    """Physical and command parameters of one experiment, plus the scene/robot model strings built from them."""

    def __init__(self,exp_name:str, 
                 profile_type:str, 
                 vel:float=0.1,
                 floor_stiffness:float=5e6, 
                 dissipation:float=50.0, 
                 joint_damping:float=0.3, 
                 noise_std:float=0.0,
                 sim_time:float=20.0,
                 sim_timestep:float=0.0001,
                 joint_torque_limit:float=12.0,
                 multi_terrain:bool=False,
                 floor_prop = None,
                 wrenches:list=[0,0,0,0,0,0],
                 apply_wrenches_t:float=0.0):
        self.exp_name = exp_name
        self.profile_type = profile_type
        self.floor_stiffness = floor_stiffness
        self.dissipation = dissipation
        self.joint_damping = joint_damping
        self.joint_torque_limit = joint_torque_limit
        self.noise_std = noise_std
        self.sim_time = sim_time
        self.sim_timestep = sim_timestep
        self.multi_terrain = multi_terrain
        self.tau = wrenches[0:3]
        self.forces = wrenches[3:6]
        self.apply_wrenches_t = apply_wrenches_t
        self.mesh_dir = f"file://{MODELS_DIR}/Volta_rev5/meshes"
        self.VRefCommand = GetVelocityProfile(profile_type, vel)
        self.floor_sdf = floor_sdf(floor_stiffness, dissipation)
        self.multi_terrain_sdf, self.surface_position = multi_terrain_sdf(properties=floor_prop)
        self.robot_urdf = volta_urdf(self.mesh_dir, joint_damping, joint_torque_limit)

    def DisplayConfig(self):
        print(f"Experiment Name: {self.exp_name}")
        print(f"Profile Type: {self.profile_type}")
        print(f"Floor Stiffness: {self.floor_stiffness:.0e}")
        print(f"Dissipation: {self.dissipation}")
        print(f"Joint Damping: {self.joint_damping}")
        print(f"Noise Std Dev: {self.noise_std}")
        print(f"Simulation Time: {self.sim_time}")
        print(f"Simulation Time Step: {self.sim_timestep}")


exp_1 = {
    "exp_0a": ExperimentConfig(
        exp_name="exp_0a_stablization",
        profile_type="0",
        floor_stiffness=5e6,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        wrenches=[0.0, 0.0, 0.0,
                  12.0, 10.0, 15.0],
        apply_wrenches_t=1.0,
        sim_time=3.0,
        sim_timestep=0.0001
    ),
    "exp_1a": ExperimentConfig(
        exp_name="exp_1a_straight_walk",
        profile_type="straight",
        floor_stiffness=5e6,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=15.0,
        sim_timestep=0.0001
    ),
    "exp_1b": ExperimentConfig(
        exp_name="exp_1b_diagonal_walk",
        profile_type="diagonal",
        floor_stiffness=5e6,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=15.0,
        sim_timestep=0.0001
    ),
    "exp_1c": ExperimentConfig(
        exp_name="exp_1c_circular_walk",
        profile_type="circular",
        floor_stiffness=5e6,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=15.0,
        sim_timestep=0.0001
    ),
    "exp_1d": ExperimentConfig(
        exp_name="exp_1d_rotate_around",
        profile_type="rotational",
        floor_stiffness=5e6,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=15.0,
        sim_timestep=0.0001
    ),
    "exp_1e": ExperimentConfig(
        exp_name="exp_1e_L_walk",
        profile_type="L",        
        floor_stiffness=5e6,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=15.0,
        sim_timestep=0.0001
    ),
    "exp_1f": ExperimentConfig(
        exp_name="exp_1f_combined_walk",
        profile_type="combined",        
        floor_stiffness=5e6,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=25.0,
        sim_timestep=0.0001
    ),
    "exp_1g": ExperimentConfig(
        exp_name="exp_1g_forward_backward_walk",
        profile_type="forward_backward",        
        floor_stiffness=5e6,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=10.0,
        sim_timestep=0.0001
    )
    }

exp_2 = {
    "exp_2a": ExperimentConfig(
        exp_name="exp_2a_soft_floor_combined",
        profile_type="combined",
        floor_stiffness=5e3,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=25.0,
        sim_timestep=0.0001
    ),
    "exp_2b": ExperimentConfig(
        exp_name="exp_2b_soft_floor_L",
        profile_type="L",
        floor_stiffness=5e3,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=25.0,
        sim_timestep=0.0001
    ),
    "exp_2c": ExperimentConfig(
        exp_name="exp_2c_softer_floor_combined",
        profile_type="combined",
        floor_stiffness=2e3,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=25.0,
        sim_timestep=0.0001
    ),
    "exp_2d": ExperimentConfig(
        exp_name="exp_2d_softer_floor_L",
        profile_type="L",
        floor_stiffness=2e3,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=25.0,
        sim_timestep=0.0001
    )
}

exp_3 = {
    "exp_3a": ExperimentConfig(
        exp_name="exp_3a_low_dissipation_30",
        profile_type="combined",
        floor_stiffness=5e3,
        dissipation=30.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=25.0,
        sim_timestep=0.0001
    ),
    "exp_3b": ExperimentConfig(
        exp_name="exp_3b_low_dissipation_20",
        profile_type="combined",
        floor_stiffness=5e3,
        dissipation=20.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=25.0,
        sim_timestep=0.0001
    ),
    "exp_3c": ExperimentConfig(
        exp_name="exp_3c_low_dissipation_10",
        profile_type="combined",
        floor_stiffness=2e3,
        dissipation=10.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=25.0,
        sim_timestep=0.0001
    ),
    "exp_3d": ExperimentConfig(
        exp_name="exp_3d",
        profile_type="straight_10",
        floor_stiffness=2e3,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=5.0,
        sim_timestep=0.0001
    ),
    "exp_3e": ExperimentConfig(
        exp_name="exp_3e",
        profile_type="straight_10",
        floor_stiffness=2e3,
        dissipation=10.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=5.0,
        sim_timestep=0.0001
    ),
    "exp_3f": ExperimentConfig(
        exp_name="exp_3f_no_stblization",
        profile_type="straight_10",
        floor_stiffness=2e3,
        dissipation=10.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=5.0,
        sim_timestep=0.0001
    )
}

exp_4 = {
    "exp_4a": ExperimentConfig(
        exp_name="exp_4a_mixed_floor",
        profile_type="straight_20",
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=25.0,
        sim_timestep=0.0001,
        multi_terrain=True,
        floor_prop= {
                'orange': {'k': 5e6, 'd': 50, 'mu': 1.0, 'color': [0.9, 0.6, 0.4, 0.5]},
                'green':  {'k': 4e3, 'd': 30, 'mu': 1.0, 'color': [0.4, 0.8, 0.4, 0.5]},
                'yellow': {'k': 5e3, 'd': 50, 'mu': 1.0, 'color': [0.9, 0.9, 0.4, 0.5]},
                'blue':   {'k': 2e4, 'd': 10, 'mu': 1.0, 'color': [0.4, 0.6, 0.9, 0.5]}
            }
    ),
    "exp_4b": ExperimentConfig(
        exp_name="exp_4b_mixed_floor_high",
        profile_type="straight_20",
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=25.0,
        sim_timestep=0.0001,
        multi_terrain=True,
        floor_prop= {
                'orange': {'k': 5e6, 'd': 50, 'mu': 1.0, 'color': [0.9, 0.6, 0.4, 1]},
                'green':  {'k': 5e6, 'd': 50, 'mu': 1.0, 'color': [0.4, 0.8, 0.4, 1]},
                'yellow': {'k': 5e6, 'd': 50, 'mu': 1.0, 'color': [0.9, 0.9, 0.4, 1]},
                'blue':   {'k': 5e6, 'd': 50, 'mu': 1.0, 'color': [0.4, 0.6, 0.9, 1]}
            }
    )
}

exp_5 = {
    "exp_5a": ExperimentConfig(
        exp_name="exp_5a_damping_0.2",
        profile_type="straight_10",
        vel=0.1,
        floor_stiffness=5e6,
        dissipation=50.0,
        joint_damping=0.2,
        noise_std=0.0,
        sim_time=15.0,
        sim_timestep=0.0001
    ),
    "exp_5b": ExperimentConfig(
        exp_name="exp_5b_damping_0.1",
        profile_type="straight_10",
        vel=0.1,
        floor_stiffness=5e6,
        dissipation=50.0,
        joint_damping=0.1,
        noise_std=0.0,
        sim_time=15.0,
        sim_timestep=0.0001
    )
}

exp_6 = {
    "exp_6a": ExperimentConfig(
        exp_name="exp_6a",
        profile_type="straight_10",
        vel=0.15,
        floor_stiffness=5e6,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=15.0,
        sim_timestep=0.0001
    ),
    "exp_6b": ExperimentConfig(
        exp_name="exp_6b",
        profile_type="straight_10",
        vel=0.2,
        floor_stiffness=5e6,
        dissipation=50.0,
        joint_damping=0.3,
        noise_std=0.0,
        sim_time=15.0,
        sim_timestep=0.0001
    )
}

exp_7 = {
    "exp_7a": ExperimentConfig(
        exp_name="exp_7a_fx",
        profile_type="straight_10",
        vel=0.1,
        wrenches=[0.0, 0.0, 0.0,
                  12.0, 0.0, 0.0],
        apply_wrenches_t=3.0,
        sim_time=4.0
    ),
    "exp_7b": ExperimentConfig(
        exp_name="exp_7b_fy",
        profile_type="straight_10",
        vel=0.1,
        wrenches=[0.0, 0.0, 0.0,
                  0.0, -12.0, 0.0],
        apply_wrenches_t=3.0,
        sim_time=4.0,
    ),
    "exp_7c": ExperimentConfig(
        exp_name="exp_7c_fxnz",
        profile_type="straight_10",
        vel=0.1,
        wrenches=[0.0, 0.0, 5.0,
                  10.0, 0.0, 0.0],
        apply_wrenches_t=3.0,
        sim_time=4.0,
    ),
    "exp_7d": ExperimentConfig(
        exp_name="exp_7d_nofn",
        profile_type="straight_10",
        vel=0.1,
        wrenches=[0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0],
        apply_wrenches_t=0.0,
        sim_time=4.0
    ),
}


def GetConfigByName(exp_id: str)-> ExperimentConfig:
    """
    Returns the ExperimentConfig for a given ID like '1a', '2c', etc.
    """
    # Combine all experiment dictionaries into one master lookup
    # The 'exp_' prefix is added to match your key format
    all_configs = {**exp_1, **exp_2, **exp_3, **exp_4, **exp_5, **exp_6, **exp_7}
    
    # Format the input string to match the dictionary keys (e.g., "1a" -> "exp_1a")
    search_key = f"exp_{exp_id.lower()}"
    
    # Retrieve the config
    config = all_configs.get(search_key)
    
    if config is None:
        print(f"Warning: Configuration for '{exp_id}' not found.")
        return None
        
    return config
