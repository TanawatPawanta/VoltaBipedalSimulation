"""SDF/URDF strings for the experiment scenes: flat floor, tiled terrain and the Volta robot."""


def floor_sdf(stiffness: float, dissipation: float) -> str:
    """Flat 10.55 x 5.1 m floor with point-contact stiffness and Hunt-Crossley dissipation."""
    return f"""<?xml version="1.0"?>
                <sdf version="1.7">
                <model name="floor_top">
                    <link name="floor_top_link">
                    <visual name="visual">
                        <pose>0 0 -0.025 0 0 0</pose>
                        <!-- <pose>0 0 0.0 0 0 0</pose> -->
                        <geometry>
                        <box>
                            <size>10.55 5.1 0.05</size>
                        </box>
                        </geometry>
                        <material>
                        <diffuse>0.9 0.8 0.7 0.5</diffuse>
                        </material>
                    </visual>
                    <collision name="collision">
                        <pose>0 0 -0.025 0 0 0</pose>
                        <geometry>
                        <box>
                            <size>10.55 5.1 0.05</size>
                        </box>
                        </geometry>
                        <drake:proximity_properties>
                        <drake:mu_dynamic>0.8</drake:mu_dynamic>
                        <drake:mu_static>1.0</drake:mu_static>
                        <drake:point_contact_stiffness>{stiffness}</drake:point_contact_stiffness>
                        <drake:hunt_crossley_dissipation>{dissipation}</drake:hunt_crossley_dissipation>
                        </drake:proximity_properties>
                    </collision>
                    </link>
                    <frame name="floor_top_center">
                    <pose relative_to="floor_top_link">0 0 0.0 0 0 0</pose>
                    </frame>
                </model>
                </sdf>
                """


def multi_terrain_sdf(model_name="test_terrain", tile_L=0.5, tile_W=0.4, tile_thickness=0.1, properties=None):
    """
    Generates an SDF string for a multi-property and multi-height grid terrain.
    
    Args:
        model_name: Name of the SDF model.
        tile_L: Length of one tile (X direction)
        tile_W: Width of one tile (Y direction)
        tile_thickness: Thickness of the box geometry (Z direction)
        properties: Dict mapping color names to Drake physics parameters

    Returns:
        (sdf string, list of [x, y, surface_z] for each tile)
    """
    if properties is None:
        # Default properties if none provided
        # properties = {
        #     'orange': {'k': 5e6, 'd': 50, 'mu': 1.0, 'color': [0.9, 0.6, 0.4, 0.5]},
        #     'green':  {'k': 4e3, 'd': 30, 'mu': 1.0, 'color': [0.4, 0.8, 0.4, 0.5]},
        #     'yellow': {'k': 5e3, 'd': 50, 'mu': 1.0, 'color': [0.9, 0.9, 0.4, 0.5]},
        #     'blue':   {'k': 2e4, 'd': 10, 'mu': 1.0, 'color': [0.4, 0.6, 0.9, 0.5]}
        # }
        properties = {
            'orange': {'k': 5e6, 'd': 50, 'mu': 1.0, 'color': [0.9, 0.6, 0.4, 1]},
            'green':  {'k': 5e6, 'd': 50, 'mu': 1.0, 'color': [0.4, 0.8, 0.4, 1]},
            'yellow': {'k': 5e6, 'd': 50, 'mu': 1.0, 'color': [0.9, 0.9, 0.4, 1]},
            'blue':   {'k': 5e6, 'd': 50, 'mu': 1.0, 'color': [0.4, 0.6, 0.9, 1]}
        }
    # Grid mapping from the image (Bottom to Top)
    # Now defined as a list of tuples: (color, surface_z_height)
    # A height of 0.0 means the top of the tile is at exactly Z=0.
    grid = [
        [('orange', 0.0)],                                   # Row 0: Flat start
        [('yellow', 0.005), ('green', -0.005)],                # Row 1: Left slightly high, right slightly low
        [('blue', 0.0),   ('yellow', 0.005)],                # Row 2: A step up (5cm)
        [('green', 0.002),   ('orange', 0.008)],                # Row 3: Uneven drop and rise
        [('orange', 0.0)]                                    # Row 4: Back to flat
    ]

    sdf = f'<?xml version="1.0" ?>\n<sdf version="1.7">\n  <model name="{model_name}">\n'
    joints_sdf = ""  # To accumulate joint definitions for welding tiles together
    tile_count = 0
    
    surface_position = []

    for row_idx, row in enumerate(grid):
        for col_idx, (color, height) in enumerate(row):
            # Calculate Dimensions
            width = tile_W * 2 if len(row) == 1 else tile_W
            length = tile_L
            
            # Calculate Poses
            x_pos = (row_idx * tile_L) 
            y_pos = 0.0 if len(row) == 1 else ((tile_W / 2.0) if col_idx == 0 else -(tile_W / 2.0))
            
            # Since the reference frame for a box is its center of mass, 
            # to make the top surface sit exactly at `height`, we shift the Z pose down by half the thickness.
            z_pos = height - (tile_thickness / 2.0)
            
            surface_position.append([x_pos, y_pos, height]) # Added height to surface tracking
            
            p = properties[color]
            link_name = f"floor_{tile_count}"
            
            sdf += f'''
            <link name="{link_name}">
            <visual name="visual">
                <pose>{x_pos} {y_pos} {z_pos} 0 0 0</pose>
                <geometry><box><size>{length} {width} {tile_thickness}</size></box></geometry>
                <material><diffuse>{" ".join(map(str, p['color']))}</diffuse></material>
            </visual>
            <collision name="collision">
                <pose>{x_pos} {y_pos} {z_pos} 0 0 0</pose>
                <geometry><box><size>{length} {width} {tile_thickness}</size></box></geometry>
                <drake:proximity_properties>
                <drake:mu_static>{p['mu']}</drake:mu_static>
                <drake:mu_dynamic>{p['mu']*0.8}</drake:mu_dynamic>
                <drake:point_contact_stiffness>{p['k']}</drake:point_contact_stiffness>
                <drake:hunt_crossley_dissipation>{p['d']}</drake:hunt_crossley_dissipation>
                </drake:proximity_properties>
            </collision>
            </link>
            <frame name="{link_name}_frame">
            <pose relative_to="{link_name}">0 0 {tile_thickness/2.0} 0 0 0</pose>
            </frame>
            '''
            
            # Weld all subsequent tiles to the first tile
            if tile_count > 0:
                joints_sdf += f'''
                <joint name="{link_name}_weld" type="fixed">
                    <parent>floor_0</parent>
                    <child>{link_name}</child>
                </joint>'''
            tile_count += 1
            
    sdf += joints_sdf + '\n  </model>\n</sdf>'
    return sdf, surface_position


def volta_urdf(mesh_dir: str, joint_damping: float, joint_torque_limit: float) -> str:
    """Volta rev5 URDF with per-experiment joint damping and torque limit."""
    return f"""<?xml version="1.0" ?>
                    <robot name="Volta">
                    <link name="world"/>
                    <!-- Torso -->
                    <link name="torso">
                        <inertial>
                        <origin xyz="0.0001557 -2.0783e-12 0.024021"/>
                        <mass value="2.5"/>
                        <inertia ixx="0.0028215" ixy="-7.1828e-09" ixz="-5.7023e-06" iyy="0.0015625" iyz="-6.0301e-09" izz="0.0025688"/>
                        </inertial>
                        <visual>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/torso.obj"/>
                        </geometry>
                        <material name="">
                            <color rgba="0.25098 0.25098 0.25098 1"/>
                        </material>
                        </visual>
                        <collision name="torso">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/torso_collision.obj"/>
                        </geometry>
                        </collision>
                    </link>
                    <!-- Left leg -->
                    <joint name="JL_hip_y" type="continuous">
                        <parent link="L_hip"/>
                        <child link="L_thigh"/>
                        <origin rpy="0 1.5708 0" xyz="0.124 0 -0"/>
                        <dynamics damping="{joint_damping}"  />
                        <axis xyz="0 0 1"/>
                        <limit effort="{joint_torque_limit}" lower="-3.14152" upper="3.14152" velocity="0"/>
                    </joint>
                    <transmission name="trans_JL_hip_y" type="SimpleTransmission">
                        <actuator name="JL_hip_y_motor">
                        <mechanicalReduction>1.0</mechanicalReduction>
                        </actuator>
                        <joint name="JL_hip_y"/>
                    </transmission>
                    <link name="L_hip_connector">
                        <inertial>
                        <origin xyz="0.00010593 0.0079546 0.052571"/>
                        <mass value="0.36"/>
                        <inertia ixx="0.00011402" ixy="4.9119e-09" ixz="-6.2942e-07" iyy="0.0001669" iyz="-1.2947e-05" izz="7.8252e-05"/>
                        </inertial>
                        <visual>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/L_hip_connector.obj"/>
                        </geometry>
                        <material name="">
                            <color rgba="0.9098 0.44314 0.031373 1"/>
                        </material>
                        </visual>
                    </link>
                    <joint name="JL_hip_r" type="continuous">
                        <parent link="L_hip_connector"/>
                        <child link="L_hip"/>
                        <origin rpy="1.5708 0 0" xyz="0 0 0.058"/>
                        <dynamics damping="{joint_damping}"  />
                        <axis xyz="0 0 1"/>
                        <limit effort="{joint_torque_limit}" lower="-3.14152" upper="3.14152" velocity="0"/>
                    </joint>
                    <transmission name="trans_JL_hip_r" type="SimpleTransmission">
                        <actuator name="JL_hip_r_motor">
                        <mechanicalReduction>1.0</mechanicalReduction>
                        </actuator>
                        <joint name="JL_hip_r"/>
                    </transmission>
                    <link name="L_hip">
                        <inertial>
                        <origin xyz="0.084531 1.0364e-05 0.0052271"/>
                        <mass value="0.365"/>
                        <inertia ixx="0.00013738" ixy="-8.2548e-09" ixz="1.9227e-05" iyy="0.00017953" iyz="-5.5353e-09" izz="0.00018391"/>
                        </inertial>
                        <visual>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/L_hip.obj"/>
                        </geometry>
                        <material name="">
                            <color rgba="0.25098 0.25098 0.25098 1"/>
                        </material>
                        </visual>
                    </link>
                    <joint name="JL_hip_p" type="continuous">
                        <parent link="torso"/>
                        <child link="L_hip_connector"/>
                        <origin rpy="-1.5708 1.5708 0" xyz="0 0.067 0"/>
                        <dynamics damping="{joint_damping}"  />
                        <axis xyz="0 0 1"/>
                        <limit effort="{joint_torque_limit}" lower="-3.14152" upper="3.14152" velocity="0"/>
                    </joint>
                    <transmission name="trans_JL_hip_p" type="SimpleTransmission">
                        <actuator name="JL_hip_p_motor">
                        <mechanicalReduction>1.0</mechanicalReduction>
                        </actuator>
                        <joint name="JL_hip_p"/>
                    </transmission>
                    <link name="L_thigh">
                        <inertial>
                        <origin xyz="2.613e-05 0.0057128 0.10482"/>
                        <mass value="0.434"/>
                        <inertia ixx="0.0009106" ixy="-1.0875e-07" ixz="-4.769e-06" iyy="0.00099215" iyz="-5.8959e-05" izz="0.00013331"/>
                        </inertial>
                        <visual>
                        <origin rpy="0 0 -1.5708" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/L_thigh.obj"/>
                        </geometry>
                        <material name="">
                            <color rgba="0.9098 0.44314 0.031373 1"/>
                        </material>
                        </visual>
                        <collision name="L_thigh">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/L_thigh_collision.obj"/>
                        </geometry>
                        </collision>
                    </link>
                    <joint name="JL_knee" type="continuous">
                        <parent link="L_thigh"/>
                        <child link="L_leg"/>
                        <origin rpy="1.5708 -1.5708 0" xyz="0 0 0.17"/>
                        <dynamics damping="{joint_damping}"  />
                        <axis xyz="0 0 1"/>
                        <limit effort="{joint_torque_limit}" lower="-3.14152" upper="3.14152" velocity="0"/>
                    </joint>
                    <transmission name="trans_JL_knee" type="SimpleTransmission">
                        <actuator name="JL_knee_motor">
                        <mechanicalReduction>1.0</mechanicalReduction>
                        </actuator>
                        <joint name="JL_knee"/>
                    </transmission>
                    <link name="L_leg">
                        <inertial>
                        <origin xyz="0.13604 -3.7092E-05 0.02863"/>
                        <mass        
                            value="0.462" />
                        <inertia
                            ixx="0.00028553"
                            ixy="1.2607E-06"
                            ixz="-0.000125"
                            iyy="0.0016659"
                            iyz="-2.8824E-07"
                            izz="0.0017159" />
                        </inertial>
                        <visual>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/L_leg.obj"/>
                        </geometry>
                        <material name="">
                            <color rgba="0.25098 0.25098 0.25098 1"/>
                        </material>
                        </visual>
                    </link>
                    <joint name="JL_ankle_p" type="continuous">
                        <parent link="L_leg"/>
                        <child link="L_ankle"/>
                        <origin rpy="-3.1416 0 -1.5708" xyz="0.2 0 -0.0"/>
                        <dynamics damping="{joint_damping}"  />
                        <axis xyz="0 0 1"/>
                        <limit effort="{joint_torque_limit}" lower="-3.14152" upper="3.14152" velocity="0"/>
                    </joint>
                    <transmission name="trans_JL_ankle_p" type="SimpleTransmission">
                        <actuator name="JL_ankle_p_motor">
                        <mechanicalReduction>1.0</mechanicalReduction>
                        </actuator>
                        <joint name="JL_ankle_p"/>
                    </transmission>
                    <link name="L_ankle">
                        <inertial>
                        <origin xyz="0.037457 8.4957e-06 -0.0014016"/>
                        <mass value="0.387"/>
                        <inertia ixx="0.00016902" ixy="-4.7177e-09" ixz="-1.2522e-05" iyy="0.00037861" iyz="-2.4542e-09" izz="0.00037947"/>
                        </inertial>
                        <visual>
                        <origin rpy="0 0 0" xyz="0 0 0.0061"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/L_ankle.obj"/>
                        </geometry>
                        <material name="">
                            <color rgba="0.9098 0.44314 0.031373 1"/>
                        </material>
                        </visual>
                    </link>
                    <joint name="JL_ankle_r" type="continuous">
                        <parent link="L_ankle"/>
                        <child link="L_foot"/>
                        <origin rpy="-1.5708 0 -1.5708" xyz="0.094 0 -0.0"/>
                        <dynamics damping="{joint_damping}"  />
                        <axis xyz="0 0 1"/>
                        <limit effort="{joint_torque_limit}" lower="-3.14152" upper="3.14152" velocity="0"/>
                    </joint>
                    <transmission name="trans_JL_ankle_r" type="SimpleTransmission">
                        <actuator name="JL_ankle_r_motor">
                        <mechanicalReduction>1.0</mechanicalReduction>
                        </actuator>
                        <joint name="JL_ankle_r"/>
                    </transmission>
                    <link name="L_foot">
                        <inertial>
                        <origin xyz="0.07325 -3.4565E-08 -0.086848"/>
                        <mass
                            value="0.295" />
                        <inertia
                            ixx="0.0015362"
                            ixy="1.4009E-09"
                            ixz="0.00015597"
                            iyy="0.0014868"
                            iyz="-2.5295E-09"
                            izz="0.00039154" />
                        </inertial>
                        <visual>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/L_foot.obj"/>
                        </geometry>
                        <material name="">
                            <color rgba="0.9098 0.44314 0.031373 1"/>
                        </material>
                        </visual>
                        <collision name="L_foot">
                        <origin rpy="0 0 0" xyz="-0.003 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/L_foot_collision.obj"/>
                        </geometry>
                        </collision>
                    </link>
                    <link name="L_foot_contact"/>
                    <joint name="L_foot_contact_offset" type="fixed">
                        <parent link="L_foot"/>
                        <child link="L_foot_contact"/>
                        <origin rpy="0 1.5705 3.1416" xyz="0.112 0.00 -0.094"/>
                    </joint>
                    <!-- L foot contact plates -->
                    <link name="L_foot_contact_FL">
                        <collision name="L_foot_contact_FL">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/Foot_contact_plate.obj"/>
                        </geometry>
                        </collision>
                    </link>

                    <link name="L_foot_contact_FR">
                        <collision name="L_foot_contact_FR">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/Foot_contact_plate.obj"/>
                        </geometry>
                        </collision>
                    </link>

                    <link name="L_foot_contact_RL">
                        <collision name="L_foot_contact_RL">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/Foot_contact_plate.obj"/>
                        </geometry>
                        </collision>
                    </link>

                    <link name="L_foot_contact_RR">
                        <collision name="L_foot_contact_RR">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/Foot_contact_plate.obj"/>
                        </geometry>
                        </collision>
                    </link>

                    <joint name="L_foot_contact_FL_offset" type="fixed">
                        <parent link="L_foot_contact"/>
                        <child link="L_foot_contact_FL"/>
                        <origin rpy="0 0 0" xyz="0.08458 0.03386 0.0"/>
                    </joint>

                    <joint name="L_foot_contact_FR_offset" type="fixed">
                        <parent link="L_foot_contact"/>
                        <child link="L_foot_contact_FR"/>
                        <origin rpy="0 0 0" xyz="0.08458 -0.03386 0.0"/>
                    </joint>

                    <joint name="L_foot_contact_RL_offset" type="fixed">
                        <parent link="L_foot_contact"/>
                        <child link="L_foot_contact_RL"/>
                        <origin rpy="0 0 0" xyz="-0.09258 0.03386 0.0"/>
                    </joint>

                    <joint name="L_foot_contact_RR_offset" type="fixed">
                        <parent link="L_foot_contact"/>
                        <child link="L_foot_contact_RR"/>
                        <origin rpy="0 0 0" xyz="-0.09258 -0.03386 0.0"/>
                    </joint>

                    <!-- Right leg -->
                    <joint name="JR_hip_y" type="continuous">
                        <parent link="R_hip"/>
                        <child link="R_thigh"/>
                        <origin rpy="0 1.5708 0" xyz="0.124 0 -0"/>
                        <dynamics damping="{joint_damping}"  />
                        <axis xyz="0 0 1"/>
                        <limit effort="{joint_torque_limit}" lower="-3.14152" upper="3.14152" velocity="0"/>
                    </joint>
                    <transmission name="trans_JR_hip_y" type="SimpleTransmission">
                        <actuator name="JR_hip_y_motor">
                        <mechanicalReduction>1.0</mechanicalReduction>
                        </actuator>
                        <joint name="JR_hip_y"/>
                    </transmission>
                    <link name="R_hip_connector">
                        <inertial>
                        <origin
                            xyz="0.00010202 -0.0073451 0.052593"
                            rpy="0 0 0" />
                        <mass
                            value="0.372" />
                        <inertia
                            ixx="0.00011402"
                            ixy="-3.2815E-09"
                            ixz="-6.3402E-07"
                            iyy="0.0001669"
                            iyz="1.2938E-05"
                            izz="7.825E-05" />
                        </inertial>
                        <visual>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/R_hip_connector.obj"/>
                        </geometry>
                        <material name="">
                            <color rgba="0.9098 0.44314 0.031373 1"/>
                        </material>
                        </visual>
                    </link>
                    <joint name="JR_hip_r" type="continuous">
                        <parent link="R_hip_connector"/>
                        <child link="R_hip"/>
                        <origin rpy="-1.5708 0 0" xyz="0 0 0.058"/>
                        <dynamics damping="{joint_damping}"  />
                        <axis xyz="0 0 1"/>
                        <limit effort="{joint_torque_limit}" lower="-3.14152" upper="3.14152" velocity="0"/>
                    </joint>
                    <transmission name="trans_JR_hip_r" type="SimpleTransmission">
                        <actuator name="JR_hip_r_motor">
                        <mechanicalReduction>1.0</mechanicalReduction>
                        </actuator>
                        <joint name="JR_hip_r"/>
                    </transmission>
                    <link name="R_hip">
                        <inertial>
                        <origin xyz="0.084531 1.0364e-05 0.0052271"/>
                        <mass value="0.365"/>
                        <inertia ixx="0.00013738" ixy="-8.2548e-09" ixz="1.9227e-05" iyy="0.00017953" iyz="-5.5353e-09" izz="0.00018391"/>
                        </inertial>
                        <visual>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/R_hip.obj"/>
                        </geometry>
                        <material name="">
                            <!-- <color rgba="0.9098 0.44314 0.031373 1"/> -->
                            <color rgba="0.25098 0.25098 0.25098 1"/>
                        </material>
                        </visual>
                    </link>
                    <joint name="JR_hip_p" type="continuous">
                        <parent link="torso"/>
                        <child link="R_hip_connector"/>
                        <origin rpy="1.5708 1.5708 0" xyz="0 -0.067001 0"/>
                        <dynamics damping="{joint_damping}"  />
                        <axis xyz="0 0 1"/>
                        <limit effort="{joint_torque_limit}" lower="-3.14152" upper="3.14152" velocity="0"/>
                    </joint>
                    <transmission name="trans_JR_hip_p" type="SimpleTransmission">
                        <actuator name="JR_hip_p_motor">
                        <mechanicalReduction>1.0</mechanicalReduction>
                        </actuator>
                        <joint name="JR_hip_p"/>
                    </transmission>
                    <link name="R_thigh">
                        <inertial>
                        <origin xyz="2.6122e-05 -0.0057129 0.10483"/>
                        <mass value="0.434"/>
                        <inertia ixx="0.0009106" ixy="1.0872e-07" ixz="-4.7736e-06" iyy="0.00099215" iyz="5.895e-05" izz="0.00013331"/>
                        </inertial>
                        <visual>
                        <origin rpy="0 0 -1.5708" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/R_thigh.obj"/>
                        </geometry>
                        <material name="">
                            <color rgba="0.9098 0.44314 0.031373 1"/>
                        </material>
                        </visual>
                        <collision name="R_thigh">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/R_thigh_collision.obj"/>
                        </geometry>
                        </collision>
                    </link>
                    <joint name="JR_knee" type="continuous">
                        <parent link="R_thigh"/>
                        <child link="R_leg"/>
                        <origin rpy="-1.5708 -1.5708 0" xyz="0 0 0.17"/>
                        <dynamics damping="{joint_damping}"  />
                        <axis xyz="0 0 1"/>
                        <limit effort="{joint_torque_limit}" lower="-3.14152" upper="3.14152" velocity="0"/>
                    </joint>
                    <transmission name="trans_JR_knee" type="SimpleTransmission">
                        <actuator name="JR_knee_motor">
                        <mechanicalReduction>1.0</mechanicalReduction>
                        </actuator>
                        <joint name="JR_knee"/>
                    </transmission>
                    <link name="R_leg">
                        <inertial>
                        <origin
                            xyz="0.13604 -3.7092E-05 0.02863"
                            rpy="0 0 0" />
                        <mass
                            value="0.462" />
                        <inertia
                            ixx="0.00028553"
                            ixy="1.2607E-06"
                            ixz="-0.000125"
                            iyy="0.0016659"
                            iyz="-2.8824E-07"
                            izz="0.0017159" />
                        </inertial>
                        <visual>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/R_leg.obj"/>
                        </geometry>
                        <material name="">
                            <color rgba="0.25098 0.25098 0.25098 1"/>
                        </material>
                        </visual>
                    </link>
                    <joint name="JR_ankle_p" type="continuous">
                        <parent link="R_leg"/>
                        <child link="R_ankle"/>
                        <origin rpy="-3.1416 0 1.5708" xyz="0.2 0 -0.0"/>
                        <dynamics damping="{joint_damping}"  />
                        <axis xyz="0 0 1"/>
                        <limit effort="{joint_torque_limit}" lower="-3.14152" upper="3.14152" velocity="0"/>
                    </joint>
                    <transmission name="trans_JR_ankle_p" type="SimpleTransmission">
                        <actuator name="JR_ankle_p_motor">
                        <mechanicalReduction>1.0</mechanicalReduction>
                        </actuator>
                        <joint name="JR_ankle_p"/>
                    </transmission>
                    <link name="R_ankle">
                        <inertial>
                        <origin xyz="0.037457 8.4957e-06 -0.0014016"/>
                        <mass value="0.387"/>
                        <inertia ixx="0.00016902" ixy="-4.7177e-09" ixz="-1.2522e-05" iyy="0.00037861" iyz="-2.4542e-09" izz="0.00037947"/>
                        </inertial>
                        <visual>
                        <origin rpy="0 0 0" xyz="0 0 0.0061"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/R_ankle.obj"/>
                        </geometry>
                        <material name="">
                            <color rgba="0.9098 0.44314 0.031373 1"/>
                        </material>
                        </visual>
                    </link>
                    <joint name="JR_ankle_r" type="continuous">
                        <parent link="R_ankle"/>
                        <child link="R_foot"/>
                        <origin rpy="1.5708 0 1.5708" xyz="0.094 0 -0.0"/>
                        <dynamics damping="{joint_damping}"  />
                        <axis xyz="0 0 1"/>
                        <limit effort="{joint_torque_limit}" lower="-3.14152" upper="3.14152" velocity="0"/>
                    </joint>
                    <transmission name="trans_JR_ankle_r" type="SimpleTransmission">
                        <actuator name="JR_ankle_r_motor">
                        <mechanicalReduction>1.0</mechanicalReduction>
                        </actuator>
                        <joint name="JR_ankle_r"/>
                    </transmission>
                    <link name="R_foot">
                        <inertial>
                        <origin
                            xyz="0.07325 -3.4565E-08 -0.086848"
                            rpy="0 0 0" />
                        <mass
                            value="0.295" />
                        <inertia
                            ixx="0.0015362"
                            ixy="1.4009E-09"
                            ixz="0.00015597"
                            iyy="0.0014868"
                            iyz="-2.5295E-09"
                            izz="0.00039154" />
                        </inertial>
                        <visual>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/R_foot.obj"/>
                        </geometry>
                        <material name="">
                            <color rgba="0.9098 0.44314 0.031373 1"/>
                        </material>
                        </visual>
                        <collision name="R_foot">
                        <origin rpy="0 0 0" xyz="-0.003 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/R_foot_collision.obj"/>
                        </geometry>
                        </collision>
                    </link>
                    <link name="R_foot_contact"/>
                    <joint name="R_foot_contact_offset" type="fixed">
                        <parent link="R_foot"/>
                        <child link="R_foot_contact"/>
                        <origin rpy="0 1.5705 3.1416" xyz="0.112 0.00 -0.094"/>
                    </joint>
                    <!-- R foot contact plates -->
                    <link name="R_foot_contact_FL">
                        <collision name="R_foot_contact_FL">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/Foot_contact_plate.obj"/>
                        </geometry>
                        </collision>
                    </link>

                    <link name="R_foot_contact_FR">
                        <collision name="R_foot_contact_FR">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/Foot_contact_plate.obj"/>
                        </geometry>
                        </collision>
                    </link>

                    <link name="R_foot_contact_RL">
                        <collision name="R_foot_contact_RL">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/Foot_contact_plate.obj"/>
                        </geometry>
                        </collision>
                    </link>

                    <link name="R_foot_contact_RR">
                        <collision name="R_foot_contact_RR">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="{mesh_dir}/Foot_contact_plate.obj"/>
                        </geometry>
                        </collision>
                    </link>

                    <joint name="R_foot_contact_FL_offset" type="fixed">
                        <parent link="R_foot_contact"/>
                        <child link="R_foot_contact_FL"/>
                        <origin rpy="0 0 0" xyz="0.08458 0.03386 0.0"/>
                    </joint>

                    <joint name="R_foot_contact_FR_offset" type="fixed">
                        <parent link="R_foot_contact"/>
                        <child link="R_foot_contact_FR"/>
                        <origin rpy="0 0 0" xyz="0.08458 -0.03386 0.0"/>
                    </joint>

                    <joint name="R_foot_contact_RL_offset" type="fixed">
                        <parent link="R_foot_contact"/>
                        <child link="R_foot_contact_RL"/>
                        <origin rpy="0 0 0" xyz="-0.09258 0.03386 0.0"/>
                    </joint>

                    <joint name="R_foot_contact_RR_offset" type="fixed">
                        <parent link="R_foot_contact"/>
                        <child link="R_foot_contact_RR"/>
                        <origin rpy="0 0 0" xyz="-0.09258 -0.03386 0.0"/>
                    </joint>
                    </robot>
                    """
