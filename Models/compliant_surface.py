def CompliantBox(hydroelastic_modulus='1e7',size="0.5 0.5 0.1"):
    return f"""<?xml version="1.0"?>
    <sdf version="1.7">
      <model name="CompliantBox">
        <pose>0 0 0 0 0 0</pose>
        <link name="compliant_box">
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.016666</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>
              <iyy>0.014166</iyy> <iyz>0.0</iyz>
              <izz>0.004166</izz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry>
              <box>
                <size>0.10 0.20 0.40</size>
              </box>
            </geometry>
            <drake:proximity_properties>
              <drake:compliant_hydroelastic/>
              <drake:hydroelastic_modulus>{hydroelastic_modulus}</drake:hydroelastic_modulus>
              <drake:mu_dynamic>0.5</drake:mu_dynamic>
              <drake:hunt_crossley_dissipation>1.25</drake:hunt_crossley_dissipation>
            </drake:proximity_properties>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>0.10 0.20 0.40</size>
              </box>
            </geometry>
            <material>
              <diffuse>1.0 1.0 1.0 0.5</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """