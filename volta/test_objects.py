"""SDF builders for extra test objects (rigid / compliant boxes, soft mattress)."""
def RigidHydroBox(mass=60,name="rigid_hydro_box"):
    return f"""<?xml version="1.0"?>
    <sdf version="1.7">
      <model name="{name}">
        <pose>0 0 0 0 0 0</pose>
        <link name="{name}_link">
          <inertial>
            <mass>{mass}</mass>
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
              <drake:rigid_hydroelastic/>
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
              <diffuse>1.0 0.0 0.0 1.0</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """

def CompliantHydroBox(hydroelastic_modulus="1e7",mass="60",name="compliant_hydro_box"):
    return f"""<?xml version="1.0"?>
    <sdf version="1.7">
      <model name="{name}">
        <pose>0 0 0 0 0 0</pose>
        <link name="{name}_link">
          <inertial>
            <mass>{mass}</mass>
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
              <diffuse>0.0 1.0 0.0 1.0</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """

def NormalBox(mass="60",name="normal_box"):
    return f"""<?xml version="1.0"?>
    <sdf version="1.7">
      <model name="{name}">
        <pose>0 0 0 0 0 0</pose>
        <link name="{name}_link">
          <inertial>
            <mass>{mass}</mass>
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
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>0.10 0.20 0.40</size>
              </box>
            </geometry>
            <material>
              <diffuse>0.0 0.0 1.0 1.0</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """
class SoftMattressProperties:
    def __init__(self, modulus="1e7", dissipation="1.25", mu_dynamic="0.5", mu_static="0.5", resolution_hint="0.01"):
        self.modulus = modulus
        self.dissipation = dissipation
        self.mu_dynamic = mu_dynamic
        self.mu_static = mu_static
        self.resolution_hint = resolution_hint
        self.color = "0.2 0.2 0.2 0.8"
        self.thick = "0.2"
def SoftMattress(properties=SoftMattressProperties(),name="soft_mattress"):
    return f"""<?xml version="1.0"?>
    <sdf version="1.7">
    <model name="{name}">
        <link name="{name}_link">
        <inertial>
        <mass>10</mass>
        </inertial>
        <visual name="visual">
            <geometry>
            <box>
                <size>0.4 0.4 {properties.thick}</size>
            </box>
            </geometry>
            <material>
                <diffuse>{properties.color}</diffuse>
            </material>
        </visual>
        <collision name="collision">
            <geometry>
            <box>
                <size>0.4 0.4 {properties.thick}</size>
            </box>
            </geometry>
            <drake:proximity_properties>
                <drake:mu_dynamic>0.8</drake:mu_dynamic>
                <drake:mu_static>1.0</drake:mu_static>
                <drake:point_contact_stiffness>{properties.modulus}</drake:point_contact_stiffness>
                <drake:hunt_crossley_dissipation>{properties.dissipation}</drake:hunt_crossley_dissipation>
            </drake:proximity_properties>
        </collision>
        </link>
    </model>
    </sdf>
    """
# def SoftMattress(properties=SoftMattressProperties(),name="soft_mattress"):
#     return f"""<?xml version="1.0"?>
#     <sdf version="1.7">
#     <model name="{name}">
#         <link name="{name}_link">
#         <inertial>
#         <mass>10</mass>
#         </inertial>
#         <visual name="visual">
#             <geometry>
#             <box>
#                 <size>1 1 {properties.thick}</size>
#             </box>
#             </geometry>
#             <material>
#                 <diffuse>{properties.color}</diffuse>
#             </material>
#         </visual>
#         <collision name="collision">
#             <geometry>
#             <box>
#                 <size>1 1 {properties.thick}</size>
#             </box>
#             </geometry>
#             <drake:proximity_properties>
#                 <drake:compliant_hydroelastic/>
#                 <drake:mesh_resolution_hint>{properties.resolution_hint}</drake:mesh_resolution_hint>
#                 <drake:mu_dynamic>{properties.mu_dynamic}</drake:mu_dynamic>
#                 <drake:mu_static>{properties.mu_static}</drake:mu_static>
#                 <drake:hydroelastic_modulus>{properties.modulus}</drake:hydroelastic_modulus>
#                 <drake:hunt_crossley_dissipation>{properties.dissipation}</drake:hunt_crossley_dissipation>
#             </drake:proximity_properties>
#         </collision>
#         </link>
#     </model>
#     </sdf>
#     """