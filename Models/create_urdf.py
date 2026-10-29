from xacrodoc import XacroDoc
import os

dir = "Models/" 
local_file = "Volta_rev4/urdf/Volta_L_fixed.urdf.xacro"    # local xacro file
save_to_file = "Volta_rev4_L_fixed.urdf"

doc = XacroDoc.from_file(dir+local_file)
doc.to_urdf_file(dir+save_to_file)

# Create a floor top SDFormat model.
# floor_top_sdf_file = os.path.join(dir, "floor_top.sdf")
# floor_top_sdf = """<?xml version="1.0"?>
# <sdf version="1.7">
#   <model name="floor_top">
#     <link name="floor_top_link">
#       <visual name="visual">
#         <pose>0 0 0.445 0 0 0</pose>
#         <geometry>
#           <box>
#             <size>5.55 5.1 0.05</size>
#           </box>
#         </geometry>
#         <material>
#          <diffuse>0.9 0.8 0.7 1.0</diffuse>
#         </material>
#       </visual>
#       <collision name="collision">
#         <pose>0 0 0.445  0 0 0</pose>
#         <geometry>
#           <box>
#             <size>5.55 5.1 0.05</size>
#           </box>
#         </geometry>
#       </collision>
#     </link>
#     <frame name="floor_top_center">
#       <pose relative_to="floor_top_link">0 0 0.47 0 0 0</pose>
#     </frame>
#   </model>
# </sdf>

# """

# with open(floor_top_sdf_file, "w") as f:
#     f.write(floor_top_sdf)