# from pydrake.systems.framework import LeafSystem
# from pydrake.common.value import Value
# from pydrake.multibody.math import SpatialForce
# from pydrake.multibody.plant import ExternallyAppliedSpatialForce
# import numpy as np

# class DisturbanceGenerator(LeafSystem):
#     def __init__(self, plant, body_name, force_xyz, tau, start_time, duration):
#         LeafSystem.__init__(self)
#         self.plant = plant
#         self.body_index = plant.GetBodyByName(body_name).index()
        
#         # Define the force disturbance
#         self.force_xyz = np.array(force_xyz)
#         self.tau = np.array(tau)
#         self.start_time = start_time
#         self.duration = duration
        
#         # The plant's port expects a 'Value' containing a list of ExternallyAppliedSpatialForce
#         self.DeclareAbstractOutputPort(
#             "spatial_forces",
#             lambda: Value([ExternallyAppliedSpatialForce()]),
#             self.CalcOutput)

#     def CalcOutput(self, context, output):
#         t = context.get_time()
#         forces = []
        
#         # Apply force only during the specified window
#         if self.start_time <= t <= (self.start_time + self.duration):
#             ext_force = ExternallyAppliedSpatialForce()
#             ext_force.body_index = self.body_index
#             # Apply force at the body's origin (p_BoBq = [0,0,0])
#             ext_force.p_BoBq_B = np.zeros(3) 
#             # SpatialForce consists of [Torque; Force]
#             ext_force.F_Bq_W = SpatialForce(tau=self.tau, f=self.force_xyz)
#             forces.append(ext_force)
            
#         output.set_value(forces)

import numpy as np
from pydrake.common.value import Value
from pydrake.geometry import Rgba
from pydrake.math import RigidTransform
from pydrake.multibody.math import SpatialForce
from pydrake.multibody.plant import ExternallyAppliedSpatialForce
from pydrake.systems.framework import LeafSystem


class DisturbanceGenerator(LeafSystem):
    """Applies a constant wrench (tau, force) to one body during [start_time, start_time + duration].

    Input 0 body poses (only for the optional Meshcat arrow), output 0 externally applied spatial forces.
    """
    def __init__(self, plant, body_name, force_xyz, tau, start_time, duration, meshcat=None):
        LeafSystem.__init__(self)
        self.plant = plant
        self.meshcat = meshcat
        self.body_index = plant.GetBodyByName(body_name).index()
        
        self.force_xyz = np.array(force_xyz)
        self.tau = np.array(tau)
        self.start_time = start_time
        self.duration = duration
        
        # We need the body poses to know where to draw the arrow in World frame
        self.pose_input_port = self.DeclareAbstractInputPort(
            "body_poses", Value([RigidTransform()] * plant.num_bodies()))

        self.DeclareAbstractOutputPort(
            "spatial_forces",
            lambda: Value([ExternallyAppliedSpatialForce()]),
            self.CalcOutput)

    def CalcOutput(self, context, output):
        t = context.get_time()
        forces = []
        is_active = self.start_time <= t <= (self.start_time + self.duration)
        
        if is_active:
            ext_force = ExternallyAppliedSpatialForce()
            ext_force.body_index = self.body_index
            ext_force.p_BoBq_B = np.zeros(3) 
            ext_force.F_Bq_W = SpatialForce(tau=self.tau, f=self.force_xyz)
            forces.append(ext_force)

        output.set_value(forces)

        # --- VISUALIZATION LOGIC ---
        if self.meshcat:
            path = "arrow"
            if is_active:
                # 1. Get current pose of the body (X_WB)
                body_poses = self.pose_input_port.Eval(context)
                X_WB = body_poses[int(self.body_index)]
                
                # 2. Calculate start point (p_Wq) and end point in World frame
                # Application point in world: p_Wq = X_WB * p_BoBq_B
                p_Wq = X_WB.multiply(np.zeros(3)) 
                
                # Scale the arrow length (e.g., 10N = 0.2m)
                scale = 0.01
                p_W_end = p_Wq + (self.force_xyz * scale)
                
                # 3. Draw a simple line segment as an arrow
                # Ensure vertices are float64 and Fortran-contiguous (f_contiguous)
                vertices = np.array([p_Wq, p_W_end], dtype=np.float64).T
                vertices = np.asfortranarray(vertices) 
                
                # Convert the list [1, 1, 0, 1] to a Drake Rgba object
                # yellow = Rgba(1.0, 1.0, 0.0, 1.0)
                
                self.meshcat.SetLine(
                    path=path, 
                    vertices=vertices, 
                    rgba=Rgba(1.0, 0.0, 0.0, 1.0), 
                    line_width=4.0
                )
            else:
                self.meshcat.Delete(path)