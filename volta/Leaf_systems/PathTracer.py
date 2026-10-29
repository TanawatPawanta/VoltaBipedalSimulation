import numpy as np
from pydrake.systems.framework import LeafSystem


class PathTracer(LeafSystem):
    def __init__(self, plant, meshcat, body_name="torso"):
        super().__init__()
        self.plant = plant
        self.meshcat = meshcat
        self.body = plant.GetBodyByName(body_name)
        self.path = []
        self.DeclarePeriodicPublish(period_sec=0.01, offset_sec=0.0)
        self.DeclareVectorInputPort("state", plant.num_multibody_states())

    def DoPublish(self, context, events):
        plant_context = self.plant.GetMyContextFromRoot(context)
        x = self.get_input_port(0).Eval(context)
        self.plant.SetPositionsAndVelocities(plant_context, x)
        p_WB = self.plant.EvalBodyPoseInWorld(plant_context, self.body).translation()
        self.path.append(p_WB.copy())

        if len(self.path) > 1:
            points = np.array(self.path).T
            lines = np.array([[i, i + 1] for i in range(len(self.path) - 1)]).T
            self.meshcat.SetLineSegments("robot_path", points, lines, line_width=0.005, rgba=[1, 0, 0, 1])
