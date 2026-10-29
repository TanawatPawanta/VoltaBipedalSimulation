import numpy as np
from pydrake.common.value import Value
from pydrake.multibody.plant import ContactResults, MultibodyPlant
from pydrake.multibody.tree import BodyIndex
from pydrake.systems.framework import BasicVector, LeafSystem


class ContactForceSensor(LeafSystem):
    """Foot ground-reaction wrenches and ZMP / CoP from the plant's contact results.

    Inputs:  0 contact results (abstract), 1 robot state (q, v)
    Outputs: 0 GRF wrench per foot [12], 1 ZMP in world [3], 2 CoP in each foot frame [4],
             3 total contact wrenches [12]
    """
    def __init__(self, dt, plant:MultibodyPlant, num_states=37):
        super().__init__()
        self.dt = 0.001
        self.plant = plant
        self.plant_context = self.plant.CreateDefaultContext()
        self.num_states = num_states
        self.L_foot_idx = BodyIndex(7)
        self.R_foot_idx = BodyIndex(13)
        self.ground_contact_forces = {
                                "L_FL": np.zeros(3),
                                "L_FR": np.zeros(3),
                                "L_RL": np.zeros(3),
                                "L_RR": np.zeros(3),
                                "R_FL": np.zeros(3),            
                                "R_FR": np.zeros(3),
                                "R_RL": np.zeros(3),
                                "R_RR": np.zeros(3)
                            }
        self.sensor_coords = [
                                [0.08,    0.02475, 0.0, 1.0],   # Sensor 1 (front-right)
                                [0.08,   -0.02475, 0.0, 1.0],  # Sensor 2 (front-left)
                                [-0.097,  0.02475, 0.0, 1.0], # Sensor 3 (rear-left)
                                [-0.097, -0.02475, 0.0, 1.0],   # Sensor 4 (rear-right)
                            ]
        self.total_force_in_z = 0
        self.total_force_l = np.zeros(3)
        self.total_moment_l = np.zeros(3)
        self.total_force_r = np.zeros(3)
        self.total_moment_r = np.zeros(3)
        self.zmp_L_location = np.array([0.0, 0.125])
        self.zmp_R_location = np.array([0.0, -0.125])
        self.ZMP_in_W = np.zeros(3)
        self.l_foot_frame = self.plant.GetFrameByName("L_foot_contact")
        self.r_foot_frame = self.plant.GetFrameByName("R_foot_contact")
        self._geom_id_to_body_name = {}
        for i in range(self.plant.num_bodies()):
            body = self.plant.get_body(BodyIndex(i))
            if self.plant.geometry_source_is_registered():
                 for geom_id in self.plant.GetCollisionGeometriesForBody(body):
                    self._geom_id_to_body_name[geom_id] = body.name()
        # Declare an output port 
        self.input_port_0 = self.DeclareAbstractInputPort(
            name="Contact results",
            model_value=Value(ContactResults())
        )
        self.input_port_1 = self.DeclareVectorInputPort(
            "Robot states feedback",
            BasicVector(self.num_states)  
        )
        # out 0
        self.DeclareVectorOutputPort(
            "Ground reaction force",
            BasicVector(12),
            self.CalcOutput_GRF
        )
        # out 1
        self.DeclareVectorOutputPort(
            "ZMP location in world frame",
            BasicVector(3),
            self.CalcOutput_ZMP
        )
        # out 2
        self.DeclareVectorOutputPort(
            "CoP location in foot frame",
            BasicVector(4),
            self.CalcOutput_CoP_in_foot_frame
        )
        self.DeclareVectorOutputPort(
            "total contact wrenches",
            BasicVector(12),
            self.CalcOutput_total_contact_wrenches  
        )
        # Declare periodic events
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=self.dt,  
            offset_sec=0.0,  
            update=self.DiscreteUpdate)
        
    def UpdateVariables(self,context):
        self.current_robot_state = self.input_port_1.Eval(context)
        if self.num_states == 37:
            self.generalized_position = self.current_robot_state[0:19]
            self.generalized_velocity = self.current_robot_state[19:37]
        else:
            self.generalized_position = self.current_robot_state[0:12]
            self.generalized_velocity = self.current_robot_state[12:24]
        self.plant.SetPositions(self.plant_context, self.generalized_position) 
        self.plant.SetVelocities(self.plant_context, self.generalized_velocity)

    def DiscreteUpdate(self, context, discrete_state):
        contact_results:ContactResults = self.input_port_0.Eval(context)
        self.UpdateVariables(context)
        force_datas = self.HandleContactResult(contact_results)
        self.total_force_in_z = 0.0
        W_T_lfoot = self.l_foot_frame.CalcPoseInWorld(self.plant_context)
        W_T_rfoot = self.r_foot_frame.CalcPoseInWorld(self.plant_context)
        transformed_sensor_coordinate_l = []
        transformed_sensor_coordinate_r = []
        for p in self.sensor_coords:
            transformed_sensor_coordinate_l.append(W_T_lfoot@np.array(p).T)
            transformed_sensor_coordinate_r.append(W_T_rfoot@np.array(p).T)
        sensor_coordinates = np.concatenate((transformed_sensor_coordinate_l,
                                             transformed_sensor_coordinate_r))
        zmp_w, zmp_l, zmp_r= self.CalcZMP(force_datas=force_datas,
                                            sensor_coordinates=sensor_coordinates)
        self.ZMP_in_W[0] = zmp_w[0] + 0.005
        self.ZMP_in_W[1] = zmp_w[1]
        self.zmp_L_location = zmp_l
        self.zmp_R_location = zmp_r

    def HandleContactResult(self, contact_results:ContactResults):
        self.ground_contact_forces = {
                        "L_FL": np.zeros(3),
                        "L_FR": np.zeros(3),
                        "L_RL": np.zeros(3),
                        "L_RR": np.zeros(3),
                        "R_FL": np.zeros(3),            
                        "R_FR": np.zeros(3),
                        "R_RL": np.zeros(3),
                        "R_RR": np.zeros(3)
                    }
        self.total_force_l = np.zeros(3)
        self.total_moment_l = np.zeros(3)
        self.total_force_r = np.zeros(3)
        self.total_moment_r = np.zeros(3)
        # Point Contact
        if contact_results.num_point_pair_contacts() == 0:
            pass
        else:
            for i in range(contact_results.num_point_pair_contacts()):
                info = contact_results.point_pair_contact_info(i)
                bodyA_index = info.bodyA_index()
                bodyB_index = info.bodyB_index()
                bodyA_name = self.plant.get_body(bodyA_index).name()
                bodyB_name = self.plant.get_body(bodyB_index).name()
                if "foot_contact" in bodyA_name and "floor" in bodyB_name:
                    force = info.contact_force()
                    # contact_point_in_W = info.contact_point()
                    key =  bodyA_name.split("_")[0] + "_" + bodyA_name.split("_")[-1]
                    self.ground_contact_forces [key] = -1.0*force
                    self.total_force_in_z += -force[2]
                    if "L_" in key:
                        self.total_force_l += -force
                        # self.total_moment_l += np.cross(info.contact_point(), -1.0*force)
                    elif "R_" in key:
                        self.total_force_r += -force
                        # self.total_moment_r += np.cross(info.contact_point(), -1.0*force)

        # Hydroelastic Contact
        if contact_results.num_hydroelastic_contacts() == 0:
            pass
        else:
            # print("-"*40)
            for i in range(contact_results.num_hydroelastic_contacts()):
                info = contact_results.hydroelastic_contact_info(i)
                contact_surface = info.contact_surface()
                
                bodyA_name = self._geom_id_to_body_name.get(contact_surface.id_M(), "Unknown")
                bodyB_name = self._geom_id_to_body_name.get(contact_surface.id_N(), "Unknown")
                if "foot_contact" in bodyA_name and bodyB_name == "floor_top_link":
                    force = info.F_Ac_W().translational()
                    moment = info.F_Ac_W().rotational()
                    # print(f"Contact between: {bodyA_name} and {bodyB_name}")
                    # print(f"Contact force: {force}")
                    # print(f"Contact torque: {moment}")
                    key =  bodyA_name.split("_")[0] + "_" + bodyA_name.split("_")[-1]
                    self.ground_contact_forces [key] = 1.0*force
                    self.total_force_in_z += force[2]
                    if "L_" in key:
                        self.total_force_l += force
                        self.total_moment_l += moment
                    elif "R_" in key:
                        self.total_force_r += force
                        self.total_moment_r += moment

        return np.array([
                        self.ground_contact_forces["L_FL"][2],  # Front-right sensor
                        self.ground_contact_forces["L_FR"][2],  # Front-left sensor      
                        self.ground_contact_forces["L_RL"][2],  # Rear-left sensor
                        self.ground_contact_forces["L_RR"][2],   # Rear-right sensor
                        self.ground_contact_forces["R_FL"][2],  # Front-right sensor
                        self.ground_contact_forces["R_FR"][2],  # Front-left sensor
                        self.ground_contact_forces["R_RL"][2],  # Rear-left sensor
                        self.ground_contact_forces["R_RR"][2]   # Rear-right sensor
                        ])


    def CalcZMP(self,force_datas, sensor_coordinates):
        """
        Calculates the Zero Moment Point (ZMP) from force sensor data.

        Args:
            force_sensors: A list or numpy array of the force readings from each sensor.
                        Example: [F1, F2, F3, F4]
            sensor_coordinates: A list or numpy array of the (x, y) coordinates of each sensor.
                                Example: [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]

        Returns:
            A numpy array representing the (x, y) coordinates of the ZMP.
            Returns None if the total force is zero to avoid division by zero.
        """
        sum_force = np.sum(force_datas)
        weight_sum_x = 0.0
        weight_sum_y = 0.0
        temp_x = 0.0
        temp_y = 0.0
        for i in range(force_datas.shape[0]):
            weight_sum_x += force_datas[i]*sensor_coordinates[i][0]
            weight_sum_y += force_datas[i]*sensor_coordinates[i][1]
            if i == 3:
                temp_x = weight_sum_x
                temp_y = weight_sum_y
        zmpl_x = temp_x/np.sum(force_datas[0:4])
        zmpl_y = temp_y/np.sum(force_datas[0:4])
        zmpr_x = (weight_sum_x - temp_x)/np.sum(force_datas[4:8])
        zmpr_y = (weight_sum_y - temp_y)/np.sum(force_datas[4:8])
        zmp_x = weight_sum_x/sum_force
        zmp_y = weight_sum_y/sum_force
        return np.array([zmp_x, zmp_y]), np.array([zmpl_x, zmpl_y]), np.array([zmpr_x, zmpr_y])
    
    def CalcWrenchAtContactPoints(self,contact_forces_Z):
        momemt_L = np.zeros(3)
        momemt_R = np.zeros(3)
        fz_L = 0.0
        fz_R = 0.0
        contect_position = np.array([[0.08073, 0.02475, 0.0],
                                    [0.08073, -0.02475, 0.0],
                                    [-0.0907, 0.02475, 0.0],
                                    [-0.0907, -0.02475, 0.0]])
        for i in range(len(contact_forces_Z)):
            force = np.array([0.0, 0.0, contact_forces_Z[i]])
            moment = np.cross(contect_position[i%4,:], force)
            if i < 4:
                momemt_L += moment
                fz_L += contact_forces_Z[i]
            else:
                momemt_R += moment
                fz_R += contact_forces_Z[i]

    def CalcOutput_GRF(self, context, output):
        GRF_L = np.zeros(3)
        GRF_R = np.zeros(3)
        momemt_L = np.zeros(3)
        momemt_R = np.zeros(3)
        contect_position = np.array([[0.08073, 0.02475, 0.0],
                                    [0.08073, -0.02475, 0.0],
                                    [-0.0907, 0.02475, 0.0],
                                    [-0.0907, -0.02475, 0.0]])
        i=0
        for key, value in self.ground_contact_forces.items():
            force = np.array([0.0, 0.0, value[2]])
            moment = np.cross(contect_position[i%4,:], force)
            i+=1
            if "L_" in key:
                GRF_L += value
                momemt_L += moment
            elif "R_" in key:
                GRF_R += value
                momemt_R += moment
        out = np.concatenate((-1.0*momemt_L,GRF_L, -1.0*momemt_R,GRF_R))
        output.SetFromVector(out)
        
    def CalcOutput_ZMP(self, context, output):
        # zmp = np.zeros(4)
        output.SetFromVector(self.ZMP_in_W)

    def CalcOutput_CoP_in_foot_frame(self, context, output):
        output.SetFromVector(np.concatenate((self.zmp_L_location, self.zmp_R_location)))

    def CalcOutput_total_contact_wrenches(self, context, output):
        out = np.concatenate((self.total_moment_l, self.total_force_l, self.total_moment_r, self.total_force_r))
        output.SetFromVector(out)
