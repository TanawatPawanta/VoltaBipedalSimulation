import numpy as np
from pydrake.geometry import Rgba, Sphere
from pydrake.math import RigidTransform
from pydrake.systems.framework import LeafSystem


class WalkRecorder(LeafSystem):
    def __init__(self, meshcat, plant, context, model_instances):
        super().__init__()
        self.meshcat = meshcat
        self.plant = plant
        self.root_context = context
        self.model_instances = model_instances
        self.time_step  = 0.02
        plant_context = self.plant.GetMyContextFromRoot(self.root_context)
        self.CoM_world,self.CoM_base = self.CalcCoM(plant_context=plant_context)
        self.CoM_world_m1 = self.CoM_world
        self.W_P_Lsole,self.W_P_Rsole = self.CalcFootTraj(plant_context=plant_context)
        self.W_CoM_log = [self.CoM_world]
        self.L_P_com, self.R_P_com = self.CalcFootContact_P_com(plant_context=plant_context)
        self.ind = 0
        # self.CoM_world = [0.0,0.0,0.0]
        # self.CoM_base = 0.0
        # self.DeclareVectorOutputPort(
        #     "W_P_com",
        #     BasicVector(3),
        #     self.CalW_P_com
        # )
        # self.DeclareVectorOutputPort(
        #     "base_P_com",
        #     BasicVector(3),
        #     self.Calbase_P_com
        # )  
        # self.DeclareVectorOutputPort(
        #     "W_P_torso",
        #     BasicVector(3),
        #     self.Caltorso_P_W
        # ) 
        # self.DeclareVectorOutputPort(
        #     "L_P_com",
        #     BasicVector(3),
        #     self.CalcL_P_com
        # )
        # self.DeclareVectorOutputPort(
        #     "R_P_com",
        #     BasicVector(3),
        #     self.CalcR_P_com
        # )
        # self.DeclareVectorOutputPort(
        #     "W_P_feet",
        #     BasicVector(6),
        #     self.CalcW_P_feetOutput
        # )            

        self.DeclarePeriodicUnrestrictedUpdateEvent(period_sec=self.time_step                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   ,
                                                    offset_sec=0.0,
                                                    update=self.Update)
    def Update(self, context, state):
        plant_context = self.plant.GetMyContextFromRoot(self.root_context)
        self.CoM_world,self.CoM_base = self.CalcCoM(plant_context=plant_context)
        self.W_P_Lsole,self.W_P_Rsole = self.CalcFootTraj(plant_context=plant_context)
        self.L_P_com, self.R_P_com = self.CalcFootContact_P_com(plant_context=plant_context)
        # self.W_CoM_log.append(self.CoM_world)
        # if np.all(abs(self.CoM_world - self.CoM_world_m1) <= 0.00001) :
        self.meshcat.SetObject(f"com_sphere_{self.ind}", Sphere(0.001), Rgba(0.5, 0, 0.5, 1))
        self.meshcat.SetTransform(f"com_sphere_{self.ind}", RigidTransform(self.CoM_world))
        self.meshcat.SetObject(f"projected_com_sphere_{self.ind}", Sphere(0.001), Rgba(1, 0, 1, 1))
        comXY = np.array([self.CoM_world[0],self.CoM_world[1],0.0])
        self.meshcat.SetTransform(f"projected_com_sphere_{self.ind}", RigidTransform(comXY))
        self.meshcat.SetObject(f"L_sphere_{self.ind}", Sphere(0.002), Rgba(1, 0, 0, 1))
        self.meshcat.SetTransform(f"L_sphere_{self.ind}", RigidTransform(self.W_P_Lsole))
        self.meshcat.SetObject(f"R_sphere_{self.ind}", Sphere(0.002), Rgba(0, 0, 1, 1))
        self.meshcat.SetTransform(f"R_sphere_{self.ind}", RigidTransform(self.W_P_Rsole))
        self.ind += 1

    def Calbase_P_com(self,context,output):
        pass
    def CalW_P_com(self,context,output):
        output.SetFromVector(self.CoM_world)

    def Caltorso_P_W(self, context, output):
        base_frame = self.plant.GetFrameByName("torso") 
        # X_WB = base_frame.CalcPoseInWorld(self.plant.GetMyContextFromRoot(self.root_context))
        X_WB = np.zeros(3)
        output.SetFromVector(X_WB)

    # def CalcL_P_com(self,context,output):      
    #     output.SetFromVector(self.L_P_com) 

    # def CalcR_P_com(self,cotext,output):
    #     output.SetFromVector(self.R_P_com) 

    def CalcFootTraj(self, plant_context):
        L_foot_contact_frame = self.plant.GetFrameByName("L_foot_contact")
        W_T_Lsole = L_foot_contact_frame.CalcPoseInWorld(plant_context)
        W_P_Lsole = W_T_Lsole.translation()
        R_foot_contact_frame = self.plant.GetFrameByName("R_foot_contact")   
        W_T_Rsole = R_foot_contact_frame.CalcPoseInWorld(plant_context)   
        W_P_Rsole = W_T_Rsole.translation()  
        return W_P_Lsole,W_P_Rsole
    
    def CalcCoM(self, plant_context):
        com_world = self.plant.CalcCenterOfMassPositionInWorld(plant_context,self.model_instances)
        base_frame = self.plant.GetFrameByName("torso") 
        X_WB = base_frame.CalcPoseInWorld(plant_context)  
        com_base = X_WB.inverse().multiply(com_world)
        return com_world,com_base
    
    def CalcFootContact_P_com(self,plant_context):
        W_P_com = self.plant.CalcCenterOfMassPositionInWorld(plant_context,self.model_instances)
        R_foot_contact = self.plant.GetFrameByName("R_foot_contact")
        W_T_R = R_foot_contact.CalcPoseInWorld(plant_context)
        R_P_com = W_T_R.inverse().multiply(W_P_com)
        L_foot_contact = self.plant.GetFrameByName("L_foot_contact")
        W_T_L = L_foot_contact.CalcPoseInWorld(plant_context)
        L_P_com = W_T_L.inverse().multiply(W_P_com)
        return L_P_com, R_P_com
    
    def CalcW_P_feetOutput(self,cotext,output):
        output.SetFromVector(np.concatenate((self.W_P_Lsole,self.W_P_Rsole))) 
    
