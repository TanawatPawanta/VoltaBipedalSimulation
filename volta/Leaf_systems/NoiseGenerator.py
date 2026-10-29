import numpy as np
from pydrake.systems.framework import LeafSystem


class RobotStateNoiseGenerator(LeafSystem):
    """Adds zero-mean Gaussian noise (global np.random) to the floating-base robot state.

    Input 0 clean state [7 + n + 6 + n], output 0 noisy state (quaternion re-normalised).
    """
    def __init__(self, num_joints=12, 
                 q_base_ori_sig=0.002,   # Orientation jitter
                 q_base_pos_sig=0.01,    # Translation/Mocap noise
                 q_joint_sig=0.0005,     # Encoder precision
                 v_base_ang_sig=0.05,    # Gyroscope noise
                 v_base_lin_sig=0.1,     # Estimated velocity drift
                 v_joint_sig=0.08):      # Differentiated joint vel
        LeafSystem.__init__(self)
        
        self.num_joints = num_joints
        self.nq = 7 + num_joints
        self.nv = 6 + num_joints
        
        # Mapping sigmas to a dictionary for easy access
        self.sigmas = {
            'q_ori': q_base_ori_sig,
            'q_pos': q_base_pos_sig,
            'q_jnt': q_joint_sig,
            'v_ang': v_base_ang_sig,
            'v_lin': v_base_lin_sig,
            'v_jnt': v_joint_sig
        }

        self.DeclareVectorInputPort("clean_state", self.nq + self.nv)
        self.DeclareVectorOutputPort("noisy_state", self.nq + self.nv, self.CalcOutput)

    def CalcOutput(self, context, output):
        state = self.get_input_port(0).Eval(context)
        noisy_state = np.array(state)
        
        # 1. Orientation Noise (Quat: w, x, y, z)
        noisy_state[0:4] += np.random.normal(0, self.sigmas['q_ori'], 4)
        # Normalize to maintain SO(3) validity
        noisy_state[0:4] /= np.linalg.norm(noisy_state[0:4])
        
        # 2. Translation Noise (x, y, z)
        noisy_state[4:7] += np.random.normal(0, self.sigmas['q_pos'], 3)
        
        # 3. Joint Position Noise
        noisy_state[7:self.nq] += np.random.normal(0, self.sigmas['q_jnt'], self.num_joints)

        # 4. Angular Velocity Noise (wx, wy, wz)
        v_start = self.nq
        noisy_state[v_start:v_start+3] += np.random.normal(0, self.sigmas['v_ang'], 3)
        
        # 5. Linear Velocity Noise (vx, vy, vz)
        noisy_state[v_start+3:v_start+6] += np.random.normal(0, self.sigmas['v_lin'], 3)

        # 6. Joint Velocity Noise
        noisy_state[v_start+6:] += np.random.normal(0, self.sigmas['v_jnt'], self.num_joints)

        output.SetFromVector(noisy_state)


# Deprecated misspelled names, kept so older notebooks keep working.
RobotStateNoisGenerator = RobotStateNoiseGenerator
