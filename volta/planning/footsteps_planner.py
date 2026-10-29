"""Footstep planning from a reference velocity command v_ref(t)."""
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import numpy as np
import pydrake.solvers as solvers
from matplotlib.patches import Rectangle


class FootstepsPlanned:
    def __init__(self): 
        self.X = np.zeros(1)
        self.Y = np.zeros(1)
        self.orientation = np.zeros(1)
        self.timing = np.zeros(1)
        self.foot_side = "left"
    
    def ResetFootstep(self):
        self.X = np.zeros(1)
        self.Y = np.zeros(1)
        self.orientation = np.zeros(1)
        self.timing = np.zeros(1)
        self.foot_side = "left"

class FootstepsPlanner:
    def __init__(self,params:dict=None):
        # Parameters from Section III-A and VIII (Simulations)
        # Cruise parameters
        self.v_bar = 0.1       # Nominal velocity (m/s)
        self.T_bar = 1.0       # Nominal step duration (s)
        self.alpha = 0.1       # Parameter alpha for timing rule (m/s) 
        
        # Geometry and Constraints
        self.l = 0.125*2.0        # Reference coronal distance between footsteps (m)
        self.theta_max = np.pi/4  # Max rotation between steps (rad)
        
        # Kinematic admissible region (Rectangle dimensions)
        self.d_ax = 0.6        # Sagittal size
        self.d_ay = 0.2        # Coronal size
        # self.candidate_footsteps = FootstepsPlanned()
        self.plan = []
        self.params = params

    def ComputeStepDuration(self, v_mag):
        """
        Implements Equation (1): T_s = T_bar * (alpha + v_bar) / (alpha + v)
        """
        t = (self.params['ss_duration'] + self.params['ds_duration']) * self.params['mpc_time_step']
        # return self.T_bar * (self.alpha + self.v_bar) / (self.alpha + v_mag)
        return t

    def IntegrateReferenceTrajectory(self, v_ref_func, t_start, t_end, current_pose):
        """
        Integrates the unicycle template model (Eq 2) incrementally.
        """
        dt = 0.01 
        time_steps = np.arange(t_start, t_end, dt)
        x, y, theta = current_pose
        
        delta_x_global = 0.0
        delta_y_global = 0.0
        delta_theta_global = 0.0

        for t in time_steps:
            vx, vy, w = v_ref_func(t)
            
            # Global velocities based on current template orientation
            vx_global = vx * np.cos(theta) - vy * np.sin(theta)
            vy_global = vx * np.sin(theta) + vy * np.cos(theta)
            
            x += vx_global * dt
            y += vy_global * dt
            theta += w * dt
            
            delta_x_global += vx_global * dt
            delta_y_global += vy_global * dt
            delta_theta_global += w * dt

        return delta_x_global, delta_y_global, delta_theta_global

    def PlanFootsteps(self, initial_foot_pose, v_ref_func, preview_horizon, first_support_leg):
        """
        Generates candidate footsteps over the preview horizon using 
        Sequential QP (Orientation QP -> Placement QP).
        """
        steps = []
        t_current = 0.0
        self.preview_horizon = preview_horizon
        # Previous footstep state (initialized to current support)
        prev_x, prev_y, prev_theta = initial_foot_pose
        # self.candidate_footsteps.foot_side = first_support_leg
        # Determine swing leg (next step is opposite to support)
        # -1 for Right (if support is Left), 1 for Left (if support is Right)
        current_swing_sign = -1 if first_support_leg == 'lfoot' else 1 

        # --- 1. Generate Timing (Section III-A) ---
        timing_sequence = []
        step_timestamps = []
        while t_current < preview_horizon:
            # print(f"t_current: {t_current}")
            vx, vy, _ = v_ref_func(t_current)
            v_mag = np.sqrt(vx**2 + vy**2)
            
            T_s = self.ComputeStepDuration(v_mag)
            t_next = t_current + T_s
            
            if t_next > preview_horizon:
                break
                
            timing_sequence.append({'t_start': t_current, 't_end': t_next, 'T_s': T_s})
            step_timestamps.append(t_next)
            t_current = t_next
        # print("Footsteps timestamp",timing_sequence)
        # print("Step timestamp",step_timestamps)
        # print(len(step_timestamps))
        num_steps = len(timing_sequence)
        if num_steps == 0:
            return []

        # --- 2. Orientation QP (Section III-B) ---
        prog_theta = solvers.MathematicalProgram()
        theta_vars = prog_theta.NewContinuousVariables(num_steps, "theta")
        
        for j in range(num_steps):
            t_start = timing_sequence[j]['t_start']
            t_end = timing_sequence[j]['t_end']
            
            _, _, d_theta_ref = self.IntegrateReferenceTrajectory(
                v_ref_func, t_start, t_end, [0, 0, 0]
            )
            
            theta_prev = theta_vars[j-1] if j > 0 else prev_theta
            
            # Cost: Minimize (theta_j - theta_{j-1} - d_theta_ref)^2
            prog_theta.AddQuadraticCost(
                (theta_vars[j] - theta_prev - d_theta_ref)**2
            )
            
            # Constraint: |theta_j - theta_{j-1}| <= theta_max
            prog_theta.AddLinearConstraint(theta_vars[j] - theta_prev <= self.theta_max)
            prog_theta.AddLinearConstraint(theta_vars[j] - theta_prev >= -self.theta_max)

        result_theta = solvers.Solve(prog_theta)
        if not result_theta.is_success():
            print("Orientation QP failed")
            return []
        
        opt_thetas = result_theta.GetSolution(theta_vars)

        # --- 3. Placement QP (Section III-B) ---
        prog_pos = solvers.MathematicalProgram()
        x_vars = prog_pos.NewContinuousVariables(num_steps, "x")
        y_vars = prog_pos.NewContinuousVariables(num_steps, "y")
        
        ref_template_pose = [prev_x, prev_y, prev_theta] 

        for j in range(num_steps):
            t_start = timing_sequence[j]['t_start']
            t_end = timing_sequence[j]['t_end']
            sign = current_swing_sign * ((-1)**j) 
            
            # Calculate Nominal Displacement
            dx_ref, dy_ref, dth_ref = self.IntegrateReferenceTrajectory(
                v_ref_func, t_start, t_end, ref_template_pose
            )
            
            # Update virtual template pose
            ref_template_pose[0] += dx_ref
            ref_template_pose[1] += dy_ref
            ref_template_pose[2] += dth_ref
            
            theta_j = opt_thetas[j]
            c_j = np.cos(theta_j)
            s_j = np.sin(theta_j)
            
            # Lateral offset relative to body center
            lat_offset_local = np.array([0, sign * self.l / 1.0])
            
            # Rotate offset to world frame
            lat_offset_world = np.array([
                c_j * lat_offset_local[0] - s_j * lat_offset_local[1],
                s_j * lat_offset_local[0] + c_j * lat_offset_local[1]
            ])
            
            delta_x_nom = dx_ref + lat_offset_world[0]
            delta_y_nom = dy_ref + lat_offset_world[1]
            
            x_prev = x_vars[j-1] if j > 0 else prev_x
            y_prev = y_vars[j-1] if j > 0 else prev_y
            
            # Cost: || P_j - P_{j-1} - Delta_nom ||^2
            prog_pos.AddQuadraticCost( (x_vars[j] - x_prev - delta_x_nom)**2 )
            prog_pos.AddQuadraticCost( (y_vars[j] - y_prev - delta_y_nom)**2 )
            
            # Kinematic Constraints (Box in previous foot frame)
            theta_prev_val = opt_thetas[j-1] if j > 0 else prev_theta
            c_p = np.cos(theta_prev_val)
            s_p = np.sin(theta_prev_val)
            
            # Center of reachable box relative to previous foot
            center_shift = np.array([0, 1.0*sign * self.l]) 
            box_half_size = 0.5 * np.array([self.d_ax, self.d_ay])
            
            ub = center_shift + box_half_size
            lb = center_shift - box_half_size
            
            # Rotate difference vector into previous foot frame
            diff_x = x_vars[j] - x_prev
            diff_y = y_vars[j] - y_prev
            
            x_local = c_p * diff_x + s_p * diff_y
            y_local = -s_p * diff_x + c_p * diff_y
            
            prog_pos.AddLinearConstraint(x_local <= ub[0])
            prog_pos.AddLinearConstraint(x_local >= lb[0])
            prog_pos.AddLinearConstraint(y_local <= ub[1])
            prog_pos.AddLinearConstraint(y_local >= lb[1])

        result_pos = solvers.Solve(prog_pos)
        if not result_pos.is_success():
            print("Placement QP failed")
            return []

        opt_xs = result_pos.GetSolution(x_vars)
        opt_ys = result_pos.GetSolution(y_vars)

        # self.candidate_footsteps.X = np.append(prev_x,opt_xs)
        # self.candidate_footsteps.Y = np.append(prev_y,opt_ys) 
        # self.candidate_footsteps.orientation = np.append(prev_theta,opt_thetas)
        # self.candidate_footsteps.timing = np.append(0.0,step_timestamps)
        # print(timing_sequence)
        if first_support_leg == 'rfoot':
            temp0 = 'lfoot'
            temp1 = 'rfoot'
        else:
            temp0 = 'rfoot'
            temp1 = 'lfoot'
        steps.append({
                    'index': 0,
                    'pos': np.array([prev_x, -prev_y,  0. ]),
                    'ang': np.array([0., 0., prev_theta]),
                    'ss_duration': 0,
                    'ds_duration': 200,
                    'foot_id': temp0
        })
        steps.append({
                    'index': 1,
                    'pos': np.array([prev_x, prev_y,  0. ]),
                    'ang': np.array([0., 0., prev_theta]),
                    'ss_duration': self.params['ss_duration'],
                    'ds_duration': self.params['ds_duration'],
                    'foot_id': temp1
        })
        
        for j in range(num_steps):
            side = 'lfoot' if (current_swing_sign * ((-1)**j)) == 1 else 'rfoot'
            steps.append({
                        'index': j+2,
                        'pos': np.array([opt_xs[j], opt_ys[j],  0. ]),
                        'ang': np.array([0., 0., opt_thetas[j]]),
                        'ss_duration': self.params['ss_duration'],
                        'ds_duration': self.params['ds_duration'],
                        'foot_id': side
            })
        self.plan = steps
        return steps

    def get_step_index_at_time(self, time):
        t = 0
        # print(f"time {time}")
        for i in range(len(self.plan)):
            t += self.plan[i]['ss_duration'] + self.plan[i]['ds_duration']
            if t >= time: return i
        return None

    def get_start_time(self, step_index):
        t = 0
        for i in range(step_index):
            t += self.plan[i]['ss_duration'] + self.plan[i]['ds_duration']
        return t

    def get_phase_at_time(self, time):
        step_index = self.get_step_index_at_time(time)
        # print(f"step_index {step_index}")
        start_time = self.get_start_time(step_index)
        time_in_step = time - start_time
        if time_in_step < self.plan[step_index]['ss_duration']:
            return 'ss'
        else:
            return 'ds'
    def GetStanceAtTime(self,time):
        step_index = self.get_step_index_at_time(time)
        return self.plan[step_index]['foot_id']
    
    def PlotFootsteps(self, steps, initial_pose, v_ref_func=None, label_idx_offset=0, save_path=None):
        """
        Visualizes the plan with foot identification labels (Legend).
        """
        if v_ref_func:
            fig, axes = plt.subplots(2, 1, figsize=(10, 10), 
                                    gridspec_kw={'height_ratios': [2, 1]})
            ax_map, ax_vel = axes[0], axes[1]
        else:
            fig, ax_map = plt.subplots(figsize=(10, 6))
            ax_vel = None

        ax_map.set_title("Footstep Planner Output")
        ax_map.set_xlabel("X (m)")
        ax_map.set_ylabel("Y (m)")
        ax_map.axis('equal')
        ax_map.grid(True, alpha=0.3)

        def DrawFoot(ax, x, y, theta, side, label_idx=None):
            width = 0.23  
            height = 0.11 
            # Mapping foot_id to standard humanoid nomenclature
            is_left = (side == 'lfoot')
            color = 'red' if is_left else 'blue'
            label_text = 'Left Foot' if is_left else 'Right Foot'
            
            rect = Rectangle((-width/2, -height/2), width, height, 
                            color=color, alpha=0.5)
            
            t = transforms.Affine2D().rotate(theta).translate(x, y) + ax.transData
            rect.set_transform(t)
            ax.add_patch(rect)
            
            ax.arrow(x, y, 0.15*np.cos(theta), 0.15*np.sin(theta),
                    head_width=0.03, color='k', alpha=0.7)
            
            if label_idx is not None:
                ax.text(x, y, str(label_idx), color='white', 
                        ha='center', va='center', fontweight='bold', fontsize=8)
                
            return rect, label_text

        # Track which labels we've already added to avoid redundant legend entries
        handles = {}
        
        # Draw Steps
        for s in steps:
            rect, label_text = DrawFoot(ax_map, s['pos'][0], s['pos'][1], s['ang'][2], s['foot_id'], 
                                        label_idx=s['index']+label_idx_offset)
            if label_text not in handles:
                handles[label_text] = rect

        # Add the Legend to the map
        ax_map.legend(handles.values(), handles.keys(), loc='upper left', framealpha=1.0)

        # Draw Path (Center of Mass / Support transition path)
        xs = [initial_pose[0]] + [s['pos'][0] for s in steps]
        ys = [initial_pose[1]] + [s['pos'][1] for s in steps]
        ax_map.plot(xs, ys, 'k--', alpha=0.3, marker='o', markersize=3, label='Swing Path')

        # Velocity Plot
        if ax_vel and v_ref_func and steps:
            t_end = self.preview_horizon + 0.5
            ts = np.linspace(0, t_end, 200)
            v_data = np.array([v_ref_func(t) for t in ts])
            
            ax_vel.set_title("Reference Velocity Input")
            ax_vel.plot(ts, v_data[:, 0], label='$v_x$')
            ax_vel.plot(ts, v_data[:, 1], label='$v_y$')
            ax_vel.plot(ts, v_data[:, 2], label=r'$\omega$')
            ax_vel.set_xlabel("Time (s)")
            ax_vel.set_ylabel("Velocity (m/s | rad/s)")
            ax_vel.legend()
            ax_vel.grid(True, alpha=0.3)

        plt.tight_layout()
        if save_path:
                import os
                # Create directory if it doesn't exist
                directory = os.path.dirname(save_path)
                if directory and not os.path.exists(directory):
                    os.makedirs(directory)
                    
                # Save with high DPI for thesis quality
                plt.savefig(save_path, dpi=300, bbox_inches='tight')
                print(f"Plot saved to: {save_path}")
        plt.show()
