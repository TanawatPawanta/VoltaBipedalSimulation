"""QP (Clarabel) that distributes the desired CoM wrench into per-foot contact wrenches with friction-cone, CoP and Fz-bound constraints."""
import time

import numpy as np
from numpy.linalg import inv
from pydrake.solvers import ClarabelSolver, MathematicalProgram


class ContactWrenchSolver():
    def __init__(self, fz_boundaries=np.array([5.0, 55.0, 5.0, 55.0])):
        self.default_fz_bounds = fz_boundaries  # [L_min, L_max, R_min, R_max]
        self.mu = 0.7
        self.foot_size = np.array([0.220, 0.098])
        # self.solver = NloptSolver()
        # self.solver = IpoptSolver()
        # self.solver = SnoptSolver()
        # self.solver = OsqpSolver()
        # self.solver = ScsSolver()
        self.solver = ClarabelSolver()
        
        # 1. Pre-build programs for different stance modes
        self.prog6 = self._build_program(6)
        self.prog12 = self._build_program(12)
        self.Qc = 30.0
        # self.Qc = 80.0
        self.Qf = 0.0001
        self.last_sol = None
        self.Fz_boundary = np.zeros(4)

    def _build_program(self, n_vars):
        prog = MathematicalProgram()
        f = prog.NewContinuousVariables(n_vars, "f")
        
        # Placeholder Quadratic Cost: 0.5*f'Qf + cf
        cost = prog.AddQuadraticCost(np.eye(n_vars), np.zeros(n_vars), f)
        
        # Store Fz BoundingBox references to update later
        fz_constraints = []
        num_feet = n_vars // 6
        for i in range(num_feet):
            off = i * 6
            # Add placeholders with dummy bounds [0, 1]
            fz_con = prog.AddBoundingBoxConstraint(0, 1, f[off + 5])
            fz_constraints.append(fz_con)
            
            # These structural constraints don't need to be updated
            self._add_friction_cone(prog, f[off+3 : off+6])
            self._add_cop_limits(prog, f[off : off+3], f[off+3 : off+6])
            
        return {"prog": prog, "f": f, "cost": cost, "fz_cons": fz_constraints}

    def SolveContactWrench(self, wff_c, wg, wimp_c, com_J_leg, f_imp,fz_ref, 
                            leg_state, next_leg_state, transition_rate, leg_phase):
        
        # Select structure
        is_double = "double" in leg_state or "transition" in leg_state
        struct = self.prog12 if is_double else self.prog6
        prog, f_vars, cost, fz_refs = struct["prog"], struct["f"], struct["cost"], struct["fz_cons"]
        
        # 1. Update Cost Matrices (Vectorized)
        if not is_double:
            J = com_J_leg[:6, :6] if leg_state == "L_stance" else com_J_leg[6:, :6]
            J_imp = com_J_leg[6:, :6] if leg_state == "L_stance" else com_J_leg[:6, :6]
        else:
            J, J_imp = com_J_leg[:, :6], np.zeros((6, 6))

        b_target = wff_c - wg - wimp_c - J_imp @ f_imp
        m_grf = len(f_vars)
        f_ref = np.zeros(m_grf)
        if is_double:
            f_ref[5], f_ref[11] = fz_ref[0], fz_ref[1]
        else:
            f_ref[5] = fz_ref[0] if leg_state == "L_stance" else fz_ref[1]

        # 2. Update Cost Matrices
        
        # Calculate Q_total: J*Qc*J^T + Qf*I
        Q_total = (J * self.Qc) @ J.T 
        np.fill_diagonal(Q_total, Q_total.diagonal() + self.Qf)
        
        # Calculate c_total: -(J*Qc*b_target + Qf*f_ref)
        c_total = -(J @ (self.Qc * b_target)) - (self.Qf * f_ref)

        # 3. FAST UPDATE
        cost.evaluator().UpdateCoefficients(Q_total, c_total)

        # 2. Update Fz Bounds (Fast update)
        if not is_double:
            idx = 0 if leg_state == "L_stance" else 2
            low, high = self._calc_adaptive_bounds(self.default_fz_bounds[idx:idx+2], f_imp[5], transition_rate, 1)
            self.Fz_boundary[idx] = low
            self.Fz_boundary[idx+1] = high
            fz_refs[0].evaluator().set_bounds(np.array([low]), np.array([high]))
        else:
            nl_p = [1, 0] if next_leg_state == "L_stance" else [0, 1] if next_leg_state == "R_stance" else [1, 1]
            for i in range(2):
                rate = transition_rate if nl_p[i] != leg_phase[i] else 0.0
                low, high = self._calc_adaptive_bounds(self.default_fz_bounds[0:2], f_imp[5], rate, leg_phase[i])
                # print(f"deful Fz boundary for leg l/h {i}: [{self.default_fz_bounds[2*i]}, {self.default_fz_bounds[2*i+1]}]")
                if i == 0:
                    self.Fz_boundary[0] = low
                    self.Fz_boundary[1] = high
                elif i == 1:
                    self.Fz_boundary[2] = low
                    self.Fz_boundary[3] = high
                # print(f"Fz boundary for leg l/h {i}: [{low}, {high}]")
                fz_refs[i].evaluator().set_bounds(np.array([low]), np.array([high]))

        # 3. Solve with Warm Start
        guess = self.last_sol if (self.last_sol is not None and len(self.last_sol) == len(f_vars)) else np.zeros(len(f_vars))
        start = time.time()
        result = self.solver.Solve(prog, guess)
        end = time.time()
        # print(f"Time consumed in optimization: {end-start:.4f} seconds")
        # print("Optimal cost = ", result.get_optimal_cost())
        # print("Solver result: ", result.get_solution_result())
        # print(f"Solver used: {result.get_solver_id().name()}")
        if result.is_success():
            f_grf_sol = result.GetSolution(f_vars)
            
            if leg_state == "L_stance":
                foot_wrench = np.concatenate((f_grf_sol, f_imp))
            elif leg_state == "R_stance":
                foot_wrench = np.concatenate((f_imp, f_grf_sol))
            else:
                foot_wrench = f_grf_sol
            self.last_sol = foot_wrench
            return self.last_sol, self.Fz_boundary  # Simplified return
        return np.zeros(len(f_vars)),low, high

    def _calc_adaptive_bounds(self, bound, fz_imp, rate, phase):
        if phase: # Stance to Swing
            return rate * fz_imp + (1-rate)*bound[0], rate * fz_imp + (1-rate)*bound[1]
        # Swing to Stance
        return rate * bound[0] + (1-rate)*fz_imp, rate * bound[1] + (1-rate)*fz_imp

    def _add_friction_cone(self, prog, force):
        A = np.array([[0, 0, self.mu], [1, 0, 0], [0, 1, 0]])
        prog.AddLorentzConeConstraint(A, np.zeros(3), force)

    def _add_cop_limits(self, prog:MathematicalProgram, moment, force):
        lx, ly = self.foot_size - 0.01
        # Mx and My constraints relative to Fz
        prog.AddLinearConstraint(moment[0] <= ly * force[2])
        prog.AddLinearConstraint(moment[0] >= -ly * force[2])
        prog.AddLinearConstraint(moment[1] <= lx * force[2])
        prog.AddLinearConstraint(moment[1] >= -lx * force[2])

def AddTorqueLimitConstraint(prog:MathematicalProgram,
                            J_grf_q:np.ndarray,
                            m_grf:int,
                            f_grf:np.ndarray):
    """
    Torque limit constraint: |tau| <= tau_max
    """
    tau_max = 6.0 # Nm
    print("J_grf_q shape: ",J_grf_q.shape)
    wrench_max = inv(J_grf_q.T) @ np.ones([m_grf,1]) * tau_max
    print("wrench_max: ",wrench_max.T)
    for i in range(m_grf):
        prog.AddBoundingBoxConstraint(-1.0*abs(wrench_max[i]),abs(wrench_max[i]),f_grf[i])

def CalcContactJacobian(plant, context, left_foot_name="L_foot_contact", right_foot_name="R_foot_contact"):
    """
    Compute the spatial Jacobian J_grf_c that maps contact wrenches at both feet
    to the resulting wrench at the CoM (as used in Eq. (11) of the paper).

    J_grf_c ∈ R^{6×12}, such that:
        w_CoM = J_grf_c^T * [f_L; f_R]
    
    Each foot block is 6×6:
        [ I      0      ]
        [ [r]_x  I      ]
    where r = p_CoM - p_contact (vector from contact to CoM)
    """

    def skew(v):
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])

    # CoM position in world frame
    p_CoM = plant.CalcCenterOfMassPositionInWorld(context)

    # Contact frame positions in world frame
    p_L = plant.CalcPointsPositions(context, plant.GetFrameByName(left_foot_name),
                                    np.zeros(3), plant.world_frame()).flatten()
    p_R = plant.CalcPointsPositions(context, plant.GetFrameByName(right_foot_name),
                                    np.zeros(3), plant.world_frame()).flatten()

    # Moment arms from each foot to CoM
    r_L = p_CoM - p_L
    r_R = p_CoM - p_R

    J_L = np.block([
        [np.eye(3), np.zeros((3,3))],
        [skew(r_L).T, np.eye(3)]
    ])
    J_R = np.block([
        [np.eye(3), np.zeros((3,3))],
        [skew(r_R).T, np.eye(3)]
    ])

    # Combine into 6x12 matrix
    J_grf_c = np.vstack([J_L, J_R])
    return J_grf_c