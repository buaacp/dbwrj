import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import math

class MPC:
    def __init__(self):
        # set the Parameters of MPC
        # state weight matrix ( q1, ..., q4)[4]
        self.Q = ca.diagcat(0, 2, 0.5, 0)
        # controls weights matrix
        Ra = 1
        # (q1, ..., q4)[4]
        self.R = ca.diagcat(Ra, Ra, Ra/4, Ra/8)
        R_delta = 0.2
        self.R_delta = ca.diagcat(R_delta, R_delta, R_delta, R_delta)
        self.step_horizon = 0.05  # time between steps in seconds
        self.N = 50  # number of look ahead steps

        # states symbolic variables
        self.n_states = 4
        # control symbolic variables
        self.n_controls = 4
        self.n_tarpos = 6
        # arm tip
        self.state_target = ca.DM([0, 0, 0, 0])  # target state
        self.u_pre = ca.DM([0, 0, 0, 0])  # target state
        # set the para of arm
        self.Qa = ca.diag([400, 400, 400])
        self.Qa_rot = ca.diag([1])
        # control limit
        self.v_arm_max = 0.4
        self.v_arm_min = -0.4
        self.L1 = 0.1049
        self.L2 = 0.0884
        self.L3 = 0.12
        self.k_exp = -0.5

        self.create_symbolic_variables()
        self.build_cost_function()
        self.set_limit()
        self.set_optimize_option()

    def create_symbolic_variables(self):
        # state symbolic variables
        q_mm = ca.SX.sym('q_mm', self.n_states)  # (x,y,theta,q1-q6)
        u_mm = ca.SX.sym('u_mm', self.n_controls)  # dot(v,w,q1-q6)

        # matrix containing all states over all time steps +1 (each column is a state vector)
        self.X = ca.SX.sym('X', self.n_states, self.N + 1)
        # matrix containing all control actions over all time steps (each column is an action vector)
        self.U = ca.SX.sym('U', self.n_controls, self.N)
        self.P = ca.SX.sym('P', 2 * self.n_states + self.n_tarpos * (self.N + 1))
        self.P_arm = ca.reshape(self.P[2 * self.n_states:], self.n_tarpos, self.N + 1)
        RHS = ca.vertcat(u_mm[0],
                         u_mm[1],
                         u_mm[2],
                         u_mm[3])
        self.f = ca.Function('f_mm', [q_mm, u_mm], [RHS])

    def RobFki(self,st):
        q0, q1, q2, q3 = st[0], st[1], st[2], st[3]
        arm_z = -(self.L1 * cos(q1) + self.L2 * cos(q1 + q2) + self.L3 * cos(q1 + q2 + q3))
        arm_y = cos(q0) * (self.L1 * sin(q1) + self.L2 * sin(q1 + q2) + self.L3 * sin(q1 + q2 + q3))
        arm_x = -sin(q0) * (self.L1 * sin(q1) + self.L2 * sin(q1 + q2) + self.L3 * sin(q1 + q2 + q3))
        attitude_arm = ca.vertcat(-sin(q0)*sin(q1 + q2 + q3), cos(q0)*sin(q1 + q2 + q3), -cos(q1 + q2 + q3))
        return ca.vertcat(arm_x, arm_y, arm_z), attitude_arm
    def build_cost_function(self):
        self.cost_fn = 0  # cost function
        self.g = self.X[:, 0] - self.P[:self.n_states]  # constraints in the equation
        # runge kutta
        u_pre =self.u_pre
        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]
            x_arm, attitude_arm = self.RobFki(st)
            attitude_error = ca.exp(-(attitude_arm.T @ self.P_arm[3:6, k]))
            delta_u = u_pre - con
            if k==self.N-1:
                self.cost_fn = (self.cost_fn\
                            + delta_u.T @ self.R_delta @ delta_u\
                            + st.T @ self.Q @ st \
                            + con.T @ self.R @ con \
                            + 100*(x_arm - self.P_arm[:3, k]).T @ self.Qa @ (x_arm - self.P_arm[:3, k]) \
                            + 10*attitude_error.T @ self.Qa_rot @ attitude_error)
            else:
                self.cost_fn = (self.cost_fn\
                            + delta_u.T @ self.R_delta @ delta_u\
                            + st.T @ self.Q @ st \
                            + con.T @ self.R @ con \
                            + (x_arm - self.P_arm[:3, k]).T @ self.Qa @ (x_arm - self.P_arm[:3, k]) \
                            + attitude_error.T @ self.Qa_rot @ attitude_error)
            st_next = self.X[:, k + 1]
            k1 = self.f(st, con)
            k2 = self.f(st + self.step_horizon / 2 * k1, con)
            k3 = self.f(st + self.step_horizon / 2 * k2, con)
            k4 = self.f(st + self.step_horizon * k3, con)
            st_next_RK4 = st + (self.step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            self.g = ca.vertcat(self.g, st_next - st_next_RK4)
            u_pre = con

    def set_optimize_option(self):
        OPT_variables = ca.vertcat(
            self.X.reshape((-1, 1)),
            self.U.reshape((-1, 1))
        )
        nlp_prob = {
            'f': self.cost_fn,
            'x': OPT_variables,
            'g': self.g,
            'p': self.P
        }
        opts = {
            'ipopt': {
                'max_iter': 1000,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    def set_limit(self):
        n_states, n_controls = self.n_states, self.n_controls
        N = self.N
        lbx = ca.DM.zeros((n_states * (N + 1) + n_controls * N, 1))
        ubx = ca.DM.zeros((n_states * (N + 1) + n_controls * N, 1))

        limit = [
            [x * math.pi / 180 for x in joint_limits]  # 每个关节的角度单独转换
            for joint_limits in [
                [-90, 90],  # 关节1：-130°~130°
                [-60, 90],  # 关节2：-60°~90°
                [-85, 85],  # 关节3：-85°~85°
                [-85, 85]  # 关节4：-85°~85°
            ]
        ]

        lbx[0: n_states * (N + 1): n_states] = limit[0][0]  # q1 lower bound
        lbx[1: n_states * (N + 1): n_states] = limit[1][0]  # q2 lower bound
        lbx[2: n_states * (N + 1): n_states] = limit[2][0]  # q3 lower bound
        lbx[3: n_states * (N + 1): n_states] = limit[3][0]  # q4 lower bound


        ubx[0: n_states * (N + 1): n_states] = limit[0][1]  # q1 upper bound
        ubx[1: n_states * (N + 1): n_states] = limit[1][1]  # q2 upper bound
        ubx[2: n_states * (N + 1): n_states] = limit[2][1]  # q3 upper bound
        ubx[3: n_states * (N + 1): n_states] = limit[3][1]  # q4 upper bound

        lbx[n_states * (N + 1):] = self.v_arm_min  # u lower bound for all U
        ubx[n_states * (N + 1):] = self.v_arm_max  # u upper bound for all U

        self.args = {
            'lbg': ca.DM.zeros((n_states * (N + 1), 1)),  # constraints lower bound
            'ubg': ca.DM.zeros((n_states * (N + 1), 1)),  # constraints upper bound
            'lbx': lbx,
            'ubx': ubx
        }

    def set_reference(self, p):
        self.args['p'] = p

    def set_x0(self, X0, u0):
        self.args['x0'] = ca.vertcat(
            ca.reshape(X0, self.n_states * (self.N + 1), 1),
            ca.reshape(u0, self.n_controls * self.N, 1)
        )

    def get_states_and_control(self):
        sol = self.solver(
            x0=self.args['x0'],
            lbx=self.args['lbx'],
            ubx=self.args['ubx'],
            lbg=self.args['lbg'],
            ubg=self.args['ubg'],
            p=self.args['p']
        )
        u = ca.reshape(sol['x'][self.n_states * (self.N + 1):], self.n_controls, self.N)
        X0 = ca.reshape(sol['x'][: self.n_states * (self.N + 1)], self.n_states, self.N + 1)
        self.u_pre = u[:, 0]
        return X0, u
    def get_exp_point(self,p_target_local,flag_leave,flag_default):
        # 调试信息：输入参数
        distance_b = np.linalg.norm(p_target_local)
        L_max = self.L1+self.L2+self.L3
        L_min = self.L1+self.L2
        p_target_default = np.array([0, 0, -L_max]).reshape(-1, 1)
        pose_exp = np.array([0, 0, 0]).reshape(-1, 1)
        if distance_b > L_max:
            if flag_default:
                p_target_local = p_target_default
            else:
                # 期望距离计算 L_min-L_max  越远距离越小
                L_exp = L_min + (L_max-L_min) * math.exp(self.k_exp * (distance_b - L_max))
                if(p_target_local[2] > -L_min):
                    p_target_local[2] = -L_min
                p_end = np.array([0, 0, -L_min]).reshape(-1, 1)
                # 下方L_min处与目标点的连线
                vector_e = p_target_local - p_end
                vector_e = vector_e/np.linalg.norm(vector_e)

                vector1 = np.array([0, 0, -L_min])
                c = np.dot(vector1, vector_e)
                # 解二次方程
                discriminant = c**2 - (L_min ** 2 - L_exp ** 2)
                if discriminant >= 0:
                    sqrt_discriminant = np.sqrt(discriminant)
                    k1_candidates = [-c + sqrt_discriminant, -c - sqrt_discriminant]
                    k1 = max([k for k in k1_candidates if k >= 0])  # 选择非负解
                else:
                    # 处理无解情况（如取边界值或报错）
                    k1 = 0  # 或其他默认逻辑
                pose_exp = p_target_local - (vector1.reshape(-1, 1)+k1*vector_e)
                pose_exp = pose_exp/np.linalg.norm(pose_exp)
                if flag_leave:
                    pose_exp = np.array([0, 0, 0]).reshape(-1, 1)
                p_target_local = vector1.reshape(-1, 1)+k1*vector_e
        return p_target_local,pose_exp

    def get_target_path(self, uav_arm):
        arg_p_arm = ca.DM.zeros(self.n_tarpos * (self.N + 1))
        p_b = np.copy(uav_arm.p_b)
        pos_target = np.copy(uav_arm.pos_target)
        phi = np.copy(uav_arm.phi)
        theta = np.copy(uav_arm.theta)
        delta = np.copy(uav_arm.delta)

        for k in range(self.N + 1):
            # 计算无人机是否处于脱离状态
            flag_leave = 0
            flag_default = 0
            uav_tar_dis = pos_target - p_b
            aim_heading_vector = np.array([uav_tar_dis[0, 0], uav_tar_dis[1, 0]]).flatten()
            real_heading_vector = np.array([-1 * np.sin(delta), np.cos(delta)])
            if np.dot(aim_heading_vector, real_heading_vector) < 0:
                flag_leave = 1
                if np.linalg.norm(uav_tar_dis)>10:
                    flag_default = 1

            R_z = np.array([
                [np.cos(delta), -np.sin(delta), 0],
                [np.sin(delta), np.cos(delta), 0],
                [0, 0, 1]
            ])
            R_x = np.array([
                [1, 0, 0],
                [0, np.cos(theta), -np.sin(theta)],
                [0, np.sin(theta), np.cos(theta)]
            ])
            R_y = np.array([
                [np.cos(phi), 0, np.sin(phi)],
                [0, 1, 0],
                [-np.sin(phi), 0, np.cos(phi)]
            ])
            R_b = R_x @ R_y @ R_z

            R_b_inv = np.linalg.inv(R_b)  # 逆矩阵
            p_target_local = np.dot(R_b_inv, (pos_target - p_b)) - uav_arm.p_delta
            p_exp,pose_exp = self.get_exp_point(p_target_local,flag_leave,flag_default)

            pose_target = [p_exp[0], p_exp[1], p_exp[2], pose_exp[0], pose_exp[1], pose_exp[2]]

            for j in range(self.n_tarpos):
                arg_p_arm[k * self.n_tarpos + j] = pose_target[j]

            # 更新预测状态
            # p_b += uav_arm.dt * uav_arm.d_xb[0:3]
            # delta += uav_arm.dt * uav_arm.d_xb[5, 0]  # 取消航向补偿
        return arg_p_arm