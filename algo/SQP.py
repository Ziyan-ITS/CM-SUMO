import numpy as np
from scipy.optimize import minimize, LinearConstraint, Bounds


class sqp_lower_control:

    def __init__(self, vehicle_set, tf, t0, reason):
        """
        :param vehicle_set: 传入的底层set， 如果与后车
        :param tf:
        :param t0:
        """
        self.vehicle_set = []
        for veh in vehicle_set:
            if not veh:
                self.vehicle_set.append(None)
                continue
            vehicle_state = []
            for time_step in veh:
                single_step = {'vehicle_speed': time_step['vehicle_speed'], 'vehicle_dis': -time_step['vehicle_dis']}
                vehicle_state.append(single_step)
            self.vehicle_set.append(vehicle_state)
        self.control_vehicle_set = []
        self.control_vehicle = None
        self.tf = tf + 1
        self.t0 = t0
        self.Tsim = self.tf - self.t0
        self.reason = reason
        self.situation = []

        for vehicle in vehicle_set:
            if vehicle:
                self.situation.append(vehicle[0]['vehicle_type'])
            else:
                self.situation.append(None)
        if self.reason == 1:
            if self.situation[0] == 'cav':
                # 控制主路前车
                self.control_vehicle_set.append('leader')
            if self.situation[1] == 'cav':
                # 控制匝道车辆
                self.control_vehicle_set.append('ramp')
        elif self.reason == 2:
            if self.situation[1] == 'cav':
                self.control_vehicle_set.append('ramp')
            if self.situation[2] == 'cav':
                self.control_vehicle_set.append('follower')

    def run_optimization(self):
        if not self.control_vehicle_set:
            return False
        out = []
        for self.control_vehicle in self.control_vehicle_set:
            if self.control_vehicle == 'ramp':
                initial_speed = self.vehicle_set[1][0]['vehicle_speed']
                initial_loc = self.vehicle_set[1][0]['vehicle_dis']
            elif self.control_vehicle == 'leader':
                initial_speed = self.vehicle_set[0][0]['vehicle_speed']
                initial_loc = self.vehicle_set[0][0]['vehicle_dis']
            else:
                initial_speed = self.vehicle_set[2][0]['vehicle_speed']
                initial_loc = self.vehicle_set[2][0]['vehicle_dis']

            A1, B1, A2, B2, V, X_up = self.generate_transition_matrix(initial_speed, initial_loc)

            fs_mainline_leader_loc = self.vehicle_set[0][self.tf]['vehicle_dis'] if self.situation[0] is not None else None
            fs_mainline_leader_speed = self.vehicle_set[0][self.tf]['vehicle_speed'] if self.situation[
                                                                                            0] is not None else None
            fs_mainline_follower_loc = self.vehicle_set[2][self.tf]['vehicle_dis'] if self.situation[
                                                                                          2] is not None else None
            fs_mainline_follower_speed = self.vehicle_set[2][self.tf]['vehicle_speed'] if self.situation[
                                                                                              2] is not None else None
            fs_ramp_loc = self.vehicle_set[1][self.tf]['vehicle_dis']
            fs_ramp_speed = self.vehicle_set[1][self.tf]['vehicle_speed']
            ini = np.mat([[initial_speed], [initial_loc]])
            V_ref = np.ones([self.Tsim, 1]) * 20
            result = self.cons_all(V_ref, A1, B1, ini, B2, X_up, fs_mainline_leader_speed,
                                   fs_mainline_leader_loc, fs_mainline_follower_speed
                                   , fs_mainline_follower_loc,
                                   fs_ramp_speed, fs_ramp_loc)
            x0 = np.ones([A1.shape[0]])
            res = minimize(self.obj, x0, args=(A1, ini, B1, V_ref), method='SLSQP', constraints=result)
            out.append(res.x)
        return out

    def generate_transition_matrix(self, initial_speed, initial_location):
        T_shift = 1
        a_shift = np.mat([[1, 0], [-T_shift, 1]])
        b_shift = np.array([[T_shift], [-T_shift ** 2 / 2]])
        c_shift = np.array([1, 0])

        v0 = initial_speed
        s0 = initial_location
        ini = np.mat([[v0], [s0]])

        A1_cell = []
        B1_cell = []
        A2_cell = []
        B2_cell = []

        for i in range(1, self.Tsim + 1, T_shift):
            A1_cell.append(np.array(np.matmul(c_shift, a_shift ** i)).squeeze())
            b1_cell = []
            for j in range(1, self.Tsim + 1, T_shift):
                if i <= j:
                    b1_cell.append(np.array(np.matmul(np.matmul(c_shift, (a_shift ** (j - i))), b_shift)).squeeze())
                else:
                    b1_cell.append(np.zeros(1).squeeze())
            B1_cell.append(b1_cell)

        c2 = [0, 1]
        for i in range(1, self.Tsim + 1, T_shift):
            A2_cell.append(np.array(np.matmul(c2, a_shift ** i)).squeeze())
            b2_cell = []
            for j in range(1, self.Tsim + 1, T_shift):
                if i <= j:
                    b2_cell.append(np.array(np.matmul(np.matmul(c2, (a_shift ** (j - i))), b_shift)).squeeze())
                else:
                    b2_cell.append(np.zeros(1).squeeze())
            B2_cell.append(b2_cell)

        A1 = np.mat(A1_cell)
        B1 = np.mat(B1_cell).T
        A2 = np.mat(A2_cell)
        B2 = np.mat(B2_cell).T

        V = A1 * ini
        X_up = A2 * ini
        B2 = np.vstack((np.zeros([1, self.Tsim]), B2))
        X_up = np.vstack((s0, X_up))
        return A1, B1, A2, B2, V, X_up

    @staticmethod
    def gipps_cons(x, *args):
        V_ref, B1, A1, ini = args
        b1 = V_ref + 1 - np.matmul(A1, ini)
        final = b1 - np.matmul(B1, x).T
        return final.getA().squeeze()

    @staticmethod
    def bounds(x):
        return 2 - x

    @staticmethod
    def bounds2(x):
        return x + 2

    def cons_all(self, V_ref, A1, B1, ini, B2, X_up, LL_V=None, LL_X=None, Lf_V=None, Lf_X=None, LR_V=None,
                 LR_X=None):
        LL = 6
        b_safe = 6
        b_max = 2
        cons = []
        if self.control_vehicle == 'ramp' and Lf_X:
            AA2 = B2[-1, :]
            b2 = Lf_X - X_up[-1, 0] - LL
            AA3 = -B2[-1, :]
            b3 = X_up[-1, 0] - LL_X - LL
            cons.append({'type': 'ineq', 'fun': sqp_lower_control.cons5, 'args': (AA2, b2)})
        elif self.control_vehicle == 'leader':
            AA3 = B2[-1, :]
            b3 = LR_X - X_up[-1, 0] - LL
        else:
            AA3 = -B2[-1, :]
            b3 = X_up[-1, 0] - LR_X - LL
        cons.append({'type': 'ineq', 'fun': sqp_lower_control.cons6, 'args': (AA3, b3)})
        cons.append({'type': 'ineq', 'fun': sqp_lower_control.bounds})
        cons.append({'type': 'ineq', 'fun': sqp_lower_control.bounds2})
        if self.control_vehicle == 'ramp':
            if LL_V:
                constraints1 = {'type': 'ineq', 'fun': sqp_lower_control.cons1,
                                'args': (A1[-1, :], B1[-1, :], ini, b_max, X_up[-1, :], B2[-1, :], b_safe, LL_V, LL_X)}
                cons.append(constraints1)
            if Lf_V:
                constraints2 = {'type': 'ineq', 'fun': sqp_lower_control.cons2,
                                'args': (A1[-1, :], B1[-1, :], ini, b_max, X_up[-1, :], B2[-1, :], b_safe, Lf_V, Lf_X)}
                cons.append(constraints2)
        elif self.control_vehicle == 'leader':
            constraints1 = {'type': 'ineq', 'fun': sqp_lower_control.cons3,
                            'args': (A1[-1, :], B1[-1, :], ini, b_max, X_up[-1, :], B2[-1, :], b_safe, LR_V, LR_X)}
            cons.extend([constraints1])
        elif self.control_vehicle == 'follower':
            constraints1 = {'type': 'ineq', 'fun': sqp_lower_control.cons4,
                            'args': (A1[-1, :], B1[-1, :], ini, b_max, X_up[-1, :], B2[-1, :], b_safe, LR_V, LR_X)}
            cons.extend([constraints1])

        cons.append({'type': 'ineq', 'fun': sqp_lower_control.gipps_cons, 'args': (V_ref, B1, A1, ini)})
        return cons

    @staticmethod
    def obj(x, *args):
        A1, ini, B1, V_ref = args[0], args[1], args[2], args[3]
        result = np.array(
            np.matmul((A1 * ini + np.matmul(B1, x).T - V_ref).T, (A1 * ini + np.matmul(B1, x).T - V_ref)) + x.T.dot(x))
        return result[0, 0]

    @staticmethod
    def cons1(x, *args):
        "控制匝道车辆减速， LL前车"
        A1, B1, ini, b_max, X, B2, b_safe, LL_V, LL_X = args
        # A1(end,:) B1(end,:) X(end,1) B2(end,:)
        final = -((A1 * ini + np.matmul(B1, x)) + b_max - b_safe - np.sqrt(
            (b_safe ** 2) + LL_V ** 2 + 2 * b_max * (X + np.matmul(B2, x) - LL_X - 8)))
        return np.array(final[0, 0])

    @staticmethod
    def cons2(x, *args):
        "控制匝道车辆减速，LF后车"
        A1, B1, ini, b_max, X, B2, b_safe, Lf_V, Lf_X = args
        final = -(Lf_V + b_max - b_safe - np.sqrt(
            (b_safe ** 2) + (A1 * ini + np.matmul(B1, x)) ** 2 + 2 * b_max * (Lf_X - X + np.matmul(B2, x) - 8)))
        return np.array(final[0, 0])

    @staticmethod
    def cons3(x, *args):
        "控制主路前车，LR匝道车"
        A1, B1, ini, b_max, X, B2, b_safe, LR_V, LR_X = args
        final = -(LR_V + b_max - b_safe - np.sqrt((b_safe ** 2) + (A1 * ini + np.matmul(B1, x)) ** 2 + 2 * b_max * (
                LR_X - X + np.matmul(B2, x) - 8)))
        return np.array(final[0, 0])

    @staticmethod
    def cons4(x, *args):
        "控制主路后车"
        A1, B1, ini, b_max, X, B2, b_safe, LR_V, LR_X = args
        final = -((A1 * ini + np.matmul(B1, x)) + b_max - b_safe - np.sqrt(
            (b_safe ** 2) + LR_V ** 2 + 2 * b_max * (X + np.matmul(B2, x) - LR_X - 8)))
        return np.array(final[0, 0])

    @staticmethod
    def cons5(x, *args):
        AA2, b2 = args
        final = b2 - np.matmul(AA2, x)
        return final.getA().squeeze()

    @staticmethod
    def cons6(x, *args):
        AA3, b3 = args
        final = b3-np.matmul(AA3, x)
        return final.getA().squeeze()
