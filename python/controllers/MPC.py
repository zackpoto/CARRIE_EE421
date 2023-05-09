
import cvxpy
from controllers.controller import *

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 2.0  # [m/s] target speed
N_IND_SEARCH = 10
T = 5  # horizon length

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]

class MPC_Controller():
    def __init__(self, state, course):
        self.name = "MPC"
        self.course = course

        self.odelta = None
        self.oa = None
        self.ox = []
        self.oy = []

        self.target_ind, self.min_d = self.course.calc_nearest_index(state.rear_x, state.rear_y, 0, N_IND_SEARCH)
        self.sp = self.calc_speed_profile(self.course.cx, self.course.cy, self.course.cyaw, TARGET_SPEED)
        self.xref = []

    def command(self, state):
        self.xref, target_ind, dref = self.calc_ref_trajectory(state)
        self.target_ind = target_ind
        x0 = [state.x, state.y, state.v, state.yaw] #TODO: make a function of State Class object

        self.oa, self.odelta, self.ox, self.oy, oyaw, ov = self.iterative_linear_mpc_control(self.xref, x0, dref, self.oa, self.odelta)

        return Input(self.odelta[0], self.oa[0])
    
    def plot_realtime(self):
        self.plot_o_pts()
        self.plot_xref()
        self.plot_target()

    def plot_o_pts(self):
        if self.ox is not None:
                plt.plot(self.ox, self.oy, "xr", label="MPC")
                
    def plot_xref(self):
        plt.plot(self.xref[0, :], self.xref[1, :], "xk", label="xref")

    def plot_target(self):
        plt.plot(self.course.cx[self.target_ind], self.course.cy[self.target_ind], "xg", label="target")

    def iterative_linear_mpc_control(self, xref, x0, dref, oa, od):
        """
        MPC contorl with updating operational point iteraitvely
        """

        if oa is None or od is None:
            oa = [0.0] * T
            od = [0.0] * T

        for i in range(MAX_ITER):
            xbar = self.predict_motion(x0, oa, od, xref)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= DU_TH:
                break
        else:
            print("Iterative is max iter")

        return oa, od, ox, oy, oyaw, ov
    
    def linear_mpc_control(self, xref, xbar, x0, dref):
        """
        linear mpc control

        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angle
        """

        x = cvxpy.Variable((NX, T + 1))
        u = cvxpy.Variable((NU, T))

        cost = 0.0
        constraints = []

        for t in range(T):
            cost += cvxpy.quad_form(u[:, t], R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

            A, B, C = self.get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                                UGV.MAX_DSTEER * SIM.DT]

        cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= UGV.MAX_SPEED]
        constraints += [x[2, :] >= UGV.MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= UGV.MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= UGV.MAX_STEER]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.ECOS, verbose=False)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = (x.value[0, :]).flatten()
            oy = (x.value[1, :]).flatten()
            ov = (x.value[2, :]).flatten()
            oyaw = (x.value[3, :]).flatten()
            oa = (u.value[0, :]).flatten()
            odelta = (u.value[1, :]).flatten()

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov
    
    def get_linear_model_matrix(self, v, phi, delta):

        A = np.zeros((NX, NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = SIM.DT * math.cos(phi)
        A[0, 3] = - SIM.DT * v * math.sin(phi)
        A[1, 2] = SIM.DT * math.sin(phi)
        A[1, 3] = SIM.DT * v * math.cos(phi)
        A[3, 2] = SIM.DT * math.tan(delta) / UGV.WB

        B = np.zeros((NX, NU))
        B[2, 0] = SIM.DT
        B[3, 1] = SIM.DT * v / (UGV.WB * math.cos(delta) ** 2)

        C = np.zeros(NX)
        C[0] = SIM.DT * v * math.sin(phi) * phi
        C[1] = - SIM.DT * v * math.cos(phi) * phi
        C[3] = - SIM.DT * v * delta / (UGV.WB * math.cos(delta) ** 2)

        return A, B, C

    def predict_motion(self, x0, oa, od, xref):
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        vehicle = Vehicle(State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2]))
        for (ai, di, i) in zip(oa, od, range(1, T + 1)):
            input = Input(di, ai)
            vehicle.update(input)
            xbar[0, i] = vehicle.state.x
            xbar[1, i] = vehicle.state.y
            xbar[2, i] = vehicle.state.v
            xbar[3, i] = vehicle.state.yaw

        return xbar
    
    def calc_ref_trajectory(self, state):

        pind = self.target_ind

        xref = np.zeros((NX, T + 1))
        dref = np.zeros((1, T + 1))
        ncourse = self.course.path_size

        ind, _ = self.course.calc_nearest_index(state.rear_x, state.rear_y, pind, N_IND_SEARCH)

        if pind >= ind:
            ind = pind

        xref[0, 0] = self.course.cx[ind]
        xref[1, 0] = self.course.cy[ind]
        xref[2, 0] = self.sp[ind]
        xref[3, 0] = self.course.cyaw[ind]
        dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(T + 1):
            travel += abs(state.v) * SIM.DT
            dind = int(round(travel / self.course.dl))

            if (ind + dind) < ncourse:
                xref[0, i] = self.course.cx[ind + dind]
                xref[1, i] = self.course.cy[ind + dind]
                xref[2, i] = self.sp[ind + dind]
                xref[3, i] = self.course.cyaw[ind + dind]
                dref[0, i] = 0.0
            else:
                xref[0, i] = self.course.cx[ncourse - 1]
                xref[1, i] = self.course.cy[ncourse - 1]
                xref[2, i] = self.sp[ncourse - 1]
                xref[3, i] = self.course.cyaw[ncourse - 1]
                dref[0, i] = 0.0

        return xref, ind, dref
    
    def calc_speed_profile(self, cx, cy, cyaw, target_speed):
        speed_profile = [target_speed] * len(cx)
        direction = 1.0  # forward

        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]

            move_direction = math.atan2(dy, dx)

            if dx != 0.0 and dy != 0.0:
                dangle = abs(normalize_angle(move_direction - cyaw[i]))
                if dangle >= math.pi / 4.0:
                    direction = -1.0
                else:
                    direction = 1.0

            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

        speed_profile[-1] = 0.0
        return speed_profile
    
    def export_params(self):
        return[R, Rd]
    
