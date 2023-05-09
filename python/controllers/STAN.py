from controllers.controller import *

k = 3.0  # control gain

k_soft = 0.0 # softening coefficient
k_damp = 1.0 # steering damping coefficient

Kp = 1.0  # speed proportional gain

TARGET_SPEED = 2.0  # [m/s] target speed

class Stanley_Controller(Controller):
    def __init__(self, state:State, course:Course):
        self.name = "STAN"
        self.course = course

        self.target_ind, self.min_d = self.course.calc_nearest_index(state.front_x, state.front_y, 0, 5)

    def command(self, state:State):
        delta = self.steering_control(state)
        accel = self.throttle_control(state, TARGET_SPEED)
        return Input(delta, accel)
    
    def plot_realtime(self):
        self.plot_target()
    
    def plot_target(self):
        plt.plot(self.course.cx[self.target_ind], self.course.cy[self.target_ind], "xg", label="target")

    def steering_control(self, state:State):
        self.target_ind, self.min_d = self.course.calc_nearest_index(state.front_x, state.front_y, self.target_ind, 5)
        ind = self.target_ind

        tx, ty = self.course.cx[ind], self.course.cy[ind]

        error_front_axle = self.error_front_axle(state, tx, ty)
        theta_e = normalize_angle(self.course.cyaw[ind] - state.yaw)
        theta_d = np.arctan2(k_soft + k * error_front_axle, k_damp*state.v)

        delta = theta_e + theta_d
        return delta

    def throttle_control(self, state:State, target_speed):
        return Kp * (target_speed - state.v)

    def error_front_axle(self, state:State, tx, ty):
        fx = state.front_x
        fy = state.front_y

        front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                          -np.sin(state.yaw + np.pi / 2),
        ]

        error = np.dot([fx-tx, fy-ty], front_axle_vec)

        return error