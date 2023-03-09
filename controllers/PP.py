
from controllers.controller import *

TARGET_SPEED = 2.0  # [m/s] target speed

k = 0.1  # look forward gain
Lfc = 0.4  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain

class PurePursuit_Controller(Controller):
    def __init__(self, state: State, course: Course):
        self.name = "PP"
        self.course = course

        self.nearest_ind, self.min_d = self.course.calc_nearest_index(state.rear_x, state.rear_y, 0, 10)
        self.target_ind, self.Lf = self.search_target_index(state)

    def command(self, state: State):
        delta = self.steering_control(state)
        accel = self.throttle_control(state, TARGET_SPEED)
        return Input(delta, accel)

    def plot_realtime(self):
        self.plot_nearest()
        self.plot_target()

    def plot_nearest(self):
        plt.plot(self.course.cx[self.nearest_ind], self.course.cy[self.nearest_ind], "xb", label="nearest")

    def plot_target(self):
        plt.plot(self.course.cx[self.target_ind], self.course.cy[self.target_ind], "xg", label="target")

    def steering_control(self, state: State):
        pind = self.target_ind
        plf = self.Lf
        ind, Lf = self.search_target_index(state)

        if ind < pind:
            ind = pind
            Lf = plf

        x = state.rear_x
        y = state.rear_y

        tx, ty = self.course.cx[ind], self.course.cy[ind]
        alpha = math.atan2(ty - y, tx - x) - state.yaw
        delta = math.atan2(2.0 * UGV.WB * math.sin(alpha) / Lf, 1.0)

        self.target_ind = ind
        self.Lf = Lf
        return delta

    def throttle_control(self, state: State, target_speed):
        return Kp * (target_speed - state.v)

    def search_target_index(self, state: State):
        self.nearest_ind, self.min_d = self.course.calc_nearest_index(state.rear_x, 
                                                                      state.rear_y, 
                                                                      self.nearest_ind, 
                                                                      20)
        
        Lf = k * state.v + Lfc

        # search look ahead target point index
        ind = self.nearest_ind
        while Lf > state.calc_distance(self.course.cx[ind], self.course.cy[ind]):
            if (ind + 1) >= self.course.path_size:
                break  # not exceed goal
            ind += 1

        return ind, Lf

    def export_params(self):
        return {'k': k,
                'Lfc': Lfc,
                'Kp': Kp,
                'Target Speed': TARGET_SPEED}