
import math
import numpy as np
import matplotlib.pyplot as plt

from PathPlanning.CubicSpline import cubic_spline_planner

from tools.normalize_angle import normalize_angle

class Course(): 
    def __init__(self, dl, coords = []):
        self.dl = dl
        if len(coords) == 0:
            self.name, self.cx, self.cy, self.cyaw, self.ck = self.get_forward_course(dl)
        else:
            self.name = "UNKNOWN"
            self.cx = coords[0]
            self.cy = coords[1]
            self.cyaw = coords[2]
            self.ck = coords[3]

        self.path_size = len(self.cx)
        self.goal = [self.cx[-1], self.cy[-1]]
        self.course_length = self.get_course_length(self.cx, self.cy)

    def coords(self):
        return self.cx, self.cy, self.cyaw, self.ck
    
    def check_goal(self, state, controller_target_ind, goal_dis):
        dx = state.x - self.goal[0]
        dy = state.y - self.goal[1]
        d = math.hypot(dx, dy)

        isgoal = (d <= goal_dis)
        if abs(self.path_size - controller_target_ind) >= 5:
            isgoal = False

        return isgoal
    
    def calc_nearest_index(self, x, y, pind, search_horizon):

        cx = self.cx
        cy = self.cy
        cyaw = self.cyaw

        dx = [x - icx for icx in cx[pind:(pind + search_horizon)]]
        dy = [y - icy for icy in cy[pind:(pind + search_horizon)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        mind = math.sqrt(mind)

        dxl = cx[ind] - x
        dyl = cy[ind] - y

        angle = normalize_angle(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind
    
    # Get a course
    def get_forward_course(self, dl):
        ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
        ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, 20.0]
        ax = [x / 20 for x in ax]
        ay = [y/ 20 for y in ay]
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
            ax, ay, ds=dl)

        return "COURSE_A", cx, cy, cyaw, ck

    # Get another course
    def get_switch_back_course(self, dl):
        ax = [0.0, 3.0, .60, 2.00, 3.50]
        ay = [0.0, 0.0, 2.00, 3.50, 2.00]
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
            ax, ay, ds=dl)
        ax = [3.50, 1.00, 0.0, 0.0]
        ay = [2.00, 3.00, 0.5, 0.0]
        cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
            ax, ay, ds=dl)
        cyaw2 = [i - math.pi for i in cyaw2]
        cx.extend(cx2)
        cy.extend(cy2)
        cyaw.extend(cyaw2)
        ck.extend(ck2)

        return "COURSE_B", cx, cy, cyaw, ck
    
    def smooth_cyaw(self):
        yaw = self.cyaw
        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]

            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

        self.cyaw = yaw

    def differential_length(self, cx, cy):
        dx = [0] + [cx[i+1] - cx[i] for i in range(len(cx)-1)]
        dy = [0] + [cy[i+1] - cy[i] for i in range(len(cy)-1)]
        d2 = [(dx[i]**2 + dy[i]**2) for i in range(len(dx))]
        d = [np.sqrt(d2i) for d2i in d2]

        return d

    def get_course_length(self, cx, cy):
        length = sum(self.differential_length(cx, cy))
        return length
    
    def distance_traveled(self, cx, cy):
        d = self.differential_length(cx, cy)
        int_d = [sum(d[:i]) for i in range(len(d))]
        return int_d
    
    def create_course_plot(self):
        plt.subplots()
        self.plot_course()
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.show()

    def plot_course(self):
        plt.plot(self.cx, self.cy, "-r", label="course")

    def plot_curvature(self):
        plt.plot(self.distance_traveled(self.cx, self.cy), self.ck)
