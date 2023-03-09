
import numpy as np
import matplotlib.pyplot as plt
import math

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from vehicle import *

# Vehicle parameters
NAME = "CARRIE"
LENGTH = 0.314          # [m]
WIDTH = 0.18            # [m]
BACKTOWHEEL = 0.0475    # [m]
WHEEL_LEN = 0.04        # [m]
WHEEL_WIDTH = 0.005     # [m]
TREAD = 0.23/2          # [m]
L1 = 0.07               # [m]
L2 = 0.15               # [m]
WB = L1+L2              # [m]

MAX_STEER = np.deg2rad(30.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 2.0  # maximum accel [m/ss]

def plot_car(state, plot_v=True, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    x = state.x
    y = state.y
    yaw = state.yaw
    steer = state.delta
    beta = state.beta
    v = state.v

    outline = np.array([[-BACKTOWHEEL-L2, (LENGTH - BACKTOWHEEL - L2), (LENGTH - BACKTOWHEEL - L2), -BACKTOWHEEL - L2, -BACKTOWHEEL - L2],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    velocity = np.array([[0, v * 0.5],    # scale the velocity vector to shortened for plotting
                         [0, 0]])

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])
    Rot3 = np.array([[math.cos(beta), math.sin(beta)],
                     [-math.sin(beta), math.cos(beta)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T

    fr_wheel[0, :] += L1
    fl_wheel[0, :] += L1
    rr_wheel[0, :] -= L2
    rl_wheel[0, :] -= L2

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    velocity = (velocity.T.dot(Rot1)).T
    velocity = (velocity.T.dot(Rot3)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    velocity[0, :] += state.x
    velocity[1, :] += state.y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor, linewidth = 0.5)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor, linewidth = 0.5)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor, linewidth = 0.5)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor, linewidth = 0.5)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor, linewidth = 0.5)
    plt.plot(x, y, "*")

    if plot_v:
        plt.arrow(velocity[0,0],
                  velocity[1,0],
                  velocity[0, 1] - velocity[0,0],
                  velocity[1,1] - velocity[1,0],
                  head_width=0.04, head_length=0.08, fc='b', ec='b', label='velocity')

if __name__ == '__main__':
    plt.close("all")
    plt.subplots()
    plot_car(State(0, 0, np.deg2rad(60), 2), MAX_STEER)
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.show()




    