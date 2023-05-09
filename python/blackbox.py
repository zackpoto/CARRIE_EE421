import numpy as np
import matplotlib.pyplot as plt
from course import Course
from parameters.vehicle_parameters import *

import importlib
import sys
    
class Entry():
    def __init__(self, t, **kwargs):
        self.dictionary = dict()
        for key in kwargs.keys():
            self.dictionary.update(kwargs[key].to_dict())
        self.dictionary['t'] = t

class Blackbox():
    def __init__(self, course, entry):
        self.course = course
        self.values = dict()
        self.runtime = None
        for key in entry.dictionary.keys():
            self.values[key] = [entry.dictionary[key]]

    def add_entry(self, entry):
        for key in entry.dictionary.keys():
            self.values[key].append(entry.dictionary[key])

    def plot_trajectory_points(self):
        x, y, yaw = np.array(self.values['x']), np.array(self.values['y']), np.array(self.values['yaw'])
        plt.plot(x, y, "ob", label="trajectory")

    def plot_paths(self, coursePlot = True, 
                         trajectoryPlot = True, 
                         sweepArea = False, 
                         sweepOutline = False):

        x, y, yaw = np.array(self.values['x']), np.array(self.values['y']), np.array(self.values['yaw'])

        leftx = x + L1*np.cos(yaw) - TREAD*np.sin(yaw)
        rightx = x + L1*np.cos(yaw) + TREAD*np.sin(yaw)
        lefty = y + L1*np.sin(yaw) + TREAD*np.cos(yaw)
        righty = y + L1*np.sin(yaw) - TREAD*np.cos(yaw) 

        plt.subplots()
        if coursePlot: self.course.plot_course()
        if trajectoryPlot: plt.plot(x, y, "-g", label="trajectory")
        if sweepArea: plt.fill(np.append(leftx, rightx[::-1]), np.append(lefty, righty[::-1]), 'y', alpha=0.25, label='outline')
        if sweepOutline: plt.plot(leftx, lefty, 'y', label='left')
        if sweepOutline: plt.plot(rightx, righty, 'y', label='right')
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

    def plot_velocity(self):
        plt.subplots()
        plt.plot(self.values['t'], self.values['v'], "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [kmh]")

    def plot_delta(self):
        plt.subplots()
        plt.plot(self.values['t'], self.values['d'])
        plt.xlabel("Time [s]")
        plt.ylabel("Steering Angle [rad]")

    def plot_acceleration(self):
        plt.subplots()
        plt.plot(self.values['t'], self.values['a'])
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Accel [m/s/s]")

    def plot_lateral_acceleration(self, wheelbase):
        v = np.array([self.values['v']])
        d = np.array([self.values['d']])
        lat_accel = (d*v**2).T/wheelbase

        plt.subplots()
        plt.plot(self.values['t'], lat_accel)
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Lateral Accel. [m/s/s]")

    def plot_course_error(self, dt = 2):
        x, y = np.array(self.values['x']), np.array(self.values['y'])
        cx, cy = np.array(self.course.cx), np.array(self.course.cy)

        DX = np.empty((x.size, cx.size))
        for (i, xi) in enumerate(x):
            for (j, cxj) in enumerate(cx):
                DX[i, j] = (xi - cxj)

        DY = np.empty((y.size, cy.size))
        for (i, yi) in enumerate(y):
            for (j, cyj) in enumerate(cy):
                DY[i, j] = (yi - cyj)

        DX2 = DX**2
        DY2 = DY**2
        D2 = DX2 + DY2

        mins = D2.argmin(axis=1)

        x_coords = []
        y_coords = []
        for (m, n) in enumerate(mins):
            post = D2[m, n+1]
            prev = D2[m, n-1]

            if post == prev:
                x_coords.append()
                y_coords.append()
                break
            elif post < prev:
                nn = n+1
            elif prev < post:
                nn = n-1
            else:
                print("theres an error here")

            x1, y1 = cx[n], cy[n]
            x2, y2 = cx[nn], cy[nn]
            x3, y3 = x[m], y[m]
            dx, dy = x2-x1, y2-y1

            det = dx*dx + dy*dy
            a = (dy*(y3-y1)+dx*(x3-x1))/det
            x_coords.append(x1+a*dx)
            y_coords.append(y1+a*dy)

        D2_mins = [D2[i, j] for (i,j) in enumerate(mins)]
        D_mins = np.sqrt(D2_mins)
        D_max = max(D_mins)
        D_sum = np.sum(D_mins)

        plt.subplots()
        for (m, n) in enumerate(mins):
            plt.plot([x[m], x_coords[m]], [y[m], y_coords[m]], color = 'b', marker = '.')
        plt.plot(x, y, 'g')
        plt.plot(cx, cy, 'r')
        plt.axis("equal")
        plt.grid(True)
    
    def export(self, controller):

        export_dict = {
            "cx": self.course.cx,
            "cy": self.course.cy,
            "cyaw": self.course.cyaw,
            "ck": self.course.ck,
            "dl": [self.course.dl],
            "t": self.values["t"],
            "x": self.values["x"],
            "y": self.values["y"],
            "yaw": self.values["yaw"],
            "v": self.values["v"],
            "delta": self.values["d"],
            "accel": self.values["a"],
        }

        print("Exporting Blackbox")

        filename =  "saved_simulations/" + \
                    controller.name + "-" + \
                    self.course.name + "-" + \
                    ("DNF" if self.runtime is None else ("%3.3f" % self.runtime).replace(".", "_") ) + \
                    ".py"
        
        controller_params = controller.export_params()

        with open(filename, 'w') as f:
            # write the controller parameters in the comment heading
            f.write('"""\n')
            f.write("CONTROLLER PARAMS\n")
            for key in controller_params:
                f.write(key + ": " + str(controller_params[key]) + "\n")
            f.write('"""\n')

            # write the blackbox simulation data (time, state, input)
            f.write('#---DATA---\n')
            for key in export_dict:
                f.write(key + "=[")
                for thing in export_dict[key]:
                    f.write(str(thing) + ",")
                f.write("]\n")

        print("Successfully exported blackbox to %s" %filename)

# Run this file (python3 blackbox.py 'blackboxfilename.py') to view a previous simulation in 'saved_simulations' folder
def entry_i(ind):
    entry = Entry(t = mod.t[ind])
    entry.dictionary["x"] = mod.x[ind]
    entry.dictionary["y"] = mod.y[ind]
    entry.dictionary["yaw"] = mod.yaw[ind]
    entry.dictionary["v"] = mod.v[ind]
    entry.dictionary["d"] = mod.delta[ind]
    entry.dictionary["a"] = mod.accel[ind]
    return entry

if __name__ == '__main__':
    mod = importlib.import_module("saved_simulations." + sys.argv[1])

    course = Course(mod.dl[0], coords=[mod.cx, mod.cy, mod.cyaw, mod.ck])
    course.create_course_plot()

    entry0 = entry_i(0)
    blackbox = Blackbox(course, entry0)

    for i in range(1, len(mod.t)):
        entryi = entry_i(i)
        blackbox.add_entry(entryi)