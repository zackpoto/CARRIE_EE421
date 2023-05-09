
from matplotlib import pyplot as plt, patches
import cvxpy
import math
import numpy as np
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

from course import Course
from controllers.PP import PurePursuit_Controller
from controllers.MPC import MPC_Controller
from controllers.STAN import Stanley_Controller
from vehicle import Vehicle, Input, State
from blackbox import Blackbox, Entry

from parameters.vehicle_parameters import *
from parameters.simulation_parameters import *

GOAL_DIS = 0.25         # goal distance
STOP_SPEED = 1 / 3.6    # stop speed

show_animation = True

class Simulator():
    def __init__(self, course, vehicle:Vehicle, controller):
        print("Simulator Initialized")

        self.course = course
        self.vehicle = vehicle
        self.controller = controller
        
        self.blackbox = None
        self.saved = False

    def start(self):
        print("Starting Simulation! \nPress 'q' to quit.")
        self.blackbox = self._run_simulation(self.course, self.vehicle, self.controller)
        time = self.blackbox.runtime
        if time is None:
            print("Simulation DNF")
        else:
            print("Simulated ended with time of: %5.4f" %time)

        # post-simulation plotting
        if show_animation:
            plt.close("all")

            self.blackbox.plot_paths(sweepArea=True, sweepOutline=False)

            fig = plt.gcf()
            fig.canvas.mpl_connect('key_release_event', self.processing_press)
            print("Press 'Shift+S' to save the blackbox for this simulation")
            plt.show()

            self.blackbox.plot_velocity()
            self.blackbox.plot_acceleration()
            self.blackbox.plot_delta()
            self.blackbox.plot_lateral_acceleration(WB)
            self.blackbox.plot_course_error()

            plt.show()

    def _run_simulation(self, course, vehicle:Vehicle, controller):
        """
        Simulation
        """
        time = 0.0

        # create a blackbox (sim-record) with an initial entry and course path
        blackbox = Blackbox(course, Entry(t = 0.0, input = Input(), state = vehicle.state))

        while MAX_TIME >= time:

            # Generate course controller update based on current vehicle state
            input_command = controller.command(vehicle.state)
            input_command.normalize()

            # Update vehicle state based on the new input
            vehicle.update(input_command)
             
            
            # Add new vehicle state to the blackbox
            blackbox.add_entry(Entry(t = time, input = input_command, state = vehicle.state))

            # check if vehicle is close enough to the endpoint
            if course.check_goal(vehicle.state, controller.target_ind, GOAL_DIS):
                blackbox.runtime = time
                break

            # real-time simulation animation
            if show_animation:  # pragma: no cover
                plt.cla()
                fig = plt.gcf()
                ax = fig.gca()

                fig.set_size_inches((14, 8))
                # for stopping simulation with the esc key.
                fig.canvas.mpl_connect('key_release_event', self.simulation_press)

                # Finish line bubble
                ax.add_patch(patches.Circle(course.goal, GOAL_DIS, fill = True, color = 'r', alpha=0.4))

                # Course plotting
                ax.plot(course.cx, course.cy, "-r", label="course")

                # Trajectory plotting
                # blackbox.plot_trajectory_points()
                controller.plot_realtime()
                plot_car(vehicle.state, plot_v=True)

                # Formatting
                plt.axis("equal")
                # plt.grid(True)
                plt.legend()
                plt.title("Time[s]:" + str(round(time, 2)) +
                        ", speed[m/s]:" + str(round(vehicle.state.v, 2)))
                
                # Set the x-lim and y-lim
                plt.xlim([-2, 8])
                plt.ylim([-2, 6])

                # Update
                plt.pause(0.0001)

        return blackbox

    def simulation_press(self, event):
        if event.key == 'q':
            print("User terminated program mid-simulation")
            exit()

    def processing_press(self, event):
        if event.key == 'S':
            print("Saving blackbox for simulation")
            if not self.saved:
                self.blackbox.export(self.controller)
                self.saved = True


def main():
    dl = 0.1  # course tick
    course = Course(dl)
    course.smooth_cyaw()
    vehicle = Vehicle()

    # controller = MPC_Controller(vehicle.state, course)
    # controller = PurePursuit_Controller(vehicle.state, course)
    controller = Stanley_Controller(vehicle.state, course)

    sim = Simulator(course, vehicle, controller)
    sim.start()

if __name__ == '__main__':
    main()
