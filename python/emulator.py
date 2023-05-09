# Importing Libraries
import serial
import time
import json

import numpy as np
import matplotlib.pyplot as plt

from course import Course
from controllers.PP import PurePursuit_Controller
from controllers.MPC import MPC_Controller
from controllers.STAN import Stanley_Controller
from vehicle import Vehicle, Input, State
from blackbox import Blackbox, Entry
from arduino import Arduino

from parameters.vehicle_parameters import *
from parameters.simulation_parameters import *

class Emulator():
    def __init__(self, vehicle:Vehicle, arduino:Arduino):
        print("Emulator Initialized")

        self.vehicle = vehicle
        self.arduino = arduino

    def start(self):
        print("Starting Emulator")
        self._run()

    def _run(self):

        elapsed_time = 0.0
        self.vehicle.state.v = 1

        while True:
            arduino.request_state()
            # delta = arduino.state.delta
            # input_command = Input(delta=delta, accel=0)
            # input_command.normalize()
            input_command = Input()

            self.vehicle.update(input_command, 0.05)
            self.vehicle.state.yaw = arduino.state.yaw
            elapsed_time += 0.05

            plt.cla()
            fig = plt.gcf()
            ax = fig.gca()

            fig.set_size_inches((14, 8))

            fig.canvas.mpl_connect('key_release_event', self._live_press)
            
            plot_car(self.vehicle.state, plot_v=True)

            # Formatting
            plt.axis("equal")
            # plt.grid(True)
            plt.legend()
            plt.title("Time[s]:" + str(round(elapsed_time, 2)) +
                    ", speed[m/s]:" + str(round(self.vehicle.state.v, 2)))
            
            # Set the x-lim and y-lim
            plt.xlim([-2, 8])
            plt.ylim([-2, 6])

            # Update
            plt.pause(0.0001)

            time.sleep(0.01)

    def _live_press(self, event):
        if event.key == 'q':
            print("User terminated program mid-simulation")
            exit()


if __name__ == '__main__':
    arduino = Arduino()
    input("Type enter here")
    
    vehicle = Vehicle()

    em = Emulator(vehicle, arduino)
    em.start()