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

from parameters.vehicle_parameters import *
from parameters.simulation_parameters import *

class Arduino():
    def __init__(self):
        self.arduino = serial.Serial(port='/dev/cu.usbmodem101', baudrate=115200, timeout=.01)
        self.state = State()

    def update(self, input:Input):
        self.write_command(np.rad2deg(input.delta), input.accel)
        delta = self.read_command()

        self.state.x += 0.01
        if delta != None:
            self.state.d = np.deg2rad(delta)

    def read_command(self):
        data = self.arduino.readline()
        string = data.decode()
        if string != None and string != "":
            newState = State()
            dict = json.loads(string)
            print(dict)
            newState.from_dict(dict)
            newState.delta = np.deg2rad(newState.delta)
            newState.yaw = np.deg2rad(newState.yaw)
            self.state = newState

    def write_command(self, delta, velocity):
        command = "<delta,%d,velocity,%d>" % (delta, velocity)
        self.arduino.write(command.encode('ascii'))
    
    def request_state(self):
        packet = "<REQUEST>"
        self.arduino.write(packet.encode('ascii'))
        self.read_command()

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