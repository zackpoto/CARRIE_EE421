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