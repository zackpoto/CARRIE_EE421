import matplotlib.pyplot as plt
import math
import numpy as np

from vehicle import State, Input, Vehicle
import parameters.simulation_parameters as SIM
import parameters.vehicle_parameters as UGV
from course import Course
from tools.normalize_angle import normalize_angle

class Controller():
    def __init__(self, state: State, course: Course):
        self.name = "NONE"

    def command(self, state:State):
        return Input(0.0, 0.0)
    
    def plot_realtime(self):
        pass

    def export_params(self):
        return []