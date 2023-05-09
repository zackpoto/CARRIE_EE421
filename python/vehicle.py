import math
import numpy as np
from parameters.vehicle_parameters import *
from parameters.simulation_parameters import *

class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, d = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.delta = d

        self.rear_x = self.x - L2 * math.cos(self.yaw)
        self.rear_y = self.y - L2 * math.sin(self.yaw)

        self.front_x = self.x + L1 * math.cos(self.yaw)
        self.front_y = self.y + L1 * math.sin(self.yaw)

        self.R = 0
        self.S = 0
        self.beta = 0

    def keys(self):
        return np.array(['x', 'y', 'yaw', 'v', 'd'])
    
    def from_dict(self, iterable=(), **kwargs):
        self.__dict__.update(iterable, **kwargs)
    
    def to_array(self):
        return np.array([self.x, self.y, self.yaw, self.v, self.delta])
    
    def to_dict(self):
        return dict(zip(self.keys(), self.to_array()))
    
    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)

class Input:
    """"
    vehicle input signals
    """
    def __init__(self, delta=0.0, accel=0.0):
        self.delta = delta
        self.accel = accel

        self.keys = np.array(['d', 'a'])

    def normalize(self):
        self.delta = np.clip(self.delta, -MAX_STEER, MAX_STEER)
        self.accel = np.clip(self.accel, -MAX_ACCEL, MAX_ACCEL)

    def to_array(self):
        return np.array([self.delta, self.accel])
    
    def to_dict(self):
        return dict(zip(self.keys, self.to_array()))

class Vehicle:
    def __init__(self, state=State()):
        self.state = state
    
    def update(self, input, dt = DT):
        a = input.accel
        delta = input.delta
        delta = np.clip(delta, -MAX_STEER, MAX_STEER)

        self.state.delta = delta
        self.state.beta = np.arctan2(L2*np.tan(delta), WB)
        self.state.S = None if (delta == 0) else WB/np.tan(delta)
        self.state.R = None if (delta == 0) else self.state.S / np.cos(self.state.beta)
        
        self.state.x = self.state.x + self.state.v * math.cos(self.state.yaw + self.state.beta) * dt
        self.state.y = self.state.y + self.state.v * math.sin(self.state.yaw + self.state.beta) * dt
        self.state.v = self.state.v + a * dt
        self.state.yaw = self.state.yaw + self.state.v / WB * math.tan(delta) * math.cos(self.state.beta) * dt

        self.state.rear_x = self.state.x - L2 * math.cos(self.state.yaw)
        self.state.rear_y = self.state.y - L2 * math.sin(self.state.yaw)

        self.state.front_x = self.state.x + L1 * math.cos(self.state.yaw)
        self.state.front_y = self.state.y + L1 * math.sin(self.state.yaw)

        if self.state.v > MAX_SPEED:
            self.state.v = MAX_SPEED
        elif self.state.v < MIN_SPEED:
            self.state.v = MIN_SPEED