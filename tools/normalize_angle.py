import math

"""
Keeps angle between pi/2 and -pi/2
"""

def normalize_angle(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle