"""chainplan Rotation Computation,"""
#2022 11-18
import numpy as np

class Low_Pass_Second_Order_Filter():
    def __init__(self, initial_value, stop_freq, damping_rate, time_gap):
        self.y         = initial_value
        self.y_plus_1  = initial_value
        self. y_plus_2 = initial_value
        self.omega_n = 2 * np.pi * stop_freq
        self.zeta    = damping_rate
        self.filter_gap = time_gap
        
    def march_forward(self, input):
        self. y =  self. y_plus_1
        self. y_plus_1 = self. y_plus_2
        self. y_plus_2 = (2 - 2 * self.zeta * self.omega_n * self.filter_gap ) * self.y_plus_1 +\
                            ( - self.omega_n * self.omega_n * self.filter_gap * self.filter_gap + \
                            2 * self.zeta * self.omega_n * self.filter_gap - 1 ) * self.y + \
                               self.omega_n * self.omega_n * self.filter_gap * self.filter_gap * input
    def Get_filtered (self): 
        return  self. y                          