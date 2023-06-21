###"""chainplan Rotation Computation,"""
#2022 11-18
import numpy as np

class Low_Pass_Second_Order_Filter():
    def __init__(self, initial_value, stop_freq, damping_rate, time_gap):
        self.y         = initial_value
        self.y_minus_1  = initial_value
        self.y_minus_2  = initial_value
        self.input_minus_1 = initial_value
        self.input_minus_2 = initial_value

        self.omega_n = 2 * np.pi * stop_freq
        self.zeta    = damping_rate
        self.filter_gap = time_gap
        
    def march_forward(self, input):
        A = 2.0 * (self.zeta * self.omega_n * self.filter_gap - 1)
        B = 1.0 - 2.0 * self.zeta * self.omega_n * self.filter_gap + \
            self.omega_n * self.filter_gap * self.omega_n * self.filter_gap
        C = self.omega_n * self.filter_gap * self.omega_n * self.filter_gap
        # self.y = C * self. input_minus_2 - A * self. y_minus_1 - B *  self. y_minus_2
        # print('A',A, 'B', B, 'C', C)
        self.y = C * self.input_minus_2 - A   * self.y_minus_1 - B * self.y_minus_2
        # print('self.u : ', input[0,0], self.input_minus_1 [0,0], self.input_minus_2[0,0])
        # print('self.y : ', self.y[0,0], self.y_minus_1[0,0],  self.y_minus_2[0,0])
        self.input_minus_2 = self.input_minus_1
        self.input_minus_1 = input
        self.y_minus_2 = self.y_minus_1
        self.y_minus_1 = self.y


    def Get_filtered (self): 
        return  self. y                          