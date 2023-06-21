### """chainplan Observer,"""
### by Chainplain 2022/11/30
### most variable if possible are in the form of np.matrix
import numpy as np
from   scipy.spatial.transform import Rotation

class Finite_time_slide_mode_observer_3dim():  
    def __init__(self, p_init, v_init, z_init, \
                   rho_e_SET, G_p_SET,  G_v_SET, G_z_SET, time_gap, robot_mass, g = 9.8):
        self. p_observer = p_init
        self. v_observer = v_init
        self. z_observer = z_init
        self. rho_e = rho_e_SET
        self. G_p = G_p_SET
        self. G_v = G_v_SET
        self. G_z = G_z_SET
        self. observer_gap = time_gap
        self. m = robot_mass
        self. e_3 = np.mat([[0],[0],[1]])
        self. gravitional_accel = g
        
    def sigma_map(self, super, vector):
        signal = np.sign(vector)
        module = np.multiply( np.abs(vector), np. power(vector, super) )
        return np.multiply(signal, module)
        
    def march_forward(self, u_t, p_real):
        p_observer_d = self. v_observer \
            - self. G_p * self.sigma_map((self. rho_e + 1)/2, self. p_observer - p_real) 
        v_observer_d = 1 / self. m * u_t - self. gravitional_accel * self. e_3 \
            - 1 / self. m * self. G_v * self.sigma_map((self. rho_e + 1)/2, self. p_observer - p_real) \
            + 1 / self. m * self. z_observer
        z_observer_d = - self. G_p * self.sigma_map(self. rho_e, self. p_observer - p_real)
        
        self. p_observer = self. p_observer + self. observer_gap * p_observer_d
        self. v_observer = self. v_observer + self. observer_gap * v_observer_d           
        self. z_observer = self. z_observer + self. observer_gap * z_observer_d   