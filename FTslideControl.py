### """chainplan Observer,"""
### by Chainplain 2022/11/30
### most variable if possible are in the form of np.matrix
import numpy as np
from   scipy.spatial.transform import Rotation

EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION = 0.00000000001

class Finite_time_slide_mode_observer_3dim():  
    def __init__(self, robot_mass, p_init = np.mat([[0],[0],[0]]) ,\
                 v_init = np.mat([[0],[0],[0]]), z_init = np.mat([[0],[0],[0]]),\
                 time_gap = 0.001, grav = 9.8):
        self. p_observer = p_init
        self. v_observer = v_init
        self. z_observer = z_init
        self. rho_e = 0.5
        self. G_p = 5 * np.matrix([[1,0,0],\
                               [0,1,0],\
                               [0,0,1]])
        self. G_v = 10 * np.matrix([[1,0,0],\
                               [0,1,0],\
                               [0,0,1]])
        self. G_z = 20 * np.matrix([[1,0,0],\
                               [0,1,0],\
                               [0,0,1]])
        self. observer_gap = time_gap
        self. m = robot_mass
        self. e_3 = np.mat([[0],[0],[1]])
        self. gravitional_accel = grav
        
        
        
    def sigma_map(self, super, vector):
        signal = np.sign(vector)
        module = np. power(np.abs(vector), super) 
        return np.multiply(signal, module)
        
    def march_forward(self, u_t, p_real):
        # print ('self. rho_e:',self. rho_e)
        # print ('self. p_observer - p_real', self. p_observer - p_real)
        # print ('self.sigma_map((self. rho_e + 1)/2, self. p_observer - p_real)',\
                # self.sigma_map((self. rho_e + 1)/2, self. p_observer - p_real))
        p_observer_d = self. v_observer \
            - self. G_p * self.sigma_map((self. rho_e + 1)/2, self. p_observer - p_real)
        v_observer_d = 1 / self. m * u_t - self. gravitional_accel * self. e_3 \
            - 1 / self. m * self. G_v * self.sigma_map((self. rho_e + 1)/2, self. p_observer - p_real)\
            + 1 / self. m * self. z_observer
        z_observer_d = - self. G_p * self.sigma_map(self. rho_e, self. p_observer - p_real)
        
        self. p_observer = self. p_observer + self. observer_gap * p_observer_d
        self. v_observer = self. v_observer + self. observer_gap * v_observer_d           
        self. z_observer = self. z_observer + self. observer_gap * z_observer_d 
        
class Positional_Traj_Track_Controller():
    def __init__(self, robot_mass, g = 9.8):
    #### First controller parameters ####
        self. K_s = 1 * np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. K_p = 0.8 * np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. K_v  = 0.5 * np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. K_ev = 0.5 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. K_ep = 10 * np.matrix([[1,0,0],\
                                     [0,1,0],\
                                     [0,0,1]])
        self. K_ip = 0.01 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. c_s = 2
        self. rho_s = 0.5
        
        self. MaxInt = 2
        
    #### Second controller parameters ####
        self. K_ev_1 = 1 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. K_ep_1 = 10 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. K_p_1  = 0.8 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])

    #### Third controller parameters ####      
        self. K_ev_2 = 0.5 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. K_ep_2 = 8 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. K_v_2  = 0.5 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. K_s_2 = 0.5 *    np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
                
                
        
        
        self. e_3 = np.mat([[0],[0],[1]])
        self. gravitional_accel = g
        self. m = robot_mass
        self. PIntegration =  np.matrix([[0],\
                                       [0],\
                                       [0]])
        
        
    def sigma_map(self, super, vector):
        signal = np.sign(vector)
        module = np. power(np.abs(vector), super)
        return np.multiply(signal, module)
                
    def Calc_u_t(self, p_d, p_hat, v_d, v_hat, d_v_d, z_hat, R_now):
        s = self. c_s * np. tanh(self. K_p * (p_d - p_hat) ) + self. K_v * (v_d - v_hat) 
        self. PIntegration = self. PIntegration + self. K_ip * R_now.T * (v_d - v_hat) + 3* self. K_ip * R_now.T * (p_d - p_hat)
        print('self. PIntegration', self. PIntegration)
        if (self. PIntegration [0,0] > self. MaxInt ):
            self. PIntegration [0,0] = self. MaxInt 
        if (self. PIntegration [0,0] < -self. MaxInt ):
            self. PIntegration [0,0] = -self. MaxInt 
            
        if (self. PIntegration [1,0] > self. MaxInt ):
            self. PIntegration [1,0] = self. MaxInt 
        if (self. PIntegration [1,0] < -self. MaxInt ):
            self. PIntegration [1,0] = -self. MaxInt 
            
        if (self. PIntegration [2,0] > self. MaxInt ):
            self. PIntegration [2,0] = self. MaxInt 
        if (self. PIntegration [2,0] < -self. MaxInt ):
            self. PIntegration [2,0] = -self. MaxInt 
        u_feedB = self. K_s * self.sigma_map(self. rho_s, s) +  self. K_ev * (v_d - v_hat)\
                 + self. K_ep * np. tanh(self. K_p * (p_d - p_hat) ) + R_now * self. PIntegration
        u_feedF = d_v_d + self. gravitional_accel * self. e_3 - 1 / self. m * z_hat 
        u_t = u_feedF + u_feedB  
        return u_t
    
    def Calc_u_t_1(self, p_d, p_hat, v_d, v_hat, d_v_d, z_hat, R_now):
        u_feedB =  self. K_ev_1 * (v_d - v_hat) + self. K_ep_1 * np. tanh(self. K_p_1 * (p_d - p_hat) ) 
        u_feedF = d_v_d + self. gravitional_accel * self. e_3
        u_t_1 = u_feedF + u_feedB
        return u_t_1
        
    def Calc_u_t_2(self, p_d, p_hat, v_d, v_hat, d_v_d, z_hat, R_now):
        s = np. sign( p_d - p_hat  + self. K_v_2 * (v_d - v_hat) )
        u_feedB =  self. K_s_2 * s + self. K_ev_1 * (v_d - v_hat) + self. K_ep_2 * ( p_d - p_hat ) 
        u_feedF = d_v_d + self. gravitional_accel * self. e_3
        u_t_2 = u_feedF + u_feedB
        return u_t_2 
    
def hat_map(R3vector):
        # from R^3 → a 3x3 skew-symmetric matrix
    so3matrix = np.matrix([[0.0,            -R3vector[2,0], R3vector[1,0] ],
                           [R3vector[2,0],  0.0,            -R3vector[0,0]],
                           [-R3vector[1,0], R3vector[0,0],  0.0          ]])
    return so3matrix

    
        
def Computing_desired_rotation( u_t, T_d, F_d):
         u_t_normalized = 1 / (np.linalg.norm(u_t) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)\
                         * u_t
         e_3 = np. mat([[0],[0],[1]])
         
         k = hat_map( e_3 ) * u_t_normalized
         
         c_in_mat = e_3.T * u_t_normalized
         c = c_in_mat[0,0]
         
         s = np.linalg.norm(k)
         
         R_d_back_z = np.eye(3) + hat_map(k) + (1 - c) / (s * s + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION) * hat_map(k) * hat_map(k)
         
         
         T_d_normalized = 1 / (np.linalg.norm(T_d) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)\
                         * T_d
         F_d_normalized = 1 / (np.linalg.norm(F_d) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)\
                         * F_d
         # print('T_d_normalized',T_d_normalized)
         k_z =  hat_map( T_d_normalized ) * F_d_normalized 
         # print('k_z',k_z)
         c_in_mat_z = T_d_normalized.T * F_d_normalized
         c_z = c_in_mat_z[0,0]         
         
         s_z = np.linalg.norm(k_z)     
         R_z = np.eye(3) + hat_map(k_z) + (1 - c_z) / (s_z * s_z + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION) * hat_map(k_z) * hat_map(k_z)
         
         R_d = R_d_back_z * R_z
         # print('R_d',R_d)
         return R_d
         
class Attitude_reference_generator():
    def __init__(self, A_time_gap = 0.01, R_time_gap = 0.001,):
        self. R_f =     np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. Omega_f = np.matrix([[0],[0],[0]])
        self. K_wf    = 8 * np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. k_Rf    = 120
        self. G_f     =  np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. generator_gap_AV = A_time_gap
        self. generator_gap_rotation = R_time_gap
                                   
    def vee_map(self, so3matrix):
        # from so(3) → R^3, well, is the inverst of the hat_map
        R3vector = np.matrix([[ 0.5 * ( so3matrix[2,1] - so3matrix[1,2]) ],
                              [ 0.5 * ( so3matrix[0,2] - so3matrix[2,0]) ],
                              [ 0.5 * ( so3matrix[1,0] - so3matrix[0,1])]])
        return R3vector
        
    def hat_map(self, R3vector):
        # from R^3 → a 3x3 skew-symmetric matrix
        so3matrix = np.matrix([[0.0,            -R3vector[2,0], R3vector[1,0] ],
                               [R3vector[2,0],  0.0,            -R3vector[0,0]],
                               [-R3vector[1,0], R3vector[0,0],  0.0          ]])
        return so3matrix
    
    def match_forward_angular_velcoity(self, R_d):
        e_R_f     = 0.5 * self. vee_map(self. G_f * R_d.T * self. R_f - self. R_f.T * R_d * self. G_f) 
        d_Omega_f = - self. K_wf * self. Omega_f - self. k_Rf * e_R_f 
        self. Omega_f = self. Omega_f + self. generator_gap_AV * d_Omega_f  
   
    def match_forward_rotation(self, R_d):      
        d_R_f     = self. R_f *  self. hat_map(  self. Omega_f) 
        self. R_f = self. R_f + self. generator_gap_rotation * d_R_f 
        R_x = np.matrix([[ self. R_f[0,0]],
                         [ self. R_f[1,0]],
                         [ self. R_f[2,0]]])
        R_y = np.matrix([[ self. R_f[0,1]],
                         [ self. R_f[1,1]],
                         [ self. R_f[2,1]]])
        error_m = R_x.T * R_y
        error   = error_m[0,0]
        R_x_new = R_x - 0.5 * error * R_y
        R_x_new = R_x_new / (np.linalg.norm(R_x_new) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)
        R_y_new = R_y - 0.5 * error * R_x
        R_y_new = R_y_new / (np.linalg.norm(R_y_new) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)
        R_z_new_array = np.cross(R_x_new.T, R_y_new.T)
        R_z_new = np.mat([[R_z_new_array[0,0]],[R_z_new_array[0,1]],[R_z_new_array[0,2]]])
        self. R_f = np.bmat('R_x_new, R_y_new, R_z_new')  
        
        
        # R_mid = 1 / (1 + np.trace(self. R_f )) * ( self. R_f. T  - self. R_f);
        # self. R_f = np.linalg.inv( np.eye(3) + R_mid ) * (np.eye(3) - R_mid) 
 
         
          
        
    



  