### """chainplan SO_3_control,"""
### by Chainplain 2022-9-23
import numpy as np
from   scipy.spatial.transform import Rotation

class SO_3_controller:
    def __init__(self, Simulation_gap = 0.001):
        self.time_step = Simulation_gap

        ###COEFFICIENTS: we name the coefficients from outside to inside, you know what I mean
        self.k_R = 2
        #  k_R is a positive constant for rotational error,
        # and the rotation is an SO(3) matrix
        self.k_Omega = 0.2
        
        self.k_sl =0.15
        
        self.rho = 0.5
        # k_Omega is a positive constant for Omega error and omega is R_3 vector
        self.G       = np.matrix([[1,0,0],\
                                  [0,1,0],\
                                  [0,0,1]])
        # G is a positive definite, diagonal matrix
        self.k_J = 0.1
        # k_J is a positive constant , the adaptive term updating rate.
        self.sigma = 20
        # sigma(σ) is a positive constant, the adaptive term damping rate.
        self.epsilon = 0.1
        # epsilon (ε) is the high gain robust term.
        self.delta   = 0.2
        # delta(δ) is the disturbance amplitude.
        self.C       = 11
        # to construct e_A

        ### VARIANTS: Now we initial the variants, I would not like to say states of the values, because
        # they are same mathematically
        self.u = np.matrix([[0],[0],[0]])
        # 3 dimensional torque control input
        self.error_rotation = np.matrix([[0],[0],[0]])
        # in R^3 rotation error
        self.error_omega    = np.matrix([[0],[0],[0]])
        # in R^3 angular velocity error
        self.error_A =        self.C * self.error_rotation + self.error_omega
        # a combination of matrix
        self.omega_last          = np.matrix([[0],[0],[0]])
        # used only for Auto_generate_d_omega_last
        # in R^3 real angular velocity of the torso


        self. not_initial_Auto_generate_d_flag = False
        # If we need derivative calculation, then at the 1st step, we return a dump value


        self.J_estimated          = 0.0001 * np.matrix([[1,0,0],\
                                               [0,1,0],\
                                               [0,0,1]])
        # J_estimated is the estimation of J, it is typed as J_bar
        self.v_high_gain          = np.matrix([[0],[0],[0]])
        # R^3, used for robust control
        self.a_desire             = np.matrix([[0],[0],[0]])
        # R^3, virtual desire angular accelaration
        self.d_Omega_desired      = np.matrix([[0],[0],[0]])
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

    def Auto_generate_d_Omega_desired(self, angular_velocity_desired):
        ### Within one simulation period, either Auto_generate_d_Omega or Set_d_Omega_desired can be used once
        if (not self. not_initial_Auto_generate_d_flag):
            self.not_initial_Auto_generate_d_flag = True
            self.omega_last = angular_velocity_desired
            return

        else:
            self.d_Omega_desired = 1/self.time_step * (angular_velocity_desired - self.omega_last) 
            self.omega_last = angular_velocity_desired
        # print('angular_velocity_desired',angular_velocity_desired)
        # print ('self.d_Omega_desired ',self.d_Omega_desired )
    def get_Psi(self, rotation_matrix_current, rotation_matrix_desired):
        I       = np.matrix([[1,0,0],\
                                  [0,1,0],\
                                  [0,0,1]])
        return 0.5 * np.trace( I - rotation_matrix_desired.T * rotation_matrix_current)

    def Set_d_Omega_desired(self, derivative_of_d_Omega_desired):
        ### Within one simulation period, either Auto_generate_d_Omega or Set_d_Omega_desired can be used once
        self.d_Omega_desired = derivative_of_d_Omega_desired
        pass

    def Generate_control_signal(self, rotation_matrix_current, angular_velocity_current,\
                                      rotation_matrix_desired, angular_velocity_desired):
        # all these are given in matrixes
        self.error_rotation = 0.5 * self.vee_map(np.matmul(np.matmul(self.G, \
        np.transpose(rotation_matrix_desired) ),rotation_matrix_current)- \
        np.matmul(np.matmul(np.transpose(rotation_matrix_current), \
                    rotation_matrix_desired),self.G))


        self.error_omega    = angular_velocity_current - \
        np.matmul(np.matmul(np.transpose(rotation_matrix_current), self.G),angular_velocity_desired)
        # print('angular_velocity_current',angular_velocity_current)

        self.error_A        = self.C * self.error_rotation + self.error_omega
        # print('self.error_rotation', self.error_rotation )
        # print('self.error_omega', self.error_omega )
        print('Psi:', self.get_Psi(rotation_matrix_current, rotation_matrix_desired)  )

        self.a_desire       = - np.matmul( np.matmul( np.matmul( self.hat_map(angular_velocity_current),      \
                                           np.transpose(rotation_matrix_current)),  rotation_matrix_desired), \
                                           angular_velocity_desired) + \
                                np.matmul( np.matmul( np.transpose(rotation_matrix_current), \
                                                      rotation_matrix_desired),self.d_Omega_desired )
        print('self.d_Omega_desired',self.d_Omega_desired)
        
        J_estimated_1_st    = - np.matmul( self.a_desire, np.transpose(self.error_A) )
        J_estimated_2_nd    = - np.matmul( self.error_A, np.transpose(self.a_desire) )
        J_estimated_3_rd    =   np.matmul( angular_velocity_current, np.matmul( \
                                np.transpose( angular_velocity_current), self.hat_map(self.error_A) ) )
        J_estimated_4_th    =   np.matmul( self.hat_map(self.error_A), np.matmul( \
                                angular_velocity_current, np.transpose(angular_velocity_current) ) )
        self.J_estimated    =   self.J_estimated - 0.5 * self.time_step * self.k_J * (J_estimated_1_st + J_estimated_2_nd + J_estimated_3_rd \
                                                  + J_estimated_4_th + 2 * self.sigma * self.J_estimated )


        self.v_high_gain    =  - self.delta * self.delta / (self. delta * np.linalg.norm( self. error_A) \
                                                           + self. epsilon) * self.error_A        
        u_1st = - self.k_R * self.error_rotation
        u_2nd = - self.k_Omega * self.error_omega
        u_3rd = np.matmul( np.matmul( self.hat_map( angular_velocity_current ), self. J_estimated), \
                                             angular_velocity_current)
        u_4th = np.matmul( self. J_estimated, self. a_desire)
        u_5th = self. v_high_gain  
        u_6th = - self.k_sl * np.multiply( np.sign(self.error_A),  np.power(np.abs(self.error_A), self.rho))    
        # self. u             =   u_1st + u_2nd + u_5th + u_4th + u_3rd + u_6th#+ u_2nd + 0 
        proposed = u_1st + u_2nd  + u_5th + u_6th
        tau1     = u_1st + u_2nd 
        tau2     = u_1st + u_2nd  + u_3rd + u_4th + u_5th
       
        self. u             =   proposed #+ u_4th + u_3rd
        
        # print('u_1st', u_1st)  
        # print('u_2nd', u_2nd)   
        # print('u_3rd', u_3rd)   
        # print('u_4th', u_4th)   
        # print('u_5th', u_5th)                      