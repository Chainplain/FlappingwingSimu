"""chainplan Bladeelement Calculation,"""
#2022-4-10 14:30:36

import linecache
import numpy as np


#### Please note that, due to my laziness, the leading edge must coincides with the rotatioin axis,
#####                  or at least the 
#### and they should also coincide with the x axis of the wing
#### All the n-dim vector inputs are set as 1 x n numpy array
class BladeAeroCalculator():
    SMALL_CONSTANT_4_NOMALIZATION = 0.000000001

    def __init__(self, x_filename, y_filename, isLeadingEdge_RotationAxis = True, bladeElementNo = 50, air_Density = 1.29, \
                                                                                  avReynoldsNumber = 7000, x_hat_0_pos = 0, lengthperiod = 0.001,\
                                                                                  centerOfMass = [0,0,0]):
        self.x_file_name = x_filename
        self.y_file_name = y_filename
        self.x_data  = np.load(self.x_file_name)
        self.y_data  = np.load(self.y_file_name)
        print('@STATE:Blade Element Calculator Activated!!! File loaded.')
        print('x_data:', self. x_data)
        print('y_data:', self. y_data)
        # Both x_data and y_data are

        # self.body_rotation_in_inertia = np.matrix([[1, 0, 0],\
        #                                            [0, 1, 0],\
        #                                            [0, 0, 1]])  It seems unnecessary.

        self.wing_rotation_relative_to_inertia  =  np.matrix([[1.0, 0, 0],\
                                                              [0, 1.0, 0],\
                                                              [0, 0, 1.0]])

        self.virturalWingPlaneRelative2Wing   =  np.matrix([[1.0, 0, 0],\
                                                            [0, 1.0, 0],\
                                                            [0, 0, 1.0]])
        # Please note that virtural wing plane frame x-axis along the wing leading edge form root to tip.
        #                                        and y-axis along chordwise direction
        #                                        and the frame is right-handed
        ###        Most importantly, it is decided when once the wing is assembled and is constant in the whole simulation，
        #          We simply need this method to rotate the virtural wing plane frame to the current wing frame.

        self.Number_of_BladeElements = bladeElementNo

        # self. v_AoA = 0
        self. d_eFV = np.zeros([self.Number_of_BladeElements, 3])
        self. Filter_Constant = 0.2
        self. v_AoA = 0.0
        print('self. v_AoA loaded')
        self.BladeElementswidth      = (max( self.x_data ) - min( self.x_data )) / (self.Number_of_BladeElements -  1)
        ###  which is considered as Δr in the paper, and is 1 X self.Number_of_BladeElements -dim

        self.v_infi =  0.0
        self.v_t    =  0.0
        self.v_r    = np.zeros([self.Number_of_BladeElements, 3])
        # Please note that different form v_infi and v_t, v_r is a 3 x bladeElementNo-dim numpy array.
        self.eFV   =  np.zeros([self.Number_of_BladeElements, 3])
        self.added_mass_FV  =  np.zeros([self.Number_of_BladeElements, 3])
        self.AoA   =  np.zeros([self.Number_of_BladeElements, 1])

        self.aDen = air_Density

        self.x_hat_0 = x_hat_0_pos

        self.posLeadingEdgeVec = np.zeros([self.Number_of_BladeElements, 3])
        # this pos vec considers the posLeadingEdgeVec from every blade element top point

        self.length_period = lengthperiod# the simulation real time between two steps.
        self.lasteFV       = self.eFV
        self.lastAoA       = self.AoA

        self.CoM = centerOfMass

        self.avRey = avReynoldsNumber
        self.A_L   = 1.966 - 3.94 *  np.power(self.avRey, -0.429)
        self.A_D   = 1.873 - 3.14 *  np.power(self.avRey, -0.369)
        self.C_D_0 = 0.031 + 10.48 * np.power(self.avRey, -0.764)

        zeros_add = np.zeros((self.Number_of_BladeElements, 1))
        xdatamat = np.mat(self.x_data)
        xdatamat_transpose = np.transpose(xdatamat)
        self.posLeadingEdgeVec = np.hstack((xdatamat_transpose, zeros_add, zeros_add)) - self.CoM

        print('@STATE:Initial values settled!!!')

###Basic Tool Methods:
    def Normalize(self, rawA):
        # Second Norm Normalization
        # Please note that rawA here may be a matrix, if so we norm every row vector independently.
        # high dimension data must be reformed as matrix not ndarray
        if not isinstance(rawA,np.matrix):
            normA = np.sqrt(np.dot(rawA, rawA) + self.SMALL_CONSTANT_4_NOMALIZATION)
        else:
            normA = np.sqrt(np.sum(np.multiply(rawA, rawA), axis=1) + self.SMALL_CONSTANT_4_NOMALIZATION)
        return  rawA / normA

    def SecNorm(self, rawA):
        # Second Norm
        # Please note that rawA here may be a matrix, if so we norm every row vector independently.
        # high dimension data must be reformed as matrix not ndarray
        if not isinstance(rawA, np.matrix):
           return np.sqrt(np.dot(rawA, rawA))

        return np.sqrt(np.sum(np.multiply(rawA, rawA), axis = 1))

###Setting Methods:
    def SetAirDensity(self, air_Density):
        self.aDen = air_Density
        print("@CHANGE:The air density is set to ", self.aDen,".")

    def SetLengthPeriod(self, lengthperiod):
        self.length_period = lengthperiod
        print("@CHANGE:The length of one simulation segment (length_period) is set to ", self.length_period, ".")

    def SetNumber_of_BladeElements(self, bladeElementNo):
        self.Number_of_BladeElements = bladeElementNo
        print("@CHANGE:The Number_of_BladeElements is set to ", self.Number_of_BladeElements, ".")
        self.v_r = np.zeros(self.Number_of_BladeElements, 3)
        self.eFV = np.zeros([self.Number_of_BladeElements, 3])
        self.AoA = np.zeros([self.Number_of_BladeElements, 1])
        self.posLeadingEdgeVec = np.zeros([self.Number_of_BladeElements, 3])
        zeros_add = np.zeros((self.Number_of_BladeElements, 1))
        xdatamat = np.mat(self.x_data)
        xdatamat_transpose = np.transpose(xdatamat)
        self.posLeadingEdgeVec = np.hstack((xdatamat_transpose, zeros_add, zeros_add)) - self.CoM
        print("@CHANGE:The veloctity due to wing rotation (v_r) effective velocity (eFV) angle of attack (AoA) posLeadingEdgeVec are accordingly reset.")
        ### the rotation induced velocities must be also updated

    def SetAverageReynoldsNumberAndDecideCoeffs(self, avReynoldsNumber):
        self.avRey = avReynoldsNumber
        self.A_L   =  1.966 - 3.94  * np.power(self.avRey, -0.429)
        self.A_D   =  1.873 - 3.14  * np.power(self.avRey, -0.369)
        self.C_D_0 =  0.031 + 10.48 * np.power(self.avRey, -0.764)
        print("@CHANGE: The average Reynolds Number(avRey) is set to", self.avRey, '.')
        print("@CHANGE: The aerodynamics coefficients A_L, A_D, C_D_0 are accordingly reset.")

    def SetCoM(self, centerOfMass):
        self.CoM   = centerOfMass
        # in 3 dimension numpy array resolved in wing frame
        print("@CHANGE: The center of mass is reset to ",self.CoM,'.')

    def SetVirturalWingPlaneRelative2Wing(self, rotationVector):
        self.virturalWingPlaneRelative2Wing  = np.matrix([[rotationVector[0], rotationVector[1], rotationVector[2]],\
                                                          [rotationVector[3], rotationVector[4], rotationVector[5]],\
                                                          [rotationVector[6], rotationVector[7], rotationVector[8]]])
        print("@CHANGE: The virturalWingPlane to real Wing rotation matrix is set as \n", self.virturalWingPlaneRelative2Wing,'.')
        #  This method rotates the virtural wing plane frame to the current wing frame.

# ###Some Initialization
#     def Init_posLeadingEdgeVec(self):


###Fast Req Methods:

    def RequestWingOrientation2InertiaFrame(self, rotationVector):
        #The rotationVector is a 9-dim numpy array ,
        self.wing_rotation_relative_to_inertia = np.matrix([[rotationVector[0], rotationVector[1], rotationVector[2]],\
                                                            [rotationVector[3], rotationVector[4], rotationVector[5]],\
                                                            [rotationVector[6], rotationVector[7], rotationVector[8]]])
        ### following is just a small test
        #self.wing_rotation_relative_to_inertia = np.transpose(self.wing_rotation_relative_to_inertia)

    def RequestWingPlaneDirection(self):
        Normalized_LeadingEdgeRoot2Tip_in_wing_frame = np.matrix([[1], [0], [0]])
        self.LERT = np.matmul(self.wing_rotation_relative_to_inertia, np.matmul(self.virturalWingPlaneRelative2Wing,\
                                                                             Normalized_LeadingEdgeRoot2Tip_in_wing_frame))
        Normalized_chordDirecLeading2Trailing        = np.matrix([[0], [1], [0]])
        self.chordD = np.matmul(self.wing_rotation_relative_to_inertia,np.matmul(self.virturalWingPlaneRelative2Wing,\
                                                                             Normalized_chordDirecLeading2Trailing))

    # Both directions are resolved in the inertia frame


    def RequestVelocities(self, vel_FreeFlow, vel_body_translation, vel_wing_rotation, vel_AoA):
        ### If you will not get into the Webots simulation, ignore below notes.
        ##!!!!!! Please ascertain which frame the vel_wing_rotation is resolved in:
        # Quote from webots reference manual:The wb_supervisor_node_get_velocity function returns the absolute velocity
        #                                   (both linear and angular) of a node expressed in the global (world) coordinate system.
        #                                   And this is affirmative, checked myself
        self.v_infi = vel_FreeFlow
        self.v_t    = vel_body_translation
        transpose_posLeadingEdgeVec = np.transpose(self. posLeadingEdgeVec) 
        # print('self. posLeadingEdgeVec:', self. posLeadingEdgeVec)
        posLeadingEdgeVec_in_Inertia_frame = np.matmul(self.wing_rotation_relative_to_inertia, transpose_posLeadingEdgeVec)
        # print('posLeadingEdgeVec_in_Inertia_frame',posLeadingEdgeVec_in_Inertia_frame)
    # Previous posLeadingEdgeVec is resolved in the wing frame, such that it must be transfered into the inertia frame,
    #                                                               like posLeadingEdgeVec_in_Inertia_frame.
    # Please note that posLeadingEdgeVec_in_Inertia_frame is (3 x self.Number_of_BladeElements)-dim
    # That is to say np.transpose(posLeadingEdgeVec_in_Inertia_frame) is (self.Number_of_BladeElements x 3)-dim
        self.v_r    = np.cross(vel_wing_rotation, np.transpose(posLeadingEdgeVec_in_Inertia_frame)) #( self.Number_of_BladeElements x 3)-dim
        if not np.isnan(vel_AoA):
           self.v_AoA  = (self. Filter_Constant) * vel_AoA + (1.0 - self. Filter_Constant) * self.v_AoA
        else:
           self.v_AoA  = self.v_AoA

    # print(' self.v_AoA type ',  type(self.v_AoA) )
    # print(' vel_AoA type ',  type(vel_AoA) )
    # All these velocities are resolved in the inertia frame.
    # The body motion induced velocity is then also considered here, however implicitly in v_t and v_r
    # v_r is numpy matrix indeed



###Calculations:
    def CalcEffectiveVelocity(self):
    # self.LERT and self.chordD and the Velocities must be updated ahead
        rawResultantFlow =  - self.v_r  - self.v_t + self.v_infi #( self.Number_of_BladeElements x 3)-dim
        # print('self.v_t:', self.v_t)
        # print('self.v_r:', self.v_r)
        # print('self.v_infi:', self.v_infi)
    # Please note that v_r is different for each bladeElements, thus is a vector. And we know rawResultantFlow is also vector.
        self.last_added_mass_FV        = self.added_mass_FV
        self.eFV            = np.mat(rawResultantFlow - np.outer(np.dot(rawResultantFlow, self.LERT), self.LERT)) #( self.Number_of_BladeElements x 3)-dim
        # print('self.eFV ',self.eFV )
        self.rotation_eFV   = np.mat(- self.v_r - np.outer(np.dot(- self.v_r, self.LERT), self.LERT))
        self.added_mass_FV  = np.mat(-self.v_r - np.outer(np.dot(-self.v_r, self.LERT), self.LERT))
        # print('self.eFV:', self.eFV)
        # print('np.dot(rawResultantFlow, self.LERT):',np.dot(rawResultantFlow, self.LERT))
        # print('np.multiply(np.dot(rawResultantFlow, self.LERT), rawResultantFlow)',np.outer(np.dot(rawResultantFlow, self.LERT), self.LERT))
    def CalcAoA(self):
    # CalcEffectiveVelocity() must be updated first
    #     print('np.matmul(self.eFV, self.chordD)',np.matmul(self.eFV, self.chordD))
        # print('is self.eFV matrix:', type(self.eFV))
        # print('(self.SecNorm(self.eFV) + self.SMALL_CONSTANT_4_NOMALIZATION)',(self.SecNorm(self.eFV) + self.SMALL_CONSTANT_4_NOMALIZATION))
        rawCosineValueBetweenEffectiveVelocityAndchordDirecLeading2Trailing = \
            np.matmul(self.eFV, self.chordD) / (self.SecNorm(self.eFV) + self.SMALL_CONSTANT_4_NOMALIZATION)
        # self.lastAoA     = self.AoA
        self.AoA         = np.arccos(rawCosineValueBetweenEffectiveVelocityAndchordDirecLeading2Trailing)#( self.Number_of_BladeElements x 3)-dim


### Main implementations:
    def CopmputeAerodynamicForce(self):
        C_Lt  = self.A_L * np.sin(2 * self.AoA)                             #( self.Number_of_BladeElements x 1)-dim matrix
        C_Dt  = self.C_D_0 + self.A_D * (1 - np.cos(2 * self.AoA))          #( self.Number_of_BladeElements x 1)-dim matrix

        ##################### 1st Force component #####################
        # print('np.abs(self.y_data) shape:',self.y_data.shape )
        # print('C_Lt shape:', C_Lt.shape)
        # print('self.SecNorm(self.eFV)) shape:', self.SecNorm(self.eFV).shape)
        # print('np.multiply(C_Lt, self.SecNorm(self.eFV)) shape:', np.multiply(C_Lt, self.SecNorm(self.eFV)).shape)
        # print('np.multiply(np.abs(self.y_data),  np.multiply(C_Lt, self.SecNorm(self.eFV))) shape:', \
        #       np.multiply(np.mat(np.abs(self.y_data)).transpose(),  np.multiply(C_Lt, self.SecNorm(self.eFV))).shape)

        F_t_lift_amp = 0.5 * self.aDen * self.BladeElementswidth * \
                        np.multiply(np.mat(np.abs(self.y_data)).transpose(),  np.multiply(C_Lt, np.power(self.SecNorm(self.eFV),2) )) #np.multiply(C_Lt, self.SecNorm(self.eFV)) this may be wrong!!!
        # self.Rec_F_t_lift_amp = F_t_lift_amp
        # 1 x self.Number_of_BladeElements -dim
        # print('F_t_lift_amp shape:',F_t_lift_amp.shape)

        ##################### 2nd Force component #####################
        F_t_drag_amp = 0.5 * self.aDen * self.BladeElementswidth * \
                        np.multiply(np.mat(np.abs(self.y_data)).transpose(),  np.multiply(C_Dt, np.power(self.SecNorm(self.eFV),2) ))
        # 1 x self.Number_of_BladeElements -dim
        # print('self.SecNorm(self.eFV) **2:', np.power(self.SecNorm(self.eFV),2))
        # print('F_t_drag_amp shape:', F_t_drag_amp.shape)
        # print('F_t_drag_amp Total:', sum(F_t_drag_amp))

        self. d_eFV = (1 - self. Filter_Constant) * self. d_eFV + \
                      self.Filter_Constant * (self.added_mass_FV - self.last_added_mass_FV) / self.length_period
        # d_AoA = (self.AoA - self.lastAoA) / self.length_period
        # print('d_eFV:',d_eFV)
        # print('d_AoA',d_AoA)
        # self. Rec_d_AoA = d_AoA
        # print('d_eFV shape:', d_eFV.shape)
        # print('self.eFV :', self.eFV )
        # print('self.lasteFV:', self.lasteFV)
        # print('d_eFV:', d_eFV)
        # print('d_AoA shape:', d_AoA.shape)
        # print('self.AoA:', self.AoA)
        # print('self.lastAoA:', self.lastAoA)
        # print('d_AoA:', d_AoA)

        C_r = np.pi * (0.75 - self.x_hat_0)
        ## Please note the specific value of C_r

        ##################### 3rd Force component #####################
        # print('self.BladeElementswidth',self.BladeElementswidth)
        Sec_Norm_eFV = self.SecNorm(self.rotation_eFV  )
        # print('Sec_Norm_eFV:', Sec_Norm_eFV)
        F_r_amp      = 0.5 * self.aDen * C_r * self.BladeElementswidth * np.abs(self.v_AoA) * \
                        np. multiply( Sec_Norm_eFV, np.matrix(self.y_data * self.y_data).transpose() )       
                       # np.matrix(self.y_data * self.y_data).transpose()
                                                   # 
        # F_r_amp      = self.aDen * C_r * self.BladeElementswidth * \
                        # np.multiply(np.matrix(self.y_data * self.y_data).transpose(), np.abs(d_AoA))
        # 1 x self.Number_of_BladeElements -dim
        # print('F_r_amp shape:', F_r_amp)
        # print('np.multiply(self.y_data * self.y_data, self.SecNorm(d_AoA)) shape:', np.multiply(self.y_data * self.y_data, np.abs(d_AoA)).shape)
        # print('np.multiply(self.y_data * self.y_data, self.SecNorm(d_AoA)) shape:',(np.multiply(np.abs(d_AoA), np.matrix(self.y_data * self.y_data).transpose())).shape)
        # print('d_eFV shape:', d_eFV.shape)
        # print('np.multiply(self.eFV, d_eFV) shape:', np.multiply(self.eFV, d_eFV).shape)
        # print('self.SecNorm(self.eFV) shape:', self.SecNorm(self.eFV).shape)
        # print('np.sin(self.AoA) shape:', np.sin(self.AoA).shape)
        # print('np.multiply(self.eFV, d_eFV) / (self.SecNorm(self.eFV) + self.SMALL_CONSTANT_4_NOMALIZATION) shape:', \
        #       (np.sum(np.multiply(self.eFV, d_eFV) / (self.SecNorm(self.eFV) + self.SMALL_CONSTANT_4_NOMALIZATION),axis=1)).shape)

        EF_tokken     = np.multiply(np.sum(np.multiply(self.eFV, self. d_eFV ) / (self.SecNorm(self.eFV) + self.SMALL_CONSTANT_4_NOMALIZATION),axis=1), np.sin(self.AoA)) \
                        + np.multiply(np.multiply(self.SecNorm(self. d_eFV ), -self.AoA + np.pi), np.cos(self.AoA ))
        ## eFV is self.Number_of_BladeElements x 3-dim
        # print('EF_tokken:',EF_tokken)

        ##################### 4th Force component #####################
        F_a_amp       = 0.25 * self.aDen * self.BladeElementswidth * \
                       np.pi * np.multiply(np.mat(self.y_data * self.y_data).transpose() , EF_tokken) #( self.Number_of_BladeElements x 1)-dim
        # 1 x self.Number_of_BladeElements -dim
        # print('####Watching:', np.multiply(np.mat(self.y_data * self.y_data).transpose() , EF_tokken))
        # print('self.y_data * self.y_data',self.y_data * self.y_data)

        ##################### Directions computation #####################
        # print('self.LERT shape:', self.LERT.shape)
        # print('self.LERT array:',np.array(self.LERT.transpose())[0,:])
        # print('self.eFV shape:', self.eFV.shape)
        # np.cross(np.array(self.LERT.transpose())[0, :], self.eFV)
        _1st_comp_direc = self.Normalize( np.matrix(np.cross(np.array(self.LERT.transpose())[0,:], self.eFV)))
        # self.Number_of_BladeElements x 3-dim
        mend_1st_direc_index = np.dot(_1st_comp_direc, self.chordD) > 0
        array_mend_1st_direc_index = np.array(mend_1st_direc_index).squeeze()
        _1st_comp_direc[array_mend_1st_direc_index] = - _1st_comp_direc[array_mend_1st_direc_index]
        # print('_1st_comp_direc:',_1st_comp_direc)
        # self.out1st = _1st_comp_direc
        _2nd_comp_direc = self.Normalize(self.eFV)
        ## self.Number_of_BladeElements x 3-dim
        # print('_2nd_comp_direc',_2nd_comp_direc)

        _3rd_and_4th_comp_direc_single = np.cross(np.array(self.LERT.transpose())[0,:],  np.array(self.chordD.transpose())[0,:])
        _3rd_and_4th_comp_direc        \
            = np.matrix(np.tile(_3rd_and_4th_comp_direc_single, (self.Number_of_BladeElements,1)))
        mend_3rd_and_4th_comp_direc_index = np.dot(self.eFV, _3rd_and_4th_comp_direc_single) < 0
        array_mend_3rd_and_4th_comp_direc_index = np.array(mend_3rd_and_4th_comp_direc_index).squeeze()
        _3rd_and_4th_comp_direc[array_mend_3rd_and_4th_comp_direc_index] =\
            - _3rd_and_4th_comp_direc[array_mend_3rd_and_4th_comp_direc_index]
        # print('_3rd_and_4th_comp_direc:',_3rd_and_4th_comp_direc)
        # _3rd_and_4th_comp_direc[array_mend_3rd_and_4th_comp_direc_index] = - _3rd_and_4th_comp_direc[array_mend_3rd_and_4th_comp_direc_index]
        ## 1 x 3-dim F_r and F_a share the same direction
        # print('np.array(self.LERT.transpose())[0,:]:', np.array(self.LERT.transpose())[0,:])
        # print('np.array(self.chordD.transpose())[0,:]:', np.array(self.chordD.transpose())[0,:])
        # print('_3rd_and_4th_comp_direc:', _3rd_and_4th_comp_direc)

        #################### Amplitude and direction composation ###############
        # self. F_t_lift_per_BE = np.multiply(np.transpose(np.mat(F_t_lift_amp)),_1st_comp_direc)   ## self.Number_of_BladeElements x 3-dim
        # self. F_t_drag_per_BE = np.multiply(np.transpose(np.mat(F_t_drag_amp)),_2nd_comp_direc)   ## self.Number_of_BladeElements x 3-dim
        # self. F_r_per_BE      = np.outer   (np.transpose(np.mat(F_r_amp)),     _3rd_and_4th_comp_direc)   ## self.Number_of_BladeElements x 3-dim
        # self. F_a_per_BE      = np.outer   (np.transpose(np.mat(F_a_amp)),     _3rd_and_4th_comp_direc)   ## self.Number_of_BladeElements x 3-dim

        Sum_abs_of_F_t_lift_amp    = np.sum(np.abs(F_t_lift_amp))
        Sum_abs_of_F_r_amp = np.sum(np.abs(F_r_amp))
        Sum_abs_of_F_a_amp = np.sum(np.abs(F_a_amp))
        
        Sign_sum_of_F_t_lift_amp   =  np.sign(np.sum(F_t_lift_amp))
        Sign_sum_of_F_r_amp   =  np.sign(np.sum(F_r_amp))
        Sign_sum_of_F_a_amp   =  np.sign(np.sum(F_a_amp))

        if Sum_abs_of_F_t_lift_amp > self.SMALL_CONSTANT_4_NOMALIZATION:
            Weighted_F_t_lift_amp  = Sign_sum_of_F_t_lift_amp * F_t_lift_amp / (Sum_abs_of_F_t_lift_amp + self.SMALL_CONSTANT_4_NOMALIZATION)
        else:
            Weighted_F_t_lift_amp  = 1 / self.Number_of_BladeElements

        if Sum_abs_of_F_r_amp > self.SMALL_CONSTANT_4_NOMALIZATION:
            Weighted_F_r_amp  = Sign_sum_of_F_r_amp * F_r_amp / (Sum_abs_of_F_r_amp + self.SMALL_CONSTANT_4_NOMALIZATION)
        else:
            Weighted_F_r_amp  = 1 / self.Number_of_BladeElements

        if Sum_abs_of_F_a_amp > self.SMALL_CONSTANT_4_NOMALIZATION:
            Weighted_F_a_amp = Sign_sum_of_F_a_amp * F_a_amp / (Sum_abs_of_F_a_amp+ self.SMALL_CONSTANT_4_NOMALIZATION)
        else:
            Weighted_F_a_amp = 1 / self.Number_of_BladeElements



        # print('Weighted_F_t_lift_amp:',Weighted_F_t_lift_amp)
        # print('Weighted_F_r_amp:',Weighted_F_r_amp)
        # print('Weighted_F_a_amp:', Sum_of_F_a_amp)
        # print('F_r_amp',F_r_amp)
        ###     Below are the results
        self. F_t_lift = np.multiply(F_t_lift_amp,    _1st_comp_direc)   ## 检查求和及相乘的顺序
        self. F_t_drag = np.multiply(F_t_drag_amp,    _2nd_comp_direc)
        self. F_r      = np.multiply(F_r_amp, _3rd_and_4th_comp_direc)   ## 检查计算方法
        self. F_a      = np.multiply(F_a_amp, _3rd_and_4th_comp_direc)
        # print('F_t_lift:', np.sum(self.F_t_lift, axis = 0))
        # print('F_t_drag:', np.sum(self.F_t_drag, axis = 0))
        # print('F_r:', np.sum(self.F_r, axis = 0))
        # print('F_a:', np.sum(self.F_a, axis = 0))

        raw_X_pos_t  = np.sum(np.multiply(Weighted_F_t_lift_amp, np.mat(self.x_data).transpose()))
        raw_X_pos_r  = np.sum(np.multiply(Weighted_F_r_amp,      np.mat(self.x_data).transpose())) ##或许应该求算数平均数
        raw_X_pos_a  = np.sum(np.multiply(Weighted_F_a_amp,      np.mat(self.x_data).transpose()))


        raw_Y_pos_t  = - np.sum(np.multiply(np.multiply(Weighted_F_t_lift_amp, np.mat(np.abs(self.y_data)).transpose()),\
                                     (np.abs(-self.AoA.transpose()+np.pi)/np.pi).transpose()))
        raw_Y_pos_r  = - 0.5    * np.sum(np.multiply(Weighted_F_r_amp, np.mat(np.abs(self.y_data)).transpose()))
        raw_Y_pos_a  = - 0.5625 * np.sum(np.multiply(Weighted_F_a_amp, np.mat(np.abs(self.y_data)).transpose()))

        raw_pos_t    =   np.array(np.matmul(self.virturalWingPlaneRelative2Wing.transpose(), np.array([raw_X_pos_t, raw_Y_pos_t, 0]))).squeeze()
        raw_pos_r    =   np.array(np.matmul(self.virturalWingPlaneRelative2Wing.transpose(), np.array([raw_X_pos_r, raw_Y_pos_r, 0]))).squeeze()
        raw_pos_a    =   np.array(np.matmul(self.virturalWingPlaneRelative2Wing.transpose(), np.array([raw_X_pos_a, raw_Y_pos_a, 0]))).squeeze()
        
        # print(raw_pos_t)
        self. X_pos_t = raw_pos_t[0]
        self. Y_pos_t = raw_pos_t[1]
        
        self. X_pos_r = raw_pos_r[0]
        self. Y_pos_r = raw_pos_r[1]
        
        self. X_pos_a = raw_pos_a[0]
        self. Y_pos_a = raw_pos_a[1]
        ### We need the relationship graph
        ##  And we need initialize all the parameters in the _init_






