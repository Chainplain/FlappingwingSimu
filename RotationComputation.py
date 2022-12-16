"""chainplan Rotation Computation,"""
#2022-7-28 15:19:33
import numpy as np
from   scipy.spatial.transform import Rotation 

EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION = 0.00000000001

def FromPointer2Axis_Angle(Pointer,origin_pointer = np.array([0,1,0])):
    # Pointer must be an 3-dimensional ndarray
    # Return value : rotation_value, rotation_axis
    
    Pointer_2_norm = np.linalg.norm(Pointer)
    # if Pointer_2_norm <  EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION and Pointer_2_norm > - EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION
    normailized_pointer = Pointer / (Pointer_2_norm + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)

    # print ('$$$ normailized_pointer:',normailized_pointer)
    rotation_axis  = np.cross(origin_pointer, normailized_pointer)
    rotation_value = np.pi

    # print('rotation_axis:',rotation_axis)

    rotation_axis_norm         = np.linalg.norm(rotation_axis)
    normailized_rotation_axis  = rotation_axis / (rotation_axis_norm \
                                                  + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)
    # print('$$$ rotation_axis_norm:', rotation_axis_norm)

    if rotation_axis_norm < EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION:
        normailized_rotation_axis = np.array([1,0,0])
        # rotation_value = np.pi
        return np.append(rotation_value, normailized_rotation_axis)


    cosine_value   = np.dot(normailized_pointer,origin_pointer)
    # print('normailized_rotation_axis:',normailized_rotation_axis)
    # print('cosine_value:',cosine_value )
    rotation_value = np.arccos(cosine_value)

    return np.append(normailized_rotation_axis, rotation_value)
    
def FromRotation2Euler_Angle_in_Rad (Rot_Vec):
    Rotation_for_Rot_Vec_But_not_just_rotation_matrix = Rotation.from_matrix(\
               [[Rot_Vec[0], Rot_Vec[1], Rot_Vec[2]],\
                [Rot_Vec[3], Rot_Vec[4], Rot_Vec[5]],\
                [Rot_Vec[6], Rot_Vec[7], Rot_Vec[8]]])
                
    return Rotation_for_Rot_Vec_But_not_just_rotation_matrix.as_euler('zyx', degrees=False)
    # Rot_Vec is the rotation matrix explained as a 1X9 Vec np.list, row by row
    # Return the Euler Angle :

def GetUnitDirection_Safe(Arrow):
    return Arrow/ ( np.linalg.norm(Arrow) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)

###From a 3D pointer we calculate the axis-angle value
def CalcTorque_in_Robot(force, arm):
    if not isinstance(force, np. ndarray):
        force = np. array(force)
    if not isinstance(arm, np. ndarray):
        arm = np. array(arm)
    torque = np.cross(force, arm) 
    # print('force',force)
    # print('arm',arm)
    # print('torque',torque)
    
    return np.cross(force, arm)
###To provide the desired attitude trajectory
class Angle_Trajectory_Generator():
    def __init__(self, initial_orientation, initial_omega, simulation_gap):
        self. omega = initial_omega
        self. orientation = initial_orientation
        self. gap = simulation_gap
    
    def hat_map(self, R3vector):
    ## from R^3 → a 3x3 skew-symmetric matrix
        so3matrix = np.matrix([[0.0,            -R3vector[2,0], R3vector[1,0] ],
                           [R3vector[2,0],  0.0,            -R3vector[0,0]],
                           [-R3vector[1,0], R3vector[0,0],  0.0          ]])
        return so3matrix
        
    def march_forward(self, computing_orientation, computing_omega):
        self. orientation = computing_orientation + self. gap * computing_orientation * self. hat_map(computing_omega) 
    # the normalization follows mahony
        R_x = np.matrix([[ self. orientation[0,0]],
                     [ self. orientation[1,0]],
                     [ self. orientation[2,0]]])
        R_y = np.matrix([[ self. orientation[0,1]],
                     [ self. orientation[1,1]],
                     [ self. orientation[2,1]]])
        error_m = R_x.T * R_y
        error   = error_m[0,0]
        R_x_new = R_x - 0.5 * error * R_y
        R_x_new = R_x_new / (np.linalg.norm(R_x_new) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)
        R_y_new = R_y - 0.5 * error * R_x
        R_y_new = R_y_new / (np.linalg.norm(R_y_new) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)
        R_z_new_array = np.cross(R_x_new.T, R_y_new.T)
        R_z_new = np.mat([[R_z_new_array[0,0]],[R_z_new_array[0,1]],[R_z_new_array[0,2]]])
        self. orientation = np.bmat('R_x_new, R_y_new, R_z_new')
        self. omega       = computing_omega
    
    def get_orientation(self):
        return self. orientation
    

