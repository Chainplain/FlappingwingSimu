"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Robot,Motor
from controller import Robot,Motor
from controller import Supervisor
from controller import Node
from controller import PositionSensor
from controller import Display


import numpy as np
from   Bladelement import BladeAeroCalculator as BAC
from   RotationComputation import FromPointer2Axis_Angle as P2AA
from   RotationComputation import FromRotation2Euler_Angle_in_Rad as R2EAR
from   RotationComputation import GetUnitDirection_Safe
from   ArrowManager import ForceArrowManager as FAM
from   SpecialorthogonalControl import SO_3_controller as SOC
from   RotationComputation import Angle_Trajectory_Generator as ATG
from   RotationComputation import CalcTorque_in_Robot as CTR
from   FilterLib import Low_Pass_Second_Order_Filter as LPSF
from   FTslideControl import Finite_time_slide_mode_observer_3dim as FT_observer
from   FTslideControl import Positional_Traj_Track_Controller as  PTTC
from   FTslideControl import Computing_desired_rotation
from   FTslideControl import Attitude_reference_generator as ARG

import random
import scipy.io as scio
# from controller import Node
# create the Robot instance.3
############### OVerall Test Region ############
This_experiemnt_name  = '5th_proposed'
Desired_Angular_velocity      = '_C_0_1_0'


is_filtered  = 1
##_DW_triangle_05pi
##_DW_0_0_05pi
Given_av = np.matrix([[0],[-0.5 * np.pi],[0]])
Zero_av = np.matrix([[0],[0],[0]])



Total_File = '_Traj_Tracking_File.mat'
mat_save_length = 20000
StrokeFreq   = 13

Total_wing_torque_list   = []
Total_wing_force_list    = []
Total_rudder_torque_list = []
Total_rudder_force_list  = []
Total_tail_torque_list   = []
Total_tail_force_list    = []

Total_body_rotation_list     = []
Total_body_translation_list  = []
Total_body_translation_desire  = []
Total_body_angular_velocity_list  = []
Total_Angular_velocity_filtered   = []
Total_desired_rotation_list     = []
# Total_desired_translation_list  = []
Total_desired_angular_velocity_list = []
Total_psi_rotation_error_list  = []
Total_yaw_input = []
Total_roll_input = []
Total_pitch_input = []

Total_Freq_stroke = []
############### OVerall Test Region ############
flapper = Supervisor()

FORCE_RELATIVECONSTANT = 0.0005
arena_Air_Density = 1.29
Simulation_Gap    = 0.001
Controller_Gap_vs_Simulation_Gap = 10
FromWing2Tail = 0.15
# dAOAfile = open("FirstTestdAOA.txt", 'a')
# dAOAfile.truncate(0)
# drawer  = Display('mirror')
initial_translation = [0, 0, 0]
initial_rotation_in_axis_angle = [1, 0 ,0, 0]

LU_bac = BAC('SimpleFlapper0Wing40BLE_X_pos.npy',\
                'SimpleFlapper0Wing40BLE_Y_pos.npy',bladeElementNo=40)
LD_bac = BAC('SimpleFlapper0Wing40BLE_X_pos.npy',\
                'SimpleFlapper0Wing40BLE_Y_pos.npy',bladeElementNo=40)
RU_bac = BAC('SimpleFlapper0Wing40BLE_X_pos.npy',\
                'SimpleFlapper0Wing40BLE_Y_pos.npy',bladeElementNo=40)
RD_bac = BAC('SimpleFlapper0Wing40BLE_X_pos.npy',\
                'SimpleFlapper0Wing40BLE_Y_pos.npy',bladeElementNo=40)
rudder_bac = BAC('SimpleFlapper0Tail_V20BLE_X_pos.npy',\
                'SimpleFlapper0Tail_V20BLE_Y_pos.npy',bladeElementNo=20) 
tail_bac =   BAC('SimpleFlapper0Tail_H20BLE_X_pos.npy',\
                'SimpleFlapper0Tail_H20BLE_Y_pos.npy',bladeElementNo=20) 
                               
S_d_Flapping_wing_actuator_disk_area = np.pi * 0.15 * 0.15 * 0.5
###                                π       radius        disk half of circle
Distance_from_vehicle_center_to_horizontal_tail_in_Z = -0.15
Distance_from_vehicle_center_to_horizontal_tail_in_X = -0.01

tail_shift_in_Flapper = np.array([Distance_from_vehicle_center_to_horizontal_tail_in_X,\
                        0,\
                        Distance_from_vehicle_center_to_horizontal_tail_in_Z])

             

LU_bac.SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
LD_bac.SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
RU_bac.SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
RD_bac.SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
rudder_bac.SetVirturalWingPlaneRelative2Wing([0,1,0,-1,0,0,0,0,1])
tail_bac.  SetVirturalWingPlaneRelative2Wing([0,0,-1,1,0,0,0,-1,0])  # Rot Y, -90    Rot  Z -90

tail_Rotation_matrix_VP2W = np.matrix([[0, 0,-1],\
                                       [1, 0, 0],\
                                       [0,-1, 0]])

R_d         = np.matrix([[1, 0, 0],\
                         [0, 1, 0],\
                         [0, 0, 1]]) 
                                       
R_z_static  = np.matrix([[1, 0, 0],\
                         [0, 1, 0],\
                         [0, 0, 1]]) 
                         
Omega_static = np.matrix([[0],\
                          [0],\
                          [0]])                          
# rudder_mat_VirturalWingPlaneRelative2Wing =  \
# np.matrix([[0,1,0],\
           # [-1,0,0],\
           # [0,0,1]])

# tail_mat_VirturalWingPlaneRelative2Wing =  \
# np.matrix([[0,1,0],\
           # [0,0,1],\
           # [1,0,0]])
# drawer.drawRectangle(0, 0, 1, 1)
# flapper = robot.getFromDef('mybot')
# flapper = Robot()
# WbDeviceTag Real_motor_LU_RD_device
## Watch out
# forward_MotorS = robot.getRoot('ball')
# get the time step of the current world.
# multi_speed = 4
timestep = int(flapper.getBasicTimeStep()) #/ multi_speed
# print (robot.getBasicTimeStep()/4)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# forward_MotorS = robot.getFromDef('LU')
# rudder_MotorS = robot.getFromDef('rudder')
 
Real_motor_LU_RD_joint  = flapper.getDevice('LU_RD_joint')
Real_motor_LD_RU_joint  = flapper.getDevice('LD_RU_joint')

Real_motor_LU_RD_joint_sensor = flapper.getDevice('LU_RD_joint_sensor')
Real_motor_LD_RU_joint_sensor = flapper.getDevice('LD_RU_joint_sensor')

Real_motor_LU_RD_joint_sensor.enable(1)
Real_motor_LD_RU_joint_sensor.enable(1)



Real_motor_LU_wing_joint = flapper.getDevice('LU_wing_joint')
Real_motor_LD_wing_joint = flapper.getDevice('LD_wing_joint')
Real_motor_RU_wing_joint = flapper.getDevice('RU_wing_joint')
Real_motor_RD_wing_joint = flapper.getDevice('RD_wing_joint')
Real_motor_rudder_joint  = flapper.getDevice('rudder_joint')


Real_motor_LU_wing_joint_sensor = flapper.getDevice('LU_wing_joint_sensor')
Real_motor_LD_wing_joint_sensor = flapper.getDevice('LD_wing_joint_sensor')
Real_motor_RU_wing_joint_sensor = flapper.getDevice('RU_wing_joint_sensor')
Real_motor_RD_wing_joint_sensor = flapper.getDevice('RD_wing_joint_sensor')
Real_motor_rudder_joint_sensor  = flapper.getDevice('rudder_joint_sensor')


Real_motor_LU_wing_joint_sensor.enable(1)
Real_motor_LD_wing_joint_sensor.enable(1)
Real_motor_RU_wing_joint_sensor.enable(1)
Real_motor_RD_wing_joint_sensor.enable(1)
Real_motor_rudder_joint_sensor. enable(1)

induced_velocity_record = []
induced_velocity_record_list_length = 1000
# print('Get Real_motor_LU_RD_joint position:',\
                     # Real_motor_LU_RD_joint_sensor.getValue())

# The sensor can only be read in the while loop Since only then it is sampled.

LU_wing                 = flapper.getFromDef('LU')
LD_wing                 = flapper.getFromDef('LD')
RU_wing                 = flapper.getFromDef('RU')
RD_wing                 = flapper.getFromDef('RD')
rudder                  = flapper.getFromDef('rudder')
# tail                    = flapper.getFromDef('TheFlapper') the same

TheFlapper              = flapper.getFromDef('TheFlapper')
tail                    = TheFlapper
Flapper_translation      = TheFlapper.getField('translation')
Flapper_rotation         = TheFlapper.getField('rotation')
# LU_RD_joint             = flapper.getFromDef('LU_RD')

### Set initial postion
# initial_translation = [0, 0, 0]
# initial_rotation_in_axis_angle = [-1, 0 ,0, np.pi]
Flapper_translation.setSFVec3f(initial_translation)
Flapper_rotation.setSFRotation(initial_rotation_in_axis_angle)

LU_drag_arrow             = flapper.getFromDef('LU_drag_arrow')
LU_drag_FAM                     = FAM(LU_drag_arrow)

# LU_lift_arrowN             = flapper.getFromDef('LU_lift_arrowN')
# LU_liftN_FAM                     = FAM(LU_lift_arrowN)

# LD_drag_arrow             = flapper.getFromDef('LD_drag_arrow')
# LD_drag_FAM                     = FAM(LD_drag_arrow)

LU_lift_arrow             = flapper.getFromDef('LU_lift_arrow')
LU_lift_FAM                     = FAM(LU_lift_arrow)

LU_r_arrow                = flapper.getFromDef('LU_r_arrow')
LU_r_FAM                        = FAM(LU_r_arrow)

LU_a_arrow                = flapper.getFromDef('LU_a_arrow')
LU_a_FAM                        = FAM(LU_a_arrow)

rudder_drag_arrow         = flapper.getFromDef('rudder_drag_arrow')
rudder_drag_FAM                 = FAM(rudder_drag_arrow)

tail_drag_arrow             = flapper.getFromDef('tail_drag_arrow')
tail_drag_FAM                     = FAM(tail_drag_arrow)
# LD_lift_arrow             = flapper.getFromDef('LD_lift_arrow')
# LD_lift_FAM                     = FAM(LD_lift_arrow)

#'MY_ROBOT'

force = [0,0.1,0.4]

count = 0;

LU_OrVec = LU_wing.getOrientation()
LD_OrVec = LD_wing.getOrientation()
RU_OrVec = RU_wing.getOrientation()
RD_OrVec = RD_wing.getOrientation()


LU_wing_Rotation_matrix = \
np.matrix([[LU_OrVec[0], LU_OrVec[1], LU_OrVec[2]],\
           [LU_OrVec[3], LU_OrVec[4], LU_OrVec[5]],\
           [LU_OrVec[6], LU_OrVec[7], LU_OrVec[8]]])
           
LD_wing_Rotation_matrix = \
np.matrix([[LD_OrVec[0], LD_OrVec[1], LD_OrVec[2]],\
           [LD_OrVec[3], LD_OrVec[4], LD_OrVec[5]],\
           [LD_OrVec[6], LD_OrVec[7], LD_OrVec[8]]])
           
RU_wing_Rotation_matrix = \
np.matrix([[RU_OrVec[0], RU_OrVec[1], RU_OrVec[2]],\
           [RU_OrVec[3], RU_OrVec[4], RU_OrVec[5]],\
           [RU_OrVec[6], RU_OrVec[7], RU_OrVec[8]]])
           
RD_wing_Rotation_matrix = \
np.matrix([[RD_OrVec[0], RD_OrVec[1], RD_OrVec[2]],\
           [RD_OrVec[3], RD_OrVec[4], RD_OrVec[5]],\
           [RD_OrVec[6], RD_OrVec[7], RD_OrVec[8]]])

print('LU_wing_Rotation_matrix:\n',LU_wing_Rotation_matrix)
print('LD_wing_Rotation_matrix:\n',LD_wing_Rotation_matrix)
print('LU_wing_Rotation_matrix:\n',RU_wing_Rotation_matrix)
print('LD_wing_Rotation_matrix:\n',RD_wing_Rotation_matrix)

print('Rotation matrix Test Complete!\n')

LU_velVec = LU_wing.getVelocity()
LD_velVec = LD_wing.getVelocity()
RU_velVec = RU_wing.getVelocity()
RD_velVec = RD_wing.getVelocity()


print('LU_velVec:',LU_velVec[0:3],LU_velVec[3:6])
print('LD_velVec:',LD_velVec[0:3],LD_velVec[3:6])
print('RU_velVec:',RU_velVec[0:3],RU_velVec[3:6])
print('RD_velVec:',RD_velVec[0:3],RD_velVec[3:6])
Flapping_wing_induced_flow_Tail         = np.array([0, 0, 0])
print('Velocity Vector Test Complete!\n')

vel_FreeFlow                       = np.array([0,0,0])
Flapping_wing_induced_flow         = np.array([0,0,0])
# print('LU_wing Orientation:',LU_wing.getOrientation())
# print('LD_wing Orientation:',LD_wing.getOrientation())
# print('RU_wing Orientation:',RU_wing.getOrientation())
# print('RD_wing Orientation:',RD_wing.getOrientation())
# Rudder_Comparing_set = Rudder_translation.getSFVec3f()
# Rudder_Comparing_np = np.array(Rudder_Comparing_set)
# TranslationBack2o = list(Rudder_Comparing_np + [0,0,0])
# print('TranslationBack2o:',TranslationBack2o
StrokeAmp    = np.pi / 8

StrokeAmpOffSet = StrokeAmp

# Real_motor_LU_RD_joint.maxVelocity = 100
# Real_motor_LD_RU_joint.maxVelocity = 100
CountKit = 0

Real_motor_LU_wing_joint_sensor_value = Real_motor_LU_wing_joint_sensor.getValue()
Real_motor_LD_wing_joint_sensor_value = Real_motor_LD_wing_joint_sensor.getValue()
Real_motor_RU_wing_joint_sensor_value = Real_motor_RU_wing_joint_sensor.getValue()
Real_motor_RD_wing_joint_sensor_value = Real_motor_RD_wing_joint_sensor.getValue()
Real_motor_rudder_joint_sensor_value  = Real_motor_rudder_joint_sensor.getValue()

# StrokeFreq_Smooth =  StrokeFreq
# K_StrokeFreq_Smooth = 1
Theta             = 0

###-----Position Observer-------###
Here_pos_observer = FT_observer(robot_mass = 0.02) 
Here_pos_observer. p_observer = np.mat([[initial_translation[0]],\
                                        [initial_translation[1]],\
                                        [initial_translation[2]]])


###------Controller Initialisation-------###
SO3_Attitude_Controller = SOC()
SO3_Attitude_Controller.time_step = 0.001


###------Position Controller------####
Postion_Controller = PTTC(robot_mass = 0.02)



### Attitude Reference Generator
Here_ARG = ARG()
Here_ARG. generator_gap_AV = Simulation_Gap * Controller_Gap_vs_Simulation_Gap
Here_ARG. generator_gap_rotation = Simulation_Gap
## we first use stationary desire 
Flapper_Rotation_desired = np.matrix([[1,0,0],[0,1,0],[0,0,1]])

Flapper_OrVec                = TheFlapper.getOrientation()
Flapper_Rotation_matrix_initial = \
            np.matrix([[1,  0, 0],\
                       [0, 1, 0],\
                       [0, 0, 1]])

Flapper_Rotation_desired = Flapper_Rotation_matrix_initial
Flapper_Angular_velocity_desired = Given_av
Angular_velocity_filter = LPSF(Zero_av, 9, 5, Simulation_Gap)

Here_ATG = ATG(Flapper_Rotation_desired, Flapper_Angular_velocity_desired, Simulation_Gap * Controller_Gap_vs_Simulation_Gap)

RecordCount = 0


### Desired trajectory
p_d_x   =   0 
p_d_x_last = 0
p_d_y   =   0
p_d_y_last = 0
   
v_d_x   =   0
v_d_x_last = 0
v_d_y   =   0
v_d_y_last = 0    
     
d_v_d_x =   0
d_v_d_y =   0



while flapper.step(timestep) != -1:
    # print('timestep',timestep)
    
    # Now_time = flapper.getTime()
    
    # StrokeFreq_Smooth = K_StrokeFreq_Smooth * (StrokeFreq - StrokeFreq_Smooth)
    Theta = Theta +  2 * np.pi * Simulation_Gap * StrokeFreq
    # print('Theta',Theta)
    if (Theta > 2 * np.pi):
        Theta = np.mod(Theta,  2 * np.pi )
    LU_RD_joint_angle = StrokeAmp * np.sin(Theta) + StrokeAmpOffSet
    LD_RU_joint_angle = -LU_RD_joint_angle
    Real_motor_LU_RD_joint.setPosition(LU_RD_joint_angle)
    Real_motor_LD_RU_joint.setPosition(LD_RU_joint_angle)
    # print('LU_RD_joint_angle',LU_RD_joint_angle)
    # Real_motor_LU_RD_joint.setVelocity(10)
    # Real_motor_LD_RU_joint.setVelocity(10)
    # print('LU_RD_joint_angle:',LU_RD_joint_angle)

    LU_OrVec = LU_wing.getOrientation()
    LD_OrVec = LD_wing.getOrientation()
    RU_OrVec = RU_wing.getOrientation()
    RD_OrVec = RD_wing.getOrientation()
    
    rudder_OrVec = rudder.getOrientation()
    tail_OrVec   = TheFlapper.getOrientation()
    Flapper_OrVec = TheFlapper.getOrientation()
    
    LU_wing_Rotation_matrix = \
    np.matrix([[LU_OrVec[0], LU_OrVec[1], LU_OrVec[2]],\
               [LU_OrVec[3], LU_OrVec[4], LU_OrVec[5]],\
               [LU_OrVec[6], LU_OrVec[7], LU_OrVec[8]]])
               
    LD_wing_Rotation_matrix = \
    np.matrix([[LD_OrVec[0], LD_OrVec[1], LD_OrVec[2]],\
               [LD_OrVec[3], LD_OrVec[4], LD_OrVec[5]],\
               [LD_OrVec[6], LD_OrVec[7], LD_OrVec[8]]])
               
    RU_wing_Rotation_matrix = \
    np.matrix([[RU_OrVec[0], RU_OrVec[1], RU_OrVec[2]],\
               [RU_OrVec[3], RU_OrVec[4], RU_OrVec[5]],\
               [RU_OrVec[6], RU_OrVec[7], RU_OrVec[8]]])
               
    RD_wing_Rotation_matrix = \
    np.matrix([[RD_OrVec[0], RD_OrVec[1], RD_OrVec[2]],\
               [RD_OrVec[3], RD_OrVec[4], RD_OrVec[5]],\
               [RD_OrVec[6], RD_OrVec[7], RD_OrVec[8]]])
               
    rudder_Rotation_matrix = \
    np.matrix([[rudder_OrVec[0], rudder_OrVec[1], rudder_OrVec[2]],\
               [rudder_OrVec[3], rudder_OrVec[4], rudder_OrVec[5]],\
               [rudder_OrVec[6], rudder_OrVec[7], rudder_OrVec[8]]])
                   
    tail_Rotation_matrix = \
    np.matrix([[tail_OrVec[0], tail_OrVec[1], tail_OrVec[2]],\
               [tail_OrVec[3], tail_OrVec[4], tail_OrVec[5]],\
               [tail_OrVec[6], tail_OrVec[7], tail_OrVec[8]]])
               
    Flapper_Rotation_current = \
    np.matrix([[Flapper_OrVec[0], Flapper_OrVec[1], Flapper_OrVec[2]],\
               [Flapper_OrVec[3], Flapper_OrVec[4], Flapper_OrVec[5]],\
               [Flapper_OrVec[6], Flapper_OrVec[7], Flapper_OrVec[8]]])

    # LU_OrVec    = LU_wing.getOrientation()
    # LD_OrVec    = LD_wing.getOrientation()
    # RU_OrVec    = RU_wing.getOrientation()
    # RD_OrVec    = RD_wing.getOrientation()
    rudder_OrVec = rudder.getOrientation()
    
    Flapper_OrVec = TheFlapper.getOrientation()
    Flapper_OrVec_in_Euler =  R2EAR(Flapper_OrVec)
    # print('Flapper_OrVec:',Flapper_OrVec_in_Euler )
    
    # LU_RD_OrVec = LU_RD_joint.getOrientation()

    LU_bac.RequestWingOrientation2InertiaFrame (LU_OrVec)
    LD_bac.RequestWingOrientation2InertiaFrame (LD_OrVec)
    RU_bac.RequestWingOrientation2InertiaFrame (RU_OrVec)
    RD_bac.RequestWingOrientation2InertiaFrame (RD_OrVec)
    rudder_bac.RequestWingOrientation2InertiaFrame (rudder_OrVec)
    tail_bac.RequestWingOrientation2InertiaFrame (tail_OrVec)
    
    LU_bac.RequestWingPlaneDirection()
    LD_bac.RequestWingPlaneDirection()
    RU_bac.RequestWingPlaneDirection()
    RD_bac.RequestWingPlaneDirection()
    rudder_bac.RequestWingPlaneDirection()
    tail_bac.RequestWingPlaneDirection()
    
    LU_velVec = LU_wing.getVelocity()
    LD_velVec = LD_wing.getVelocity()
    RU_velVec = RU_wing.getVelocity()
    RD_velVec = RD_wing.getVelocity()
    rudder_velVec = rudder.getVelocity()
    tail_velVec = tail.getVelocity()
    
    
    TheFlapper_velVec = TheFlapper.getVelocity()
    # This function returns a vector containing exactly 6 values. 
    # The first three are respectively the linear velocities in the x, y and z direction. 
    # The last three are respectively the angular velocities around the x, y and z axes
    Flapper_Rotation_current_T = np.transpose(Flapper_Rotation_current)
    Flapper_Angular_velocity_current = np.matmul(Flapper_Rotation_current_T,\
         np.matrix([[TheFlapper_velVec[3]],[TheFlapper_velVec[4]],[TheFlapper_velVec[5]]]))
    # TheFlapper_velVec[0:3]
    # LU_RD_velVec = LU_RD_joint.getVelocity()
    # print('LU_velVec:',LU_velVec)
    # print('tail_velVec:',tail_velVec)
    
    Last_Real_motor_LU_wing_joint_sensor_value = Real_motor_LU_wing_joint_sensor_value
    Last_Real_motor_LD_wing_joint_sensor_value = Real_motor_LD_wing_joint_sensor_value
    Last_Real_motor_RU_wing_joint_sensor_value = Real_motor_RU_wing_joint_sensor_value
    Last_Real_motor_RD_wing_joint_sensor_value = Real_motor_RD_wing_joint_sensor_value
    Last_Real_motor_rudder_joint_sensor_value = Real_motor_rudder_joint_sensor_value
    
    
    Real_motor_LU_wing_joint_sensor_value = Real_motor_LU_wing_joint_sensor.getValue()
    Real_motor_LD_wing_joint_sensor_value = Real_motor_LD_wing_joint_sensor.getValue()
    Real_motor_RU_wing_joint_sensor_value = Real_motor_RU_wing_joint_sensor.getValue()
    Real_motor_RD_wing_joint_sensor_value = Real_motor_RD_wing_joint_sensor.getValue()
    Real_motor_rudder_joint_sensor_value  = Real_motor_rudder_joint_sensor.getValue()
    
    d_Real_motor_LU_wing_joint_sensor_value = (Real_motor_LU_wing_joint_sensor_value - Last_Real_motor_LU_wing_joint_sensor_value) /Simulation_Gap
    d_Real_motor_LD_wing_joint_sensor_value = (Real_motor_LD_wing_joint_sensor_value - Last_Real_motor_LD_wing_joint_sensor_value) /Simulation_Gap
    d_Real_motor_RU_wing_joint_sensor_value = (Real_motor_RU_wing_joint_sensor_value - Last_Real_motor_RU_wing_joint_sensor_value) /Simulation_Gap
    d_Real_motor_RD_wing_joint_sensor_value = (Real_motor_RD_wing_joint_sensor_value - Last_Real_motor_RD_wing_joint_sensor_value) /Simulation_Gap
    d_Real_motor_rudder_joint_sensor_value  = (Real_motor_rudder_joint_sensor_value - Last_Real_motor_rudder_joint_sensor_value) /Simulation_Gap
    
    # print('d_Real_motor_RU_wing_joint_sensor_value',d_Real_motor_RU_wing_joint_sensor_value)
    
    
    LU_bac.RequestVelocities(vel_FreeFlow,LU_velVec[0:3],\
                                          LU_velVec[3:6],d_Real_motor_LU_wing_joint_sensor_value)
    LD_bac.RequestVelocities(vel_FreeFlow,LD_velVec[0:3],\
                                          LD_velVec[3:6],d_Real_motor_LD_wing_joint_sensor_value)
    RU_bac.RequestVelocities(vel_FreeFlow,RU_velVec[0:3],\
                                          RU_velVec[3:6],d_Real_motor_RU_wing_joint_sensor_value)
    RD_bac.RequestVelocities(vel_FreeFlow,RD_velVec[0:3],\
                                          RD_velVec[3:6],d_Real_motor_RD_wing_joint_sensor_value)
    Period_length = int( round( 1 / StrokeFreq / Simulation_Gap ) )
    # print ('Period_length', Period_length)
    induced_velocity_record_length = induced_velocity_record.__len__()
    if ( Period_length < induced_velocity_record_length ):
        # print('Period_length', type(Period_length))
        # print('induced_velocity_record_length', type(induced_velocity_record_length))
        ave_of_vel_in_Period_length = \
            sum ( induced_velocity_record[ - Period_length : induced_velocity_record_length] ) \
            / Period_length
        # print('ave_of_vel_in_Period_length',ave_of_vel_in_Period_length)
        caudal_direction  = [0,0,-1]
        caudal_direction_in_inertia_raw = np. matmul( Flapper_Rotation_current, caudal_direction)
        caudal_direction_in_inertia     = np.array(caudal_direction_in_inertia_raw).flatten().tolist()
        # print('caudal_direction_in_inertia', caudal_direction_in_inertia)
        mag_in_caudal_ave_vel       = sum (np. multiply(ave_of_vel_in_Period_length, \
                                           caudal_direction_in_inertia ) )
        # mag_in_caudal_ave_vel           = mag_in_caudal_ave_vel_raw[0, 0]
        # print('mag_in_caudal_ave_vel', mag_in_caudal_ave_vel)
        if (mag_in_caudal_ave_vel < 0):
            mag_in_caudal_ave_vel = 0 
        
        trace_back_length = round( FromWing2Tail / (mag_in_caudal_ave_vel + FORCE_RELATIVECONSTANT) /   Simulation_Gap)
        if (trace_back_length < induced_velocity_record_length):
            Flapping_wing_induced_flow_Tail = induced_velocity_record[ - trace_back_length]
        else:
            Flapping_wing_induced_flow_Tail = 0
    while (induced_velocity_record_list_length < induced_velocity_record.__len__()):
        induced_velocity_record.pop(0)
    
    
    rudder_bac.RequestVelocities(vel_FreeFlow + Flapping_wing_induced_flow_Tail, rudder_velVec[0:3],\
                                          rudder_velVec[3:6],d_Real_motor_rudder_joint_sensor_value)
    
    DEBUGtail_FreeFlow                       = np.array([0,0,0])
    #set as 0 0 0 when everything ok
    tail_bac.RequestVelocities(vel_FreeFlow + DEBUGtail_FreeFlow, tail_velVec[0:3],\
                                          tail_velVec[3:6],0)
    
                                          
    # TheFlapper_velVec = TheFlapper.getVelocity()
    
    # print('Flapping_wing_induced_flow:',Flapping_wing_induced_flow)
    # print('itself_moving_flow:',rudder_velVec[0:3])
    # print('TheFlapper_velVec[0:3]:',TheFlapper_velVec[0:3])
    
                                        
    LU_bac.CalcEffectiveVelocity()
    LD_bac.CalcEffectiveVelocity()
    RU_bac.CalcEffectiveVelocity()
    RD_bac.CalcEffectiveVelocity()
    rudder_bac.CalcEffectiveVelocity()
    tail_bac.CalcEffectiveVelocity()
    
    LU_bac.CalcAoA()
    LD_bac.CalcAoA()
    RU_bac.CalcAoA()
    RD_bac.CalcAoA()
    rudder_bac.CalcAoA()
    tail_bac.CalcAoA()
    
    LU_bac.CopmputeAerodynamicForce()
    LD_bac.CopmputeAerodynamicForce()
    RU_bac.CopmputeAerodynamicForce()
    RD_bac.CopmputeAerodynamicForce()
    rudder_bac.CopmputeAerodynamicForce()
    tail_bac.CopmputeAerodynamicForce()

    # print('LU_r_shift:',LU_r_shift)
    # LU_r_shift = np.matmul(LU_wing_Rotation_matrix, np.array([LU_bac. X_pos_r, -LU_bac. Y_pos_r, 0]))
    LU_r_shift_in_wing = np.array([LU_bac. X_pos_r, LU_bac. Y_pos_r, 0])
    LU_r_shift = np.array(np.matmul(LU_wing_Rotation_matrix, LU_r_shift_in_wing)).squeeze().tolist()
    LU_r = np.array(np.sum(LU_bac. F_r, axis=0)).squeeze()
    LU_r_Axis = P2AA(LU_r).tolist()
    LU_r_position = (np.array(LU_wing.getPosition()) + np.array(LU_r_shift)).tolist()
    LU_r_norm     = np.linalg.norm(LU_r)
    LU_r_FAM.update_force_device(LU_r_Axis,LU_r_position,LU_r_norm)
    
    LU_a_shift_in_wing = np.array([LU_bac. X_pos_a, LU_bac. Y_pos_a, 0])
    LU_a_shift = np.array(np.matmul(LU_wing_Rotation_matrix, LU_a_shift_in_wing)).squeeze().tolist()
    LU_a = np.array(np.sum(LU_bac. F_a, axis=0)).squeeze()
    LU_a_Axis = P2AA(LU_a).tolist()
    LU_a_position = (np.array(LU_wing.getPosition()) + np.array(LU_a_shift)).tolist()
    LU_a_norm     = np.linalg.norm(LU_a)
    LU_a_FAM.update_force_device(LU_a_Axis,LU_a_position,LU_a_norm)

    LU_t_shift_in_wing = np.array([LU_bac. X_pos_t, LU_bac. Y_pos_t, 0])
    LU_t_shift = np.array(np.matmul(LU_wing_Rotation_matrix, LU_t_shift_in_wing)).squeeze().tolist()
    LU_drag = np.array(np.sum(LU_bac. F_t_drag, axis=0)).squeeze()
    LU_drag_Axis = P2AA(LU_drag).tolist()
    LU_drag_position = (np.array(LU_wing.getPosition()) + np.array(LU_t_shift)).tolist()
    LU_drag_norm     = np.linalg.norm(LU_drag)
    LU_drag_FAM.update_force_device(LU_drag_Axis,LU_drag_position,LU_drag_norm)
    # print('LU_t_shift_in_wing',LU_t_shift_in_wing)
    
    # print('LU_r_norm',LU_r_norm)
    # print('LU_a_norm',LU_a_norm)
    # print('LU_drag_norm:',LU_drag_norm)
    


    LU_lift = np.array(np.sum(LU_bac. F_t_lift, axis=0)).squeeze()
    LU_lift_Axis = P2AA(LU_lift).tolist()
    LU_lift_position = LU_drag_position
    LU_lift_norm     = np.linalg.norm(LU_lift)
    LU_lift_FAM.update_force_device(LU_lift_Axis,LU_lift_position,LU_lift_norm)
    
    # print('LU_lift_norm:',LU_lift_norm)
    
    LD_drag = np.array(np.sum(LD_bac. F_t_drag, axis=0)).squeeze()
    # LD_drag_Axis = P2AA(LD_drag).tolist()
    # LD_drag_position = LD_wing.getPosition()
    # LD_drag_norm     = np.linalg.norm(LD_drag)
    # LD_drag_FAM.update_force_device(LD_drag_Axis,LD_drag_position,LD_drag_norm)
    
    LD_lift = np.array(np.sum(LD_bac. F_t_lift, axis=0)).squeeze()
    # LD_lift_Axis = P2AA(LD_lift).tolist()
    # LD_lift_position = LD_drag_position
    # LD_lift_norm     = np.linalg.norm(LD_lift)
    # LD_lift_FAM.update_force_device(LD_lift_Axis,LD_lift_position,LD_lift_norm)
    
    RU_lift = np.array(np.sum(RU_bac. F_t_lift, axis=0)).squeeze()
    RD_lift = np.array(np.sum(RD_bac. F_t_lift, axis=0)).squeeze()
    
    RU_drag = np.array(np.sum(RU_bac. F_t_drag, axis=0)).squeeze()
    RD_drag = np.array(np.sum(RD_bac. F_t_drag, axis=0)).squeeze()
    
    # print('LD_drag_norm:',np.linalg.norm(LD_drag))
    # print('LD_lift_norm:',np.linalg.norm(LD_lift))
    # print('RU_lift_norm:',np.linalg.norm(RU_lift))
    # print('RD_lift_norm:',np.linalg.norm(RD_lift))
    # print('RU_drag_norm:',np.linalg.norm(RU_drag))
    # print('RD_drag_norm:',np.linalg.norm(RD_drag))
    
    LD_r = np.array(np.sum(LD_bac. F_r, axis=0)).squeeze()
    RU_r = np.array(np.sum(RU_bac. F_r, axis=0)).squeeze()
    RD_r = np.array(np.sum(RD_bac. F_r, axis=0)).squeeze()
    
    # print('LD_r_norm:',np.linalg.norm(LD_r))
    # print('RU_r_norm:',np.linalg.norm(RU_r))
    # print('RD_r_norm:',np.linalg.norm(RD_r))
    # LD_r_Axis = P2AA(LD_r).tolist()
    
    LD_a = np.array(np.sum(LD_bac. F_a, axis=0)).squeeze()
    RU_a = np.array(np.sum(RU_bac. F_a, axis=0)).squeeze()
    RD_a = np.array(np.sum(RD_bac. F_a, axis=0)).squeeze()

    # print('LD_a_norm:',np.linalg.norm(LD_a))
    # print('RU_a_norm:',np.linalg.norm(RU_a))
    # print('RD_a_norm:',np.linalg.norm(RD_a))    
    
    rudder_lift = np.array(np.sum(rudder_bac. F_t_lift, axis=0)).squeeze()
    rudder_drag = np.array(np.sum(rudder_bac. F_t_drag, axis=0)).squeeze()
    rudder_r = np.array(np.sum(rudder_bac. F_r, axis=0)).squeeze()
    rudder_a = np.array(np.sum(rudder_bac. F_a, axis=0)).squeeze()
    
    # print('rudder_lift',rudder_lift)
    # print('rudder_drag',rudder_drag)  
    rudder_t_shift_in_local = np.array([rudder_bac. X_pos_t, rudder_bac. Y_pos_t, 0]).squeeze()
    
    rudder_t_shift = np.array(np.matmul(rudder_Rotation_matrix, rudder_t_shift_in_local)).squeeze().tolist()
    # LU_drag = np.array(np.sum(rudder_bac. F_t_drag, axis=0)).squeeze()
    rudder_drag_Axis = P2AA(rudder_drag).tolist()
    rudder_drag_position = (np.array(rudder.getPosition()) + np.array(rudder_t_shift)).tolist()
    rudder_drag_norm     = np.linalg.norm(rudder_drag)
    rudder_drag_FAM.update_force_device(rudder_drag_Axis,rudder_drag_position,rudder_drag_norm)
    # rudder_t_shift_in_local_to_real_wing
    # print('rudder_t_shift_in_local',rudder_t_shift_in_local)


    tail_lift = np.array(np.sum(tail_bac. F_t_lift, axis=0)).squeeze()
    tail_drag = np.array(np.sum(tail_bac. F_t_drag, axis=0)).squeeze()
    tail_r = np.array(np.sum(tail_bac. F_r, axis=0)).squeeze()
    tail_a = np.array(np.sum(tail_bac. F_a, axis=0)).squeeze()
    
    
    # print('tail_lift',tail_lift)
    # print('tail_drag',tail_drag)  
    JUSTFOTAILRDEBUG = 1
    
    tail_t_shift_in_local = np.array([tail_bac. X_pos_t, tail_bac. Y_pos_t, 0]).squeeze()
    
    
    tail_t_shift_raw      = -np.array(np.matmul(np.transpose(tail_Rotation_matrix_VP2W), tail_t_shift_in_local)).squeeze()
    # print('tail_t_shift_raw:',tail_t_shift_raw)
    tail_t_shift = np.array(np.matmul(tail_Rotation_matrix, tail_t_shift_raw + tail_shift_in_Flapper)).squeeze().tolist()
    # LU_drag = np.array(np.sum(rudder_bac. F_t_drag, axis=0)).squeeze()
    tail_drag_Axis = P2AA(tail_drag).tolist()
    tail_drag_position = (np.array(tail.getPosition()) + np.array(tail_t_shift)).tolist()
    # print("tail.getPosition()", tail.getPosition())
    # print("tail_drag_position", tail_drag_position)
    tail_drag_norm     = JUSTFOTAILRDEBUG * np.linalg.norm(tail_drag)
    # print('tail_drag_norm',tail_drag_norm)
    tail_drag_FAM. update_force_device(tail_drag_Axis,tail_drag_position,tail_drag_norm)
   
    # LD_a_Axis = P2AA(LD_a).tolist()
    # print ('LD_drag:',LD_drag,'Axis:',LD_Axis)
    # LD_drag_arrow_rotation.setSFRotation(LD_Axis)
    # LD_drag_arrow_translation.setSFVec3f(LD_wing.getPosition())
    # print('Drag:',[LU_drag[0],LU_drag[1],LU_drag[2]])
    JUSTFORDEBUG = 1 ## For magnify some 
    
    rudder_lift_position = tail_drag_position
    
    LU_wing.addForceWithOffset([LU_drag[0],LU_drag[1],LU_drag[2]],[LU_bac.X_pos_t,LU_bac.Y_pos_t,0],False)
    LD_wing.addForceWithOffset([LD_drag[0],LD_drag[1],LD_drag[2]],[LD_bac.X_pos_t,LD_bac.Y_pos_t,0],False)
    RU_wing.addForceWithOffset([RU_drag[0],RU_drag[1],RU_drag[2]],[RU_bac.X_pos_t,RU_bac.Y_pos_t,0],False)
    RD_wing.addForceWithOffset([RD_drag[0],RD_drag[1],RD_drag[2]],[RD_bac.X_pos_t,RD_bac.Y_pos_t,0],False)
    rudder .addForceWithOffset([rudder_drag[0] * JUSTFORDEBUG,rudder_drag[1] * JUSTFORDEBUG,rudder_drag[2] * JUSTFORDEBUG],rudder_t_shift_in_local.tolist(),False)
    tail   .addForceWithOffset([tail_drag[0]* JUSTFOTAILRDEBUG,tail_drag[1]* JUSTFOTAILRDEBUG,tail_drag[2]* JUSTFOTAILRDEBUG] ,(tail_t_shift_raw + tail_shift_in_Flapper) .tolist() ,False)
    # print('LU_trans_pos:',[LU_bac.X_pos_t,LU_bac.Y_pos_t,0] )
    
    LU_wing.addForceWithOffset([LU_lift[0],LU_lift[1],LU_lift[2]],[LU_bac.X_pos_t,LU_bac.Y_pos_t,0],False)
    LD_wing.addForceWithOffset([LD_lift[0],LD_lift[1],LD_lift[2]],[LD_bac.X_pos_t,LD_bac.Y_pos_t,0],False)
    RU_wing.addForceWithOffset([RU_lift[0],RU_lift[1],RU_lift[2]],[RU_bac.X_pos_t,RU_bac.Y_pos_t,0],False)
    RD_wing.addForceWithOffset([RD_lift[0],RD_lift[1],RD_lift[2]],[RD_bac.X_pos_t,RD_bac.Y_pos_t,0],False)
    rudder .addForceWithOffset([rudder_lift[0] * JUSTFORDEBUG,rudder_lift[1] * JUSTFORDEBUG,rudder_lift[2] * JUSTFORDEBUG],[rudder_bac.X_pos_t,-rudder_bac.Y_pos_t,0],False)
    tail   .addForceWithOffset([tail_lift[0] * JUSTFOTAILRDEBUG,tail_lift[1] * JUSTFOTAILRDEBUG,tail_lift[2] * JUSTFOTAILRDEBUG] ,(tail_t_shift_raw + tail_shift_in_Flapper) .tolist(),False)
    
    
    LU_lift_in_real = np.array(np.matmul(Flapper_Rotation_current_T, LU_lift)).squeeze()
    LD_lift_in_real = np.array(np.matmul(Flapper_Rotation_current_T, LD_lift)).squeeze()
    RU_lift_in_real = np.array(np.matmul(Flapper_Rotation_current_T, RU_lift)).squeeze()
    RD_lift_in_real = np.array(np.matmul(Flapper_Rotation_current_T, RD_lift)).squeeze()
    
    Total_lift_in_real =  (LU_lift_in_real + LD_lift_in_real + RU_lift_in_real + RD_lift_in_real)
    
    # print('TOTAL_LIFT', [Total_lift_in_real[0],Total_lift_in_real[1],Total_lift_in_real[2]])
    
    Flapping_wing_induced_flow_raw_list = - 0.5 * GetUnitDirection_Safe(Total_lift_in_real) * \
                                np.sqrt( 0.5 * np.linalg.norm(Total_lift_in_real) /\
                                         S_d_Flapping_wing_actuator_disk_area / arena_Air_Density )
    
    Flapping_wing_induced_flow_raw_mat  = np.matrix([[Flapping_wing_induced_flow_raw_list[0]],\
                                                     [Flapping_wing_induced_flow_raw_list[1]],\
                                                     [Flapping_wing_induced_flow_raw_list[2]]])
    Flapping_wing_induced_flow = 2 * np.array(np.matmul(Flapper_Rotation_current, Flapping_wing_induced_flow_raw_mat)).squeeze()
    # print('Flapping_wing_induced_flow', Flapping_wing_induced_flow)
    
    induced_velocity_record.append( Flapping_wing_induced_flow )
    
    LU_wing.addForceWithOffset([LU_r[0],LU_r[1],LU_r[2]],[LU_bac.X_pos_r,LU_bac.Y_pos_r,0],False)
    LD_wing.addForceWithOffset([LD_r[0],LD_r[1],LD_r[2]],[LD_bac.X_pos_r,LD_bac.Y_pos_r,0],False)
    RU_wing.addForceWithOffset([RU_r[0],RU_r[1],RU_r[2]],[RU_bac.X_pos_r,RU_bac.Y_pos_r,0],False)
    RD_wing.addForceWithOffset([RD_r[0],RD_r[1],RD_r[2]],[RD_bac.X_pos_r,RD_bac.Y_pos_r,0],False)
    
    # rudder. addForceWithOffset([rudder_r[0],rudder_r[1],rudder_r[2]],[rudder_bac.X_pos_r,rudder_bac.Y_pos_r,0],False)
    # tail.   addForceWithOffset([tail_r[0],tail_r[1],tail_r[2]],(np.array([tail_bac.X_pos_r,tail_bac.Y_pos_r,0])+ tail_shift_in_Flapper ).tolist(),False)
    # print("[tail_bac.X_pos_r,tail_bac.Y_pos_r,0]+ tail_shift_in_Flapper .tolist()",[tail_bac.X_pos_r,tail_bac.Y_pos_r,0]+ tail_shift_in_Flapper .tolist())


    LU_wing.addForceWithOffset([LU_a[0],LU_a[1],LU_a[2]],[LU_bac.X_pos_a,LU_bac.Y_pos_a,0],False)
    LD_wing.addForceWithOffset([LD_a[0],LD_a[1],LD_a[2]],[LD_bac.X_pos_a,LD_bac.Y_pos_a,0],False)
    RU_wing.addForceWithOffset([RU_a[0],RU_a[1],RU_a[2]],[RU_bac.X_pos_a,RU_bac.Y_pos_a,0],False)
    RD_wing.addForceWithOffset([RD_a[0],RD_a[1],RD_a[2]],[RD_bac.X_pos_a,RD_bac.Y_pos_a,0],False)
    
    # rudder.addForceWithOffset([rudder_a[0],rudder_a[1],rudder_a[2]],[rudder_bac.X_pos_a,rudder_bac.Y_pos_a,0],False)
    # tail.   addForceWithOffset([tail_a[0],tail_a[1],tail_a[2]],(np.array([tail_bac.X_pos_a,tail_bac.Y_pos_a,0])+ tail_shift_in_Flapper ).tolist(),False)
   
   
   ####-------------Control Tasks-------------------
    # print('d_Real_motor_LU_wing_joint_sensor_value',d_Real_motor_LU_wing_joint_sensor_value)
    
    torsion_spring_constant = 0.025
    # torsion_spring_yaw_offset = 0 ##0-0.3
    
    # K_pitch = 5
    # torsion_spring_pitch_offset =  K_pitch * (Flapper_OrVec_in_Euler[1]-0.6)### 仔细调整
    
    
    K_mo = 0.15
    
     
    
    #first order filter
    # Filtered_anular_velocity = Filtered_anular_velocity + 0.01 * (Flapper_Angular_velocity_current - Filtered_anular_velocity)
    
    #second order filter update
    Angular_velocity_filter. march_forward(Flapper_Angular_velocity_current)
    
    #FT observer update
    Flapper_translation_value = Flapper_translation.getSFVec3f()
    Flapper_pos             = np.mat( [[Flapper_translation_value[0]],\
                                       [Flapper_translation_value[1]],\
                                       [Flapper_translation_value[2]]]) 
    
    
    u_t_in_body_fixed_frame = np.mat( [[0],[0],[0.2]])  
    u_t_in_inertia_frame = Flapper_Rotation_current * u_t_in_body_fixed_frame 
    # print('u_t_in_inertia_frame',u_t_in_inertia_frame)
    
    Here_pos_observer. march_forward(u_t_in_inertia_frame, Flapper_pos)
    
   
    # print('z_har:', Here_pos_observer. z_observer)s
    # SO3_Attitude_Controller. Auto_generate_d_Omega_desired(Flapper_Angular_velocity_desired)
    k_rate  = 2
    t_rate  = 0.2
    p_d_z = 0
    v_d_z = 0
    d_v_d_z = 0
    
    
    # Circular Flight
    p_d_x   = - k_rate *  np.cos ( np.pi * t_rate * RecordCount * Simulation_Gap ) + k_rate 
    p_d_y   =   k_rate *  np.sin ( np.pi * t_rate * RecordCount * Simulation_Gap )
   
    v_d_x   =   k_rate *  np.pi * t_rate * np.sin ( np.pi * t_rate * RecordCount * Simulation_Gap )
    v_d_y   =   k_rate *  np.pi * t_rate * np.cos ( np.pi * t_rate * RecordCount * Simulation_Gap )
    
     
    d_v_d_x =   k_rate *  np.pi * t_rate *  np.pi * t_rate * np.cos ( np.pi * t_rate * RecordCount * Simulation_Gap )
    d_v_d_y = - k_rate *  np.pi * t_rate *  np.pi * t_rate * np.sin ( np.pi * t_rate * RecordCount * Simulation_Gap )
    
    # Lemniscate Flight
    # L_theta = np.pi * t_rate * RecordCount * Simulation_Gap 
    # p_d_x   =  k_rate - k_rate * np. cos(L_theta) / ( 1 + np. sin(L_theta) * np. sin(L_theta) )
    # p_d_y   =  k_rate * np. sin(L_theta) * np. cos(L_theta) / (1 + np. sin(L_theta) * np. sin(L_theta))
    
    # v_d_x   = ( p_d_x - p_d_x_last ) / Simulation_Gap
    # v_d_y   = ( p_d_y - p_d_y_last ) / Simulation_Gap    
    # p_d_x_last = p_d_x
    # p_d_y_last = p_d_y
    
    # d_v_d_x = ( v_d_x - v_d_x_last ) / Simulation_Gap
    # d_v_d_y = ( v_d_y - v_d_y_last ) / Simulation_Gap
    # v_d_x_last = v_d_x
    # v_d_y_last = v_d_y
    
    # P2P Flight
    # if (RecordCount * Simulation_Gap > 2):
        # if (RecordCount * Simulation_Gap <= 8):
            # p_d_x = 2
            # p_d_y = 2
            # p_d_z = 2
            # v_d_x = 0.5
            # v_d_y = 0.5
            # v_d_z = 0.5
        # else:
            # p_d_x = 0
            # p_d_y = 0
            # p_d_z = 0
            # v_d_x = -0.5
            # v_d_y = -0.5
            # v_d_z = -0.5
        
    
    
    p_d     = np.mat([  [p_d_x],    [p_d_y], [p_d_z]])
    v_d     = np.mat([  [v_d_x],    [v_d_y], [v_d_z]])
    d_v_d   = np.mat([  [d_v_d_x],  [d_v_d_y], [d_v_d_z]])
    
    
    
    
    Torward_direction = np.mat([  [0], [1], [0]])
    Flight_direction  = np.mat([  [v_d_x], [v_d_y], [0]])
    
    
    Here_ARG. match_forward_rotation(R_d)
    
    if ( np.mod(RecordCount, Controller_Gap_vs_Simulation_Gap)==0):
    # ####step attitude
        # if ( np.mod(RecordCount, slerp)==0):        
            # Here_ATG. orientation = np.matmul(Flapper_Rotation_matrix_initial,Given_attitude_list[np.mod(Given_attitude_count, 6)],\
                                # )
            # Given_attitude_count = Given_attitude_count + 1
    
    ###sinusoidal   
        # omega_1 = 0.5 * np.pi * np.cos(RecordCount * Simulation_Gap * np.pi)
        # omega_2 = 0.5 * np.pi * np.cos(RecordCount * Simulation_Gap * np.pi )
        # omega_3 = 0.5 * np.pi * np.cos(RecordCount * Simulation_Gap * np.pi )
        # Here_ATG. omega = np. mat([[omega_1],[omega_2],[omega_3]])
        print('Flapper_pos',Flapper_pos)
        print('p_observer:',Here_pos_observer. p_observer)
        print('p_d:',p_d) 
        # print('Here_ATG. orientation:',Here_ATG. orientation)
       
        # Here_ATG. march_forward(Here_ATG. orientation, Here_ATG. omega)
        
        K_pitch = 1
        K_yaw   = 0.3
        
        if (is_filtered  == 1):
            Angle_vel = Angular_velocity_filter. Get_filtered()
        else:
            Angle_vel =  Flapper_Angular_velocity_current
        
        
        u_t = Postion_Controller. Calc_u_t( p_d,   Here_pos_observer. p_observer, \
                                            v_d,   Here_pos_observer. v_observer, \
                                            d_v_d, Here_pos_observer. z_observer,\
                                            Flapper_Rotation_current)
        
        print('u_t',u_t)
        StrokeFreq =  11 / 9.8* np.linalg.norm (u_t)
        
        # StrokeFreq = -K_height * (Flapper_translation_value[1]-1.2) + 10
        if (StrokeFreq < 8):
            StrokeFreq = 8
        if (StrokeFreq > 15):
            StrokeFreq = 15
        
        print('StrokeFreq',StrokeFreq)
        R_d = Computing_desired_rotation( u_t, Torward_direction, Flight_direction)
        Here_ARG. match_forward_angular_velcoity(R_d)
        print('R_d', R_d)
        print('Here_ARG.R_f', Here_ARG.R_f)
        print('Here_ARG.Omega_f', Here_ARG.Omega_f)
        ##Omega_static
        SO3_Attitude_Controller. Generate_control_signal( Flapper_Rotation_current, Angle_vel,\
                                         R_d, Here_ARG.Omega_f)
        torsion_spring_pitch_offset =  - K_pitch * SO3_Attitude_Controller.u[1,0]
        torsion_spring_yaw_offset   =  K_yaw * SO3_Attitude_Controller.u[2,0]
        
        
        
        K_roll_mix = 0.5
        roll_rudder_amplitude = -K_roll_mix * SO3_Attitude_Controller.u[0,0]
        if (roll_rudder_amplitude > 0.8):
            roll_rudder_amplitude = 0.8
        else:
            if (roll_rudder_amplitude < -0.8):
                roll_rudder_amplitude = -0.8
                
        
        Real_motor_rudder_joint. setPosition(roll_rudder_amplitude)
        
        # print('SO3_Attitude_Controller.u',SO3_Attitude_Controller.u)
        
        if (torsion_spring_pitch_offset >0.7):
            torsion_spring_pitch_offset =0.7
        else:
            if (torsion_spring_pitch_offset < -0.7):
                torsion_spring_pitch_offset = -0.7
                
        if (torsion_spring_yaw_offset > 0.3):
            torsion_spring_yaw_offset = 0.3
        else:
            if (torsion_spring_yaw_offset < -0.3):
                torsion_spring_yaw_offset = -0.3
                
        print('torsion_spring_yaw_offset',torsion_spring_yaw_offset)
    
    
    # print('Flapper position', Flapper_translation_value)
    
    K_height = 5
    
        
    # print('Height:', Flapper_translation_value[1])
    # print('StrokeFreq',StrokeFreq)
    # StrokeFreq = 15
            
    # print('Flapper_OrVec_in_Euler[1],', Flapper_OrVec_in_Euler[1])    
    # print('TEST: torsion_spring_pitch_offset,', torsion_spring_pitch_offset)
    
    Real_motor_LU_wing_joint.setTorque(- torsion_spring_constant \
                                       * (Real_motor_LU_wing_joint_sensor_value - torsion_spring_yaw_offset - torsion_spring_pitch_offset))
    Real_motor_LD_wing_joint.setTorque(- torsion_spring_constant \
                                       * (Real_motor_LD_wing_joint_sensor_value - torsion_spring_yaw_offset - torsion_spring_pitch_offset))
    Real_motor_RU_wing_joint.setTorque(- torsion_spring_constant \
                                       * (Real_motor_RU_wing_joint_sensor_value - torsion_spring_yaw_offset + torsion_spring_pitch_offset))
    Real_motor_RD_wing_joint.setTorque(- torsion_spring_constant \
                                       * (Real_motor_RD_wing_joint_sensor_value - torsion_spring_yaw_offset + torsion_spring_pitch_offset))
  
    # print('StrokeFreq:',StrokeFreq)
    
    LD_t_shift_in_wing = np.array([LD_bac. X_pos_t, LD_bac. Y_pos_t, 0])
    LD_t_shift = np.array(np.matmul(LD_wing_Rotation_matrix, LD_t_shift_in_wing)).squeeze().tolist()
    LD_t_position = (np.array(LD_wing.getPosition()) + np.array(LD_t_shift)).tolist()
    
    LD_r_shift_in_wing = np.array([LD_bac. X_pos_r, LD_bac. Y_pos_r, 0])
    LD_r_shift = np.array(np.matmul(LD_wing_Rotation_matrix, LD_r_shift_in_wing)).squeeze().tolist()
    LD_r_position = (np.array(LD_wing.getPosition()) + np.array(LD_r_shift)).tolist()
    
    LD_a_shift_in_wing = np.array([LD_bac. X_pos_a, LD_bac. Y_pos_a, 0])
    LD_a_shift = np.array(np.matmul(LD_wing_Rotation_matrix, LD_a_shift_in_wing)).squeeze().tolist()
    LD_a_position = (np.array(LD_wing.getPosition()) + np.array(LD_a_shift)).tolist()
    
    
    
    RU_t_shift_in_wing = np.array([RU_bac. X_pos_t, RU_bac. Y_pos_t, 0])
    RU_t_shift = np.array(np.matmul(RU_wing_Rotation_matrix, RU_t_shift_in_wing)).squeeze().tolist()
    RU_t_position = (np.array(RU_wing.getPosition()) + np.array(RU_t_shift)).tolist()
    
    RU_r_shift_in_wing = np.array([RU_bac. X_pos_r, RU_bac. Y_pos_r, 0])
    RU_r_shift = np.array(np.matmul(RU_wing_Rotation_matrix, RU_r_shift_in_wing)).squeeze().tolist()
    RU_r_position = (np.array(RU_wing.getPosition()) + np.array(RU_r_shift)).tolist()
    
    RU_a_shift_in_wing = np.array([RU_bac. X_pos_a, RU_bac. Y_pos_a, 0])
    RU_a_shift = np.array(np.matmul(RU_wing_Rotation_matrix, RU_a_shift_in_wing)).squeeze().tolist()
    RU_a_position = (np.array(RU_wing.getPosition()) + np.array(RU_a_shift)).tolist()
    
    
    
    RD_t_shift_in_wing = np.array([RD_bac. X_pos_t, RD_bac. Y_pos_t, 0])
    RD_t_shift = np.array(np.matmul(RD_wing_Rotation_matrix, RD_t_shift_in_wing)).squeeze().tolist()
    RD_t_position = (np.array(RD_wing.getPosition()) + np.array(RD_t_shift)).tolist()
    
    RD_r_shift_in_wing = np.array([RD_bac. X_pos_r, RD_bac. Y_pos_r, 0])
    RD_r_shift = np.array(np.matmul(RD_wing_Rotation_matrix, RD_r_shift_in_wing)).squeeze().tolist()
    RD_r_position = (np.array(RD_wing.getPosition()) + np.array(RD_r_shift)).tolist()
    
    RD_a_shift_in_wing = np.array([RD_bac. X_pos_a, RD_bac. Y_pos_a, 0])
    RD_a_shift = np.array(np.matmul(RD_wing_Rotation_matrix, RD_a_shift_in_wing)).squeeze().tolist()
    RD_a_position = (np.array(RD_wing.getPosition()) + np.array(RD_a_shift)).tolist()
    
    #### torque
    rudder_drag_position = (np.array(rudder.getPosition()) + np.array(rudder_t_shift)).tolist()
    rudder_lift_position = rudder_drag_position

    
    LU_lift_torque = CTR (LU_lift, np.array(Flapper_translation_value) - np.array(LU_lift_position))
    LU_drag_torque = CTR (LU_drag, np.array(Flapper_translation_value) - np.array(LU_drag_position))
    LU_r_torque    = CTR (LU_r,    np.array(Flapper_translation_value) - np.array(LU_r_position))
    LU_a_torque    = CTR (LU_a,    np.array(Flapper_translation_value) - np.array(LU_a_position))
    
    LD_lift_torque = CTR (LD_lift, np.array(Flapper_translation_value) - np.array(LD_t_position))
    LD_drag_torque = CTR (LD_drag, np.array(Flapper_translation_value) - np.array(LD_t_position))
    LD_r_torque    = CTR (LD_r,    np.array(Flapper_translation_value) - np.array(LD_r_position))
    LD_a_torque    = CTR (LD_a,    np.array(Flapper_translation_value) - np.array(LD_a_position))
    
    RU_lift_torque = CTR (RU_lift, np.array(Flapper_translation_value) - np.array(RU_t_position))
    RU_drag_torque = CTR (RU_drag, np.array(Flapper_translation_value) - np.array(RU_t_position))
    RU_r_torque    = CTR (RU_r,    np.array(Flapper_translation_value) - np.array(RU_r_position))
    RU_a_torque    = CTR (RU_a,    np.array(Flapper_translation_value) - np.array(RU_a_position))
    
    RD_lift_torque = CTR (RD_lift, np.array(Flapper_translation_value) - np.array(RD_t_position))
    RD_drag_torque = CTR (RD_drag, np.array(Flapper_translation_value) - np.array(RD_t_position))
    RD_r_torque    = CTR (RD_r,    np.array(Flapper_translation_value) - np.array(RD_r_position))
    RD_a_torque    = CTR (RD_a,    np.array(Flapper_translation_value) - np.array(RD_a_position))
    
    
    Total_wing_torque = LU_lift_torque + LU_drag_torque + LU_r_torque + LU_a_torque +\
                        LD_lift_torque + LD_drag_torque + LD_r_torque + LD_a_torque +\
                        RU_lift_torque + RU_drag_torque + RU_r_torque + RU_a_torque +\
                        RD_lift_torque + RD_drag_torque + RD_r_torque + RD_a_torque
    
    Total_wing_force  = LU_lift+ LU_drag + LU_r + LU_a +\
                        LD_lift + LD_drag + LD_r + LD_a +\
                        RU_lift + RU_drag + RU_r + RU_a +\
                        RD_lift + RD_drag + RD_r + RD_a
                        
    # print('Total_Torque:', Total_wing_torque)
    
    rudder_lift_torque  = CTR (rudder_lift, np.array(Flapper_translation_value) - np.array(rudder_lift_position))
    rudder_drag_torque  = CTR (rudder_drag, np.array(Flapper_translation_value) - np.array(rudder_drag_position))
    
    Total_rudder_torque = rudder_lift_torque + rudder_drag_torque   
    Total_rudder_force  = rudder_lift + rudder_drag
    
    
    tail_lift_torque    = CTR (tail_lift, np.array(Flapper_translation_value) - np.array(tail_drag_position))
    tail_drag_torque    = CTR (tail_drag, np.array(Flapper_translation_value) - np.array(tail_drag_position))
    Total_tail_torque   = tail_lift_torque + tail_lift_torque
    
    Total_tail_force    = tail_lift + tail_drag
    
    if (RecordCount < mat_save_length):
        Total_wing_torque_list.      append(Total_wing_torque)
        Total_wing_force_list.       append(Total_wing_force)
        Total_rudder_torque_list.    append(Total_rudder_torque)
        Total_rudder_force_list.     append(Total_rudder_force)
        Total_tail_torque_list.      append(Total_tail_torque)
        Total_tail_force_list.       append(Total_tail_force)
        
        Total_body_rotation_list.    append(Flapper_Rotation_current)
        Total_body_translation_list. append(Flapper_translation_value)
        Total_body_translation_desire. append(p_d)
        
        Total_body_angular_velocity_list. append(Flapper_Angular_velocity_current)
        Total_Angular_velocity_filtered. append(Angular_velocity_filter. Get_filtered())
        
        Total_desired_rotation_list. append(R_d)
        Total_desired_angular_velocity_list. append(Here_ARG.Omega_f)
        Total_psi_rotation_error_list. append(\
            SO3_Attitude_Controller. get_Psi(Flapper_Rotation_current, R_d))
        
        Total_yaw_input. append(torsion_spring_yaw_offset)
        Total_roll_input. append(roll_rudder_amplitude)
        Total_pitch_input. append(torsion_spring_pitch_offset)
        
        Total_Freq_stroke. append(StrokeFreq)
        
    if (RecordCount == mat_save_length + 1):
        file_name = This_experiemnt_name + Desired_Angular_velocity 
        scio.savemat(file_name+Total_File, { 'Total_wing_torque':Total_wing_torque_list,\
                                  'Total_wing_force':Total_wing_force_list,
                                  'Total_rudder_torque':Total_rudder_torque_list,\
                                  'Total_rudder_force':Total_rudder_force_list,\
                                  'Total_tail_torque':Total_tail_torque_list,\
                                  'Total_tail_force':Total_tail_force_list,\
                                  'Total_body_rotation_list':Total_body_rotation_list,\
                                  'Total_body_translation_list':Total_body_translation_list,\
                                  'Total_body_translation_desire':Total_body_translation_desire,\
                                  'Total_body_angular_velocity_list':Total_body_angular_velocity_list,\
                                  'Total_desired_rotation_list':Total_desired_rotation_list,\
                                  'Total_desired_angular_velocity_list':Total_desired_angular_velocity_list,\
                                  'Total_psi_rotation_error_list':Total_psi_rotation_error_list,\
                                  'Total_yaw_input':Total_yaw_input,\
                                  'Total_roll_input':Total_roll_input,\
                                  'Total_pitch_input':Total_pitch_input,\
                                  'Total_Angular_velocity_filtered_list':Total_Angular_velocity_filtered,\
                                  'Total_Freq_stroke':Total_Freq_stroke}) 
        Total_wing_torque_list.      clear()
        Total_wing_force_list.       clear()
        Total_rudder_torque_list.    clear()
        Total_rudder_force_list.     clear()
        Total_tail_torque_list.      clear()
        Total_tail_force_list.       clear()
        
        Total_body_rotation_list.    clear()
        Total_body_translation_list. clear()
        Total_body_translation_desire. clear()
        Total_body_angular_velocity_list. clear()
        
        Total_desired_rotation_list. clear()
        Total_desired_angular_velocity_list. clear()
        Total_psi_rotation_error_list. clear()
        
        Total_yaw_input. clear()
        Total_roll_input. clear()
        Total_pitch_input. clear()
        
        Total_Freq_stroke. clear()
    
    RecordCount = RecordCount + 1
    # print('LU_bac.AoA[30,0]',RD_bac.Rec_F_t_lift_amp[30,0])
    # if CountKit < 1000:
        # CountKit = CountKit + 1
        # dAOAfile.write(str(LU_bac.Rec_d_AoA[30,0])+'\n')
    
    # if CountKit == 1000:
        # dAOAfile.close()
    # print('Real_motor_LU_wing_joint_sensor:',Real_motor_LU_wing_joint_sensor_value)
    # print('Real_motor_LD_wing_joint_sensor:',Real_motor_LD_wing_joint_sensor_value)
    # print('Real_motor_RU_wing_joint_sensor:',Real_motor_RU_wing_joint_sensor_value)
    # print('Real_motor_RD_wing_joint_sensor:',Real_motor_RD_wing_joint_sensor_value)
    # Real_motor_LU_RD_device.setVelocity(0.1)
    # Real_motor_LU_RD_device.setPosition(300)
    # KK = Velocity_test_solid.getVelocity()
    # print(KK)
    # wb_motor_set_position(Real_motor_LU_RD_device, -1)
    # count = count + 0.01
    # count = count % 0.4
    # force = [0, 0.2, 0]
    # Velocity_test_solid.addForceWithOffset(force, [0.1, -0.05, 0], False)
    # Test_baselink.addForceWithOffset(force,[0, 0, -0.2],True)
# np.sum(bladeAeroCalculator. F_t_lift, axis=0)
    # print('self. F_t_lift:', np.sum(LU_bac. F_t_lift, axis=0))
    # print('self. F_t_drag:',  np.sum(LU_bac. F_t_drag, axis=0))
    # print('self. F_r:',  np.sum(LU_bac. F_r, axis=0))
    # print('self. F_a:',  np.sum(LU_bac. F_a, axis=0))
# dAOAfile.close()

# Enter here exit cleanup code.
