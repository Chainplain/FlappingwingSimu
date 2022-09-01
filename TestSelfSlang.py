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
from   ArrowManager import ForceArrowManager as FAM

# from controller import Node
# create the Robot instance.
############### OVerall Test Region ############




############### OVerall Test Region ############
flapper = Supervisor()

FORCE_RELATIVECONSTANT = 0.0005
# dAOAfile = open("FirstTestdAOA.txt", 'a')
# dAOAfile.truncate(0)
# drawer  = Display('mirror')

LU_bac = BAC('FlapallnewDownWing20BLE_X_pos.npy',\
                'FlapallnewDownWing20BLE_Y_pos.npy',bladeElementNo=20)
LD_bac = BAC('FlapallnewDownWing20BLE_X_pos.npy',\
                'FlapallnewDownWing20BLE_Y_pos.npy',bladeElementNo=20)
RU_bac = BAC('FlapallnewDownWing20BLE_X_pos.npy',\
                'FlapallnewDownWing20BLE_Y_pos.npy',bladeElementNo=20)
RD_bac = BAC('FlapallnewDownWing20BLE_X_pos.npy',\
                'FlapallnewDownWing20BLE_Y_pos.npy',bladeElementNo=20)

rudder_bac = BAC('FlapallnewDownTail_V20BLE_X_pos.npy',\
                'FlapallnewDownTail_V20BLE_Y_pos.npy',bladeElementNo=20)

LU_bac.SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
LD_bac.SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
RU_bac.SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
RD_bac.SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
rudder_bac.SetVirturalWingPlaneRelative2Wing([0,1,0,-1,0,0,0,0,1])

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
Real_motor_rudder_joint = flapper.getDevice('rudder_joint')


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


# print('Get Real_motor_LU_RD_joint position:',\
                     # Real_motor_LU_RD_joint_sensor.getValue())

# The sensor can only be read in the while loop Since only then it is sampled.

LU_wing                 = flapper.getFromDef('LU')
LD_wing                 = flapper.getFromDef('LD')
RU_wing                 = flapper.getFromDef('RU')
RD_wing                 = flapper.getFromDef('RD')
rudder                  = flapper.getFromDef('rudder')


TheFlapper                 = flapper.getFromDef('TheFlapper')
# LU_RD_joint             = flapper.getFromDef('LU_RD')


LU_drag_arrow             = flapper.getFromDef('LU_drag_arrow')
LU_drag_FAM                     = FAM(LU_drag_arrow)

# LU_lift_arrowN             = flapper.getFromDef('LU_lift_arrowN')
# LU_liftN_FAM                     = FAM(LU_lift_arrowN)

# LD_drag_arrow             = flapper.getFromDef('LD_drag_arrow')
# LD_drag_FAM                     = FAM(LD_drag_arrow)

LU_lift_arrow             = flapper.getFromDef('LU_lift_arrow')
LU_lift_FAM                     = FAM(LU_lift_arrow)

LU_r_arrow             = flapper.getFromDef('LU_r_arrow')
LU_r_FAM                     = FAM(LU_r_arrow)

LU_a_arrow             = flapper.getFromDef('LU_a_arrow')
LU_a_FAM                     = FAM(LU_a_arrow)

rudder_drag_arrow             = flapper.getFromDef('rudder_drag_arrow')
rudder_drag_FAM                     = FAM(rudder_drag_arrow)
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

print('Velocity Vector Test Complete!\n')

vel_FreeFlow         = np.array([0,0,0])
# print('LU_wing Orientation:',LU_wing.getOrientation())
# print('LD_wing Orientation:',LD_wing.getOrientation())
# print('RU_wing Orientation:',RU_wing.getOrientation())
# print('RD_wing Orientation:',RD_wing.getOrientation())
# Rudder_Comparing_set = Rudder_translation.getSFVec3f()
# Rudder_Comparing_np = np.array(Rudder_Comparing_set)
# TranslationBack2o = list(Rudder_Comparing_np + [0,0,0])
# print('TranslationBack2o:',TranslationBack2o
StrokeAmp    = np.pi / 8
StrokeFreq   = 17
StrokeAmpOffSet = StrokeAmp

# Real_motor_LU_RD_joint.maxVelocity = 100
# Real_motor_LD_RU_joint.maxVelocity = 100
CountKit = 0

Real_motor_LU_wing_joint_sensor_value = Real_motor_LU_wing_joint_sensor.getValue()
Real_motor_LD_wing_joint_sensor_value = Real_motor_LD_wing_joint_sensor.getValue()
Real_motor_RU_wing_joint_sensor_value = Real_motor_RU_wing_joint_sensor.getValue()
Real_motor_RD_wing_joint_sensor_value = Real_motor_RD_wing_joint_sensor.getValue()
Real_motor_rudder_joint_sensor_value  = Real_motor_rudder_joint_sensor.getValue()

while flapper.step(timestep) != -1:
    # print('timestep',timestep)
    
    Now_time = flapper.getTime()
    LU_RD_joint_angle = StrokeAmp * np.sin(Now_time * StrokeFreq * 2 * np.pi) + StrokeAmpOffSet
    LD_RU_joint_angle = -LU_RD_joint_angle
    Real_motor_LU_RD_joint.setPosition(LU_RD_joint_angle)
    Real_motor_LD_RU_joint.setPosition(LD_RU_joint_angle)
    # Real_motor_LU_RD_joint.setVelocity(10)
    # Real_motor_LD_RU_joint.setVelocity(10)
    # print('LU_RD_joint_angle:',LU_RD_joint_angle)

    LU_OrVec = LU_wing.getOrientation()
    LD_OrVec = LD_wing.getOrientation()
    RU_OrVec = RU_wing.getOrientation()
    RD_OrVec = RD_wing.getOrientation()
    
    rudder_OrVec = rudder.getOrientation()
    
    
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

    LU_OrVec    = LU_wing.getOrientation()
    LD_OrVec    = LD_wing.getOrientation()
    RU_OrVec    = RU_wing.getOrientation()
    RD_OrVec    = RD_wing.getOrientation()
    rudder_OrVec = rudder.getOrientation()
    
    Flapper_OrVec = TheFlapper.getOrientation()
    Flapper_OrVec_in_Euler =  R2EAR(Flapper_OrVec)
    print('Flapper_OrVec:',Flapper_OrVec_in_Euler )
    
    # LU_RD_OrVec = LU_RD_joint.getOrientation()

    LU_bac.RequestWingOrientation2InertiaFrame (LU_OrVec)
    LD_bac.RequestWingOrientation2InertiaFrame (LD_OrVec)
    RU_bac.RequestWingOrientation2InertiaFrame (RU_OrVec)
    RD_bac.RequestWingOrientation2InertiaFrame (RD_OrVec)
    rudder_bac.RequestWingOrientation2InertiaFrame (rudder_OrVec)
    
    LU_bac.RequestWingPlaneDirection()
    LD_bac.RequestWingPlaneDirection()
    RU_bac.RequestWingPlaneDirection()
    RD_bac.RequestWingPlaneDirection()
    rudder_bac.RequestWingPlaneDirection()
    
    LU_velVec = LU_wing.getVelocity()
    LD_velVec = LD_wing.getVelocity()
    RU_velVec = RU_wing.getVelocity()
    RD_velVec = RD_wing.getVelocity()
    rudder_velVec = rudder.getVelocity()
    # LU_RD_velVec = LU_RD_joint.getVelocity()
    
    
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
    
    d_Real_motor_LU_wing_joint_sensor_value = (Real_motor_LU_wing_joint_sensor_value - Last_Real_motor_LU_wing_joint_sensor_value) /0.001
    d_Real_motor_LD_wing_joint_sensor_value = (Real_motor_LD_wing_joint_sensor_value - Last_Real_motor_LD_wing_joint_sensor_value) /0.001
    d_Real_motor_RU_wing_joint_sensor_value = (Real_motor_RU_wing_joint_sensor_value - Last_Real_motor_RU_wing_joint_sensor_value) /0.001
    d_Real_motor_RD_wing_joint_sensor_value = (Real_motor_RD_wing_joint_sensor_value - Last_Real_motor_RD_wing_joint_sensor_value) /0.001
    d_Real_motor_rudder_joint_sensor_value  = (Real_motor_rudder_joint_sensor_value - Last_Real_motor_rudder_joint_sensor_value) /0.001
    
    LU_bac.RequestVelocities(vel_FreeFlow,LU_velVec[0:3],\
                                          LU_velVec[3:6],d_Real_motor_LU_wing_joint_sensor_value)
    LD_bac.RequestVelocities(vel_FreeFlow,LD_velVec[0:3],\
                                          LD_velVec[3:6],d_Real_motor_LD_wing_joint_sensor_value)
    RU_bac.RequestVelocities(vel_FreeFlow,RU_velVec[0:3],\
                                          RU_velVec[3:6],d_Real_motor_RU_wing_joint_sensor_value)
    RD_bac.RequestVelocities(vel_FreeFlow,RD_velVec[0:3],\
                                          RD_velVec[3:6],d_Real_motor_RD_wing_joint_sensor_value)
    rudder_bac.RequestVelocities(vel_FreeFlow,rudder_velVec[0:3],\
                                          rudder_velVec[3:6],d_Real_motor_rudder_joint_sensor_value)
                                          
    TheFlapper_velVec = TheFlapper.getVelocity()
    
    
    # print('TheFlapper_velVec[0:3]:',TheFlapper_velVec[0:3])
    
                                        
    LU_bac.CalcEffectiveVelocity()
    LD_bac.CalcEffectiveVelocity()
    RU_bac.CalcEffectiveVelocity()
    RD_bac.CalcEffectiveVelocity()
    rudder_bac.CalcEffectiveVelocity()
    
    LU_bac.CalcAoA()
    LD_bac.CalcAoA()
    RU_bac.CalcAoA()
    RD_bac.CalcAoA()
    rudder_bac.CalcAoA()
    
    LU_bac.CopmputeAerodynamicForce()
    LD_bac.CopmputeAerodynamicForce()
    RU_bac.CopmputeAerodynamicForce()
    RD_bac.CopmputeAerodynamicForce()
    rudder_bac.CopmputeAerodynamicForce()
    

    # print('LU_r_shift:',LU_r_shift)
    # LU_r_shift = np.matmul(LU_wing_Rotation_matrix, np.array([LU_bac. X_pos_r, -LU_bac. Y_pos_r, 0]))
    LU_r_shift_in_wing = np.array([LU_bac. X_pos_r, -LU_bac. Y_pos_r, 0])
    LU_r_shift = np.array(np.matmul(LU_wing_Rotation_matrix, LU_r_shift_in_wing)).squeeze().tolist()
    LU_r = np.array(np.sum(LU_bac. F_r, axis=0)).squeeze()
    LU_r_Axis = P2AA(LU_r).tolist()
    LU_r_position = (np.array(LU_wing.getPosition()) + np.array(LU_r_shift)).tolist()
    LU_r_norm     = np.linalg.norm(LU_r)
    LU_r_FAM.update_force_device(LU_r_Axis,LU_r_position,LU_r_norm)
    
    LU_a_shift_in_wing = np.array([LU_bac. X_pos_a, -LU_bac. Y_pos_a, 0])
    LU_a_shift = np.array(np.matmul(LU_wing_Rotation_matrix, LU_a_shift_in_wing)).squeeze().tolist()
    LU_a = np.array(np.sum(LU_bac. F_a, axis=0)).squeeze()
    LU_a_Axis = P2AA(LU_a).tolist()
    LU_a_position = (np.array(LU_wing.getPosition()) + np.array(LU_a_shift)).tolist()
    LU_a_norm     = np.linalg.norm(LU_a)
    LU_a_FAM.update_force_device(LU_a_Axis,LU_a_position,LU_a_norm)

    LU_t_shift_in_wing = np.array([LU_bac. X_pos_t, -LU_bac. Y_pos_t, 0])
    LU_t_shift = np.array(np.matmul(LU_wing_Rotation_matrix, LU_t_shift_in_wing)).squeeze().tolist()
    LU_drag = np.array(np.sum(LU_bac. F_t_drag, axis=0)).squeeze()
    LU_drag_Axis = P2AA(LU_drag).tolist()
    LU_drag_position = (np.array(LU_wing.getPosition()) + np.array(LU_t_shift)).tolist()
    LU_drag_norm     = np.linalg.norm(LU_drag)
    LU_drag_FAM.update_force_device(LU_drag_Axis,LU_drag_position,LU_drag_norm)
    print('LU_t_shift_in_wing',LU_t_shift_in_wing)



    LU_lift = np.array(np.sum(LU_bac. F_t_lift, axis=0)).squeeze()
    LU_lift_Axis = P2AA(LU_lift).tolist()
    LU_lift_position = LU_drag_position
    LU_lift_norm     = np.linalg.norm(LU_lift)
    LU_lift_FAM.update_force_device(LU_lift_Axis,LU_lift_position,LU_lift_norm)
  
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
    
    
    
    LD_r = np.array(np.sum(LD_bac. F_r, axis=0)).squeeze()
    RU_r = np.array(np.sum(RU_bac. F_r, axis=0)).squeeze()
    RD_r = np.array(np.sum(RD_bac. F_r, axis=0)).squeeze()
    
    # LD_r_Axis = P2AA(LD_r).tolist()
    
    LD_a = np.array(np.sum(LD_bac. F_a, axis=0)).squeeze()
    RU_a = np.array(np.sum(RU_bac. F_a, axis=0)).squeeze()
    RD_a = np.array(np.sum(RD_bac. F_a, axis=0)).squeeze()
    
    
    rudder_lift = np.array(np.sum(rudder_bac. F_t_lift, axis=0)).squeeze()
    rudder_drag = np.array(np.sum(rudder_bac. F_t_drag, axis=0)).squeeze()
    rudder_r = np.array(np.sum(rudder_bac. F_r, axis=0)).squeeze()
    rudder_a = np.array(np.sum(rudder_bac. F_a, axis=0)).squeeze()
    
    
    rudder_t_shift_in_local = np.array([rudder_bac. X_pos_t, -rudder_bac. Y_pos_t, 0])
    rudder_t_shift = np.array(np.matmul(rudder_Rotation_matrix, rudder_t_shift_in_local)).squeeze().tolist()
    # LU_drag = np.array(np.sum(rudder_bac. F_t_drag, axis=0)).squeeze()
    rudder_drag_Axis = P2AA(rudder_drag).tolist()
    rudder_drag_position = (np.array(rudder.getPosition()) + np.array(rudder_t_shift)).tolist()
    rudder_drag_norm     = np.linalg.norm(rudder_drag)
    rudder_drag_FAM.update_force_device(rudder_drag_Axis,rudder_drag_position,rudder_drag_norm)
    print('rudder_t_shift_in_local',rudder_t_shift_in_local)

    # LD_a_Axis = P2AA(LD_a).tolist()
    # print ('LD_drag:',LD_drag,'Axis:',LD_Axis)
    # LD_drag_arrow_rotation.setSFRotation(LD_Axis)
    # LD_drag_arrow_translation.setSFVec3f(LD_wing.getPosition())
    # print('Drag:',[LU_drag[0],LU_drag[1],LU_drag[2]])
    LU_wing.addForceWithOffset([LU_drag[0],LU_drag[1],LU_drag[2]],[LU_bac.X_pos_t,-LU_bac.Y_pos_t,0],False)
    LD_wing.addForceWithOffset([LD_drag[0],LD_drag[1],LD_drag[2]],[LD_bac.X_pos_t,-LD_bac.Y_pos_t,0],False)
    RU_wing.addForceWithOffset([RU_drag[0],RU_drag[1],RU_drag[2]],[RU_bac.X_pos_t,-RU_bac.Y_pos_t,0],False)
    RD_wing.addForceWithOffset([RD_drag[0],RD_drag[1],RD_drag[2]],[RD_bac.X_pos_t,-RD_bac.Y_pos_t,0],False)
    rudder.addForceWithOffset([rudder_drag[0],rudder_drag[1],rudder_drag[2]],rudder_t_shift_in_local.tolist(),False)
    # print('DRAG', [LU_drag[0],LU_drag[1],LU_drag[2]])
    
    LU_wing.addForceWithOffset([LU_lift[0],LU_lift[1],LU_lift[2]],[LU_bac.X_pos_t,-LU_bac.Y_pos_t,0],False)
    LD_wing.addForceWithOffset([LD_lift[0],LD_lift[1],LD_lift[2]],[LD_bac.X_pos_t,-LD_bac.Y_pos_t,0],False)
    RU_wing.addForceWithOffset([RU_lift[0],RU_lift[1],RU_lift[2]],[RU_bac.X_pos_t,-RU_bac.Y_pos_t,0],False)
    RD_wing.addForceWithOffset([RD_lift[0],RD_lift[1],RD_lift[2]],[RD_bac.X_pos_t,-RD_bac.Y_pos_t,0],False)
    rudder.addForceWithOffset([rudder_lift[0],rudder_lift[1],rudder_lift[2]],[rudder_bac.X_pos_t,-rudder_bac.Y_pos_t,0],False)
    
    # print('LIFT', [LU_lift[0],LU_lift[1],LU_lift[2]])
    LU_wing.addForceWithOffset([LU_r[0],LU_r[1],LU_r[2]],[LU_bac.X_pos_r,-LU_bac.Y_pos_r,0],False)
    LD_wing.addForceWithOffset([LD_r[0],LD_r[1],LD_r[2]],[LD_bac.X_pos_r,-LD_bac.Y_pos_r,0],False)
    RU_wing.addForceWithOffset([RU_r[0],RU_r[1],RU_r[2]],[RU_bac.X_pos_r,-RU_bac.Y_pos_r,0],False)
    RD_wing.addForceWithOffset([RD_r[0],RD_r[1],RD_r[2]],[RD_bac.X_pos_r,-RD_bac.Y_pos_r,0],False)
    rudder.addForceWithOffset([rudder_r[0],rudder_r[1],rudder_r[2]],[rudder_bac.X_pos_r,-rudder_bac.Y_pos_r,0],False)


    LU_wing.addForceWithOffset([LU_a[0],LU_a[1],LU_a[2]],[LU_bac.X_pos_a,-LU_bac.Y_pos_a,0],False)
    LD_wing.addForceWithOffset([LD_a[0],LD_a[1],LD_a[2]],[LD_bac.X_pos_a,-LD_bac.Y_pos_a,0],False)
    RU_wing.addForceWithOffset([RU_a[0],RU_a[1],RU_a[2]],[RU_bac.X_pos_a,-RU_bac.Y_pos_a,0],False)
    RD_wing.addForceWithOffset([RD_a[0],RD_a[1],RD_a[2]],[RD_bac.X_pos_a,-RD_bac.Y_pos_a,0],False)
    rudder.addForceWithOffset([rudder_a[0],rudder_a[1],rudder_a[2]],[rudder_bac.X_pos_a,-rudder_bac.Y_pos_a,0],False)

   
    # print('d_Real_motor_LU_wing_joint_sensor_value',d_Real_motor_LU_wing_joint_sensor_value)
    torsion_spring_constant = 0.025
    torsion_spring_yaw_offset = -0.1 ##0-0.3
    K_pitch =0.8
    torsion_spring_pitch_offset =  K_pitch * Flapper_OrVec_in_Euler[1]### 仔细调整
    # print('TEST: torsion_spring_pitch_offset,', torsion_spring_pitch_offset)
    
    Real_motor_LU_wing_joint.setTorque(- torsion_spring_constant \
                                       * (Real_motor_LU_wing_joint_sensor_value + torsion_spring_yaw_offset + torsion_spring_pitch_offset))
    Real_motor_LD_wing_joint.setTorque(- torsion_spring_constant \
                                       * (Real_motor_LD_wing_joint_sensor_value - torsion_spring_yaw_offset - torsion_spring_pitch_offset))
    Real_motor_RU_wing_joint.setTorque(- torsion_spring_constant \
                                       * (Real_motor_RU_wing_joint_sensor_value - torsion_spring_yaw_offset + torsion_spring_pitch_offset))
    Real_motor_RD_wing_joint.setTorque(- torsion_spring_constant \
                                       * (Real_motor_RD_wing_joint_sensor_value + torsion_spring_yaw_offset - torsion_spring_pitch_offset))
    
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
