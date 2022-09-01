"""Simple Arrow Manager,"""
#2022-8-1 09:35:03
##### Possible Wrong 2022-7-29 08:00:21：About FromPointer2Axis_Angle

import numpy as np
from controller import Robot,Motor
from controller import Supervisor

class ForceArrowManager():
    ARROWHEIGHT_RE = 0.2
    def __init__(self, force_device):
        self.force_arrow_translation = force_device.getField('translation')             ### Force translation!!
        self.force_arrow_rotation = force_device.getField('rotation')                   ### Force rotation!!
        force_arrow_children = force_device.getField('children')
        force_arrow_children_cone_Transform = force_arrow_children.getMFNode(0)
        self.force_arrow_children_cone_Transform_translation = \
            force_arrow_children_cone_Transform.getField('translation')            #### Force cone translation!!
        force_arrow_children_cylinder_Transform = force_arrow_children.getMFNode(1)
        self.force_arrow_children_cylinder_Transform_translation = \
            force_arrow_children_cylinder_Transform.getField('translation')        #### Force cylinder translation!!
        self.force_arrow_children_cylinder_Transform_scale = \
            force_arrow_children_cylinder_Transform.getField('scale')   #### Force cylinder height!!
        print( '@STATE:',force_device.getTypeName,'configuration succeed!!!')

    def update_force_device(self, axis, force_position, force_norm):
        self.force_arrow_rotation.setSFRotation(axis)
        self.force_arrow_translation.setSFVec3f(force_position)
        self.force_arrow_children_cone_Transform_translation.setSFVec3f([0,force_norm/self.ARROWHEIGHT_RE * 0.1,0])
        self.force_arrow_children_cylinder_Transform_translation.setSFVec3f([0,force_norm/self.ARROWHEIGHT_RE/2 * 0.1,0])
        self.force_arrow_children_cylinder_Transform_scale.setSFVec3f([1,force_norm/self.ARROWHEIGHT_RE,1])