# -*- coding: utf-8 -*-
"""
Created on Tue Jun 23 11:35:46 2015

@author: adelpret
"""

import iDynTree
from iDynTree import KinDynComputations

URDF_FILE = '/home/letrend/workspace/roboy_control/src/CARDSflow/robots/msj_platform/model.urdf';

kinDyn = KinDynComputations();
kinDyn.loadRobotModelFromFile(URDF_FILE);
print "The loaded model has", kinDyn.getNrOfDegreesOfFreedom(), \
    "internal degrees of freedom and",kinDyn.getNrOfLinks(),"links."

base = iDynTree.Transform()
vel = iDynTree.Twist()
dofs = kinDyn.getNrOfDegreesOfFreedom();
q = iDynTree.VectorDynSize(dofs);
dq = iDynTree.VectorDynSize(dofs);
for dof in range(dofs):
    # For the sake of the example, we fill the joints vector with gibberish data (remember in any case
    # that all quantities are expressed in radians-based units 
    q.setVal(dof, 1.0);
    dq.setVal(dof, 0.4);


# The spatial acceleration is a 6d acceleration vector. 
# For all 6d quantities, we use the linear-angular serialization
# (the first three value are for the linear quantity, the 
#  the last  three values are for the angular quantity)
gravity = iDynTree.Vector3();
gravity.zero();
gravity.setVal(2, -9.81);
kinDyn.setRobotState(base,q,vel,dq,gravity);


