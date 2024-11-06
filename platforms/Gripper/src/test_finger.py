#from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
# -*- coding: utf-8 -*-
from softrobots.actuators import PullingCable
from splib3.loaders import loadPointListFromFile
import Sofa.Core
import Sofa.constants.Key as Key
import numpy as np
import socket
import json
import time
from Finger_Model import Finger

#Define the target position
target = [-40.0, 40.0, 40.0]

# Creation of the scene in SOFA when this file is launched
def createScene(rootNode):
    from stlib3.scene import MainHeader, ContactHeader
    n =ContactHeader(rootNode, alarmDistance=3, contactDistance=0.5, frictionCoef=1)
    m = MainHeader(rootNode, plugins=[
        "SofaPython3", 
        "SoftRobots", 

        "SoftRobots.Inverse", 
        # "SofaCUDA",
        "Sofa.Component.AnimationLoop", 
        "Sofa.Component.Collision.Detection.Algorithm",
        "Sofa.Component.Collision.Detection.Intersection",
        "Sofa.Component.Collision.Geometry",
        "Sofa.Component.Collision.Response",
        "Sofa.Component.Constraint.Lagrangian.Correction",
        "Sofa.Component.Constraint.Lagrangian.Solver", 
        "Sofa.Component.Engine.Select", 
        "Sofa.Component.LinearSolver.Direct", 
        "Sofa.Component.Mapping.Linear", 
        "Sofa.Component.Mass", 
        "Sofa.Component.ODESolver.Backward", 
        "Sofa.Component.SolidMechanics.FEM.Elastic", 
        "Sofa.Component.SolidMechanics.Spring", 
        "Sofa.Component.StateContainer", 
        "Sofa.Component.Topology.Container.Dynamic", 
        "Sofa.Component.Collision.Response.Contact",
        "Sofa.Component.Topology.Container.Constant",
        "Sofa.Component.Visual",
        "Sofa.Component.LinearSolver.Iterative"])
    
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"
    m.getObject("VisualStyle").displayFlags = 'showCollisionModels showForceFields showBehaviorModels showInteractionForceFields'
    rootNode.addObject('FreeMotionAnimationLoop')
    
    # Add a QPInverseProblemSolver to the scene if you need to solve inverse problem like the one that involved
    # when manipulating the robots by specifying their effector's position instead of by direct control
    #  of the actuator's parameters.
    rootNode.addObject('QPInverseProblemSolver', printLog=True)
    # Otherwise use a GenericConstraintSolver
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-5, maxIterations=100)
    


 
    Finger(rootNode)
    goal = m.addChild("Goal")
    #im using template Rigid3d for CUDA
    goal_position = " ".join(map(str, target)) + " 0 0 0 1"
    goal.addObject("MechanicalObject", template="Rigid3d", name="goal", position=goal_position, showObject="1", showObjectScale="10")

    return rootNode

