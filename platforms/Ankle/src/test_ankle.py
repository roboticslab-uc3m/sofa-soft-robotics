from stlib3.physics.deformable import ElasticMaterialObject
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
from Ankle_Model import Ankle


# Creation of the scene in SOFA when this file is launched
def createScene(rootNode):
    from stlib3.scene import MainHeader
    m = MainHeader(rootNode, plugins=[
        "SoftRobots", 
        "SofaPython3", 
        "SoftRobots.Inverse", 
        "Sofa.Component.AnimationLoop", 
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
        "Sofa.Component.Visual"])
    
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"
    m.getObject("VisualStyle").displayFlags = 'showForceFields showBehaviorModels showInteractionForceFields'
    m.addObject("FreeMotionAnimationLoop")
    m.addObject("GenericConstraintSolver", maxIterations=250, tolerance=1e-20)
    m.addObject('QPInverseProblemSolver', printLog=False)
    
    Ankle(rootNode)

    return rootNode

