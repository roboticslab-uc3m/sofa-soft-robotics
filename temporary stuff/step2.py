from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
# -*- coding: utf-8 -*-
from softrobots.actuators import PullingCable
from splib3.loaders import loadPointListFromFile
import Sofa.Core
import Sofa.constants.Key as Key
from cosserat.usefulFunctions import buildEdges, pluginList, BuildCosseratGeometry
from cosserat.cosseratObject import Cosserat

class FingerController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.plus:
            displacement += 1.

        elif e["key"] == Key.minus:
            displacement -= 1.
        print("C1",displacement)
        if displacement >= 0:   
            self.cable.CableConstraint.value = [displacement]

def Finger(parentNode=None, name="Finger",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[-30, -10, -30, 30, 10, 30]):
    finger = parentNode.addChild(name)
    eobject = ElasticMaterialObject(finger,
                                    volumeMeshFileName="ankle.vtk",
                                    poissonRatio=0.3,
                                    youngModulus=18000,
                                    totalMass=1.0,
                                    surfaceColor=[0.0, 0.8, 0.7, 1.0],
                                    surfaceMeshFileName="ankle.stl",
                                    collisionMesh='ankle.stl',
                                    rotation=rotation,
                                    translation=translation,
                                    scale=[1.0, 1.0, 1.0])
    
    finger.addChild(eobject)
    FixedBox(eobject,
             doVisualization=True,
             atPositions=fixingBox)
    cable = PullingCable(eobject, cableGeometry=loadPointListFromFile("cable1.json"))
    #cable2 = PullingCable(eobject, cableGeometry=loadPointListFromFile("data/mesh/cable2.json"))
    finger.addObject(FingerController(cable))
    #finger.addObject(FingerController2(cable2))


def createScene(rootNode):
    # -*- coding: utf-8 -*-
    from stlib3.scene import MainHeader
    m=MainHeader(rootNode, plugins=["SoftRobots"])
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"
    m.getObject("VisualStyle").displayFlags = 'showForceFields showBehaviorModels showInteractionForceFields'
    m.addObject("FreeMotionAnimationLoop")
    m.addObject("GenericConstraintSolver", maxIterations=250, tolerance=1e-20)

    Finger(rootNode)
    # beamGeometrie = {'init_pos': [0., 0., 0.], 'tot_length': 81, 'nbSectionS': 12,
    #                  'nbFramesF': 30, 'buildCollisionModel': 0, 'beamMass': 0.}
    # cosserat = rootNode.addChild(Cosserat(parent=rootNode, cosseratGeometry=beamGeometrie, radius=0.5,
    #                                        useCollisionModel=True, name="cosserat", youngModulus=5e6, poissonRatio=0.4))

    return rootNode
