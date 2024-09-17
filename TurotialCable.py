from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
# -*- coding: utf-8 -*-
from softrobots.actuators import PullingCable
from splib3.loaders import loadPointListFromFile
import Sofa.Core
import Sofa.constants.Key as Key



class AnkleController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0][0]
        self.cable2 = args[0][1]
        self.cable3 = args[0][2]
        self.cable4 = args[0][3]
        self.name = "AnkleController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        displacement2 = self.cable2.CableConstraint.value[0]
        displacement3 = self.cable3.CableConstraint.value[0]
        displacement4 = self.cable4.CableConstraint.value[0]
        if e["key"] == Key.rightarrow:
            displacement += 1.
            displacement3 -= 1.

        elif e["key"] == Key.leftarrow:
            displacement3 += 1.
            displacement -= 1.

        if e["key"] == Key.uparrow:
            displacement2 += 1.
            displacement4 -= 1.

        elif e["key"] == Key.downarrow:
            displacement4 += 1.
            displacement2 -= 1.
        print("C1",displacement)
        self.cable.CableConstraint.value = [displacement]
        self.cable2.CableConstraint.value = [displacement2]
        self.cable3.CableConstraint.value = [displacement3]
        self.cable4.CableConstraint.value = [displacement4]
        # if displacement >= 0:   
        #     self.cable.CableConstraint.value = [displacement]

def Ankle(parentNode=None, name="Ankle",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[-30, -10, -30, 30, 10, 30]):
    Ankle = parentNode.addChild(name)
    from stlib3.physics.deformable import ElasticMaterialObject
    from stlib3.physics.constraints import FixedBox
    femAnkle = ElasticMaterialObject(Ankle,
                                    volumeMeshFileName="ankle.vtk",
                                    poissonRatio=0.3,
                                    youngModulus=18000,
                                    totalMass=1.0,
                                    surfaceColor=[0.0, 0.8, 0.7, 1.0],
                                    surfaceMeshFileName="ankle.stl",
                                    rotation=rotation,
                                    translation=translation,
                                    scale=[1.0, 1.0, 1.0])
    
    Ankle.addChild(femAnkle)
    FixedBox(femAnkle,
             doVisualization=True,
             atPositions=fixingBox)
    cable =femAnkle.addChild("cable")
    cables=[]
    cables.append(PullingCable(cable, name="cable1",cableGeometry=loadPointListFromFile("cable1.json")))
    cables.append(PullingCable(cable, name="cable2", cableGeometry=loadPointListFromFile("cable2.json")))
    cables.append(PullingCable(cable, name="cable3", cableGeometry=loadPointListFromFile("cable3.json")))
    cables.append(PullingCable(cable, name="cable4", cableGeometry=loadPointListFromFile("cable4.json")))

   


    goal = Ankle.addChild('goal')
    goal.addObject('MechanicalObject', name='goalMO',
                   position=[26, 100, 26])

    effector = femAnkle.addChild('fingertip')
    effector.addObject('MechanicalObject', position=([0, 100, 0]),name="dd")
    effector.addObject('PositionEffector', template='Vec3', name="Efector_final",
                       indices=0,
                       effectorGoal=goal.goalMO.position.value)
    effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    Ankle.addObject(AnkleController(cables))
    return femAnkle



def createScene(rootNode):
    # -*- coding: utf-8 -*-
    from stlib3.scene import MainHeader
    m = MainHeader(rootNode, plugins=["SoftRobots"])
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"

    m.getObject("VisualStyle").displayFlags = 'showForceFields showBehaviorModels showInteractionForceFields'
    m.addObject("FreeMotionAnimationLoop")
    m.addObject("GenericConstraintSolver", maxIterations=250, tolerance=1e-20)

    Ankle(rootNode)

    return rootNode
