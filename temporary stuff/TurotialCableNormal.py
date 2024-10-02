from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
# -*- coding: utf-8 -*-
from softrobots.actuators import PullingCable
from splib3.loaders import loadPointListFromFile
import Sofa.Core
import Sofa.constants.Key as Key

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

def Finger(parentNode=None, name="Finger"):
    finger = parentNode.addChild(name) 
    finger.addObject('RequiredPlugin', pluginName=['Sofa.GL.Component.Rendering3D', 'Sofa.Component.IO.Mesh'])
    volumeMeshFileName='finger.vtk'
    rotation=[0.0, 0.0, 0.0]
    translation=[0.0, 0.0, 0.0]
    scale=[1.0, 1.0, 1.0]
    surfaceMeshFileName='finger.stl'
    collisionMesh='finger.stl'
    withConstrain=True
    surfaceColor=[1., 1., 1., 1.]
    poissonRatio=0.3
    youngModulus=18000
    totalMass=1.0
    solverName=''
    color=[1., 1., 1., 1.]
    finger.integration = finger.addObject('EulerImplicitSolver', name='integration')
    finger.solver = finger.addObject('SparseLDLSolver', name="solver", template='CompressedRowSparseMatrixd')
    finger.loader = finger.addObject('MeshVTKLoader', name='loader', filename=volumeMeshFileName, rotation=rotation, translation=translation,scale3d=scale)
    finger.container = finger.addObject('TetrahedronSetTopologyContainer', position=finger.loader.position.getLinkPath(),
    tetras=finger.loader.tetras.getLinkPath(), name='container')
    finger.dofs = finger.addObject('MechanicalObject', template='Vec3', name='dofs')
    finger.mass = finger.addObject('UniformMass', totalMass=totalMass, name='mass')
    finger.forcefield = finger.addObject('TetrahedronFEMForceField', template='Vec3',
                                         method='large', name='forcefield',
                                         poissonRatio=poissonRatio, youngModulus=youngModulus)
    finger.correction = finger.addObject('LinearSolverConstraintCorrection', name='correction')


    collisionmodel = finger.addChild('CollisionModel')
    collisionmodel.addObject('MeshSTLLoader', name='loader', filename=collisionMesh, rotation=rotation,
                                    translation=translation, scale3d=scale)
    collisionmodel.addObject('TriangleSetTopologyContainer', src='@loader', name='container')
    collisionmodel.addObject('MechanicalObject', template='Vec3', name='dofs')
    collisionmodel.addObject('TriangleCollisionModel')
    collisionmodel.addObject('LineCollisionModel')
    collisionmodel.addObject('PointCollisionModel')
    collisionmodel.addObject('BarycentricMapping')

    fingerVisu = finger.addChild("VisualModel")
    fingerVisu.loader = fingerVisu.addObject('MeshSTLLoader', name="loader", filename=surfaceMeshFileName)
    fingerVisu.addObject('OglModel', name="model", src="@loader",rotation=rotation, translation=translation, scale3d=[1.0, 1.0, 1.0], color=surfaceColor, updateNormals=True)
    fingerVisu.addObject('BarycentricMapping', name='mapping')

    fixingBox=[-30, -10, -30, 30, 10, 30]

    FixedBox(finger,
             doVisualization=True,
             atPositions=fixingBox)
    cable =finger.addChild("cable")
    cable = PullingCable(cable, cableGeometry=loadPointListFromFile("cable.json"))
    # cable2 = PullingCable(finger, cableGeometry=loadPointListFromFile("cable2.json"))
    # cable3 = PullingCable(finger, cableGeometry=loadPointListFromFile("cable3.json"))
    # cable4 = PullingCable(finger, cableGeometry=loadPointListFromFile("cable4.json"))
    #cable2 = PullingCable(eobject, cableGeometry=loadPointListFromFile("data/mesh/cable2.json"))
    finger.addObject(FingerController(cable))
    return None


def createScene(rootNode):
    # -*- coding: utf-8 -*-
    from stlib3.scene import MainHeader
    m = MainHeader(rootNode, plugins=["SoftRobots"])
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"

    m.getObject("VisualStyle").displayFlags = 'showForceFields showBehaviorModels showInteractionForceFields'
    m.addObject("FreeMotionAnimationLoop")
    m.addObject("GenericConstraintSolver", maxIterations=250, tolerance=1e-20)

    Finger(rootNode)

    return rootNode
