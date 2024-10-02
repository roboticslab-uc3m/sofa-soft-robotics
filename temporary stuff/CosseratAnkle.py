from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
# -*- coding: utf-8 -*-
from softrobots.actuators import PullingCable
from splib3.loaders import loadPointListFromFile
import Sofa.Core
import Sofa.constants.Key as Key
from cosserat.usefulFunctions import buildEdges, pluginList, BuildCosseratGeometry
from cosserat.cosseratObject import Cosserat
from splib3.numerics import Quat

class Animation(Sofa.Core.Controller):
    """
        Implements the AnimationManager as a PythonScriptController
    """

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.rigidBaseMO = args[0]
        self.rateAngularDeformMO = args[1]

        self.rate = 0.2
        self.angularRate = 0.02
        return

    def _extracted_from_onKeypressedEvent_10(self, qOld, posA, angularRate):
        qNew = Quat.createFromEuler([0., angularRate, 0.], 'ryxz')
        qNew.normalize()
        qNew.rotateFromQuat(qOld)
        for i in range(4):
            posA[0][i+3] = qNew[i]

    def onKeypressedEvent(self, event):
        key = event['key']
        if ord(key) == 19:  # up
            with self.rigidBaseMO.rest_position.writeable() as posA:
                qOld = Quat()
                for i in range(4):
                    qOld[i] = posA[0][i+3]

                self._extracted_from_onKeypressedEvent_10(
                     qOld, posA, self.angularRate)

        if ord(key) == 21:  # down
            with self.rigidBaseMO.rest_position.writeable() as posA:
                qOld = Quat()
                for i in range(4):
                    qOld[i] = posA[0][i+3]

                self._extracted_from_onKeypressedEvent_10(
                    qOld, posA, -self.angularRate)

        if ord(key) == 18:  # left
            with self.rigidBaseMO.rest_position.writeable() as posA:
                posA[0][0] -= self.rate
        if ord(key) == 20:  # right
            with self.rigidBaseMO.rest_position.writeable() as posA:
                posA[0][0] += self.rate

def Finger(parentNode=None, name="Finger",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[-18, -15, -8, 2, -3, 8]):
    Ankle = parentNode.addChild(name)
    femAnkle = ElasticMaterialObject(Ankle,
                                    volumeMeshFileName="finger.vtk",
                                    poissonRatio=0.3,
                                    youngModulus=18000,
                                    totalMass=1.0,
                                    surfaceColor=[0.0, 0.8, 0.7, 1.0],
                                    surfaceMeshFileName="finger.stl",
                                    collisionMesh='finger.stl',
                                    rotation=rotation,
                                    translation=translation,
                                    scale=[1.0, 1.0, 1.0])
    
    Ankle.addChild(femAnkle)
    FixedBox(femAnkle,
             doVisualization=True,
             atPositions=fixingBox)
    return femAnkle
    # cable = PullingCable(eobject, cableGeometry=loadPointListFromFile("cable1.json"))
    #cable2 = PullingCable(eobject, cableGeometry=loadPointListFromFile("data/mesh/cable2.json"))
    # finger.addObject(FingerController(cable))
    #finger.addObject(FingerController2(cable2))



def ankle(parentNode=None, name="Ankle",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[-30, -10, -30, 30, 10, 30]):
    Ankle = parentNode.addChild(name)
    femAnkle = ElasticMaterialObject(Ankle,
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
    
    Ankle.addChild(femAnkle)
    FixedBox(femAnkle,
             doVisualization=True,
             atPositions=fixingBox)
    return femAnkle
    # cable = PullingCable(eobject, cableGeometry=loadPointListFromFile("cable1.json"))
    #cable2 = PullingCable(eobject, cableGeometry=loadPointListFromFile("data/mesh/cable2.json"))
    # finger.addObject(FingerController(cable))
    #finger.addObject(FingerController2(cable2))

def createScene(rootNode):
    # -*- coding: utf-8 -*-
    from stlib3.scene import MainHeader
    rootNode.addObject('RequiredPlugin', name='CSparseSolvers') # Needed to use components [SparseLUSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop') # Needed to use components [FreeMotionAnimationLoop]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry') # Needed to use components [LineCollisionModel,PointCollisionModel,TriangleCollisionModel]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction') # Needed to use components [GenericConstraintCorrection,LinearSolverConstraintCorrection]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver') # Needed to use components [GenericConstraintSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select') # Needed to use components [BoxROI]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct') # Needed to use components [SparseLDLSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear') # Needed to use components [BarycentricMapping]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [UniformMass]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward') # Needed to use components [EulerImplicitSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Setting') # Needed to use components [BackgroundSetting]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic') # Needed to use components [TetrahedronFEMForceField]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring') # Needed to use components [RestShapeSpringsForceField]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic') # Needed to use components [TetrahedronSetTopologyContainer,TriangleSetTopologyContainer]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual') # Needed to use components [VisualStyle]  
    rootNode.addObject('RequiredPlugin', name='Cosserat') # Needed to use components [BeamHookeLawForceField,DiscreteCosseratMapping]
    m=MainHeader(rootNode, plugins=["SoftRobots"])
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels "
    m.getObject("VisualStyle").displayFlags = 'showForceFields showBehaviorModels showInteractionForceFields showMechanicalMappings'
    m.addObject("GenericConstraintSolver", maxIterations=250, tolerance=1e-20)
    gravity = [0, 0, 0]
    #asignamos el valore  de la gravedad
    rootNode.gravity.value = gravity
    #configuración del fondo
    rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
    #permite la asignacion del frame principal
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    
    femAnkle =Finger(rootNode,translation=[-7.5,-2.5,7.5],rotation=[0.0, 180.0, 0.0],fixingBox=[-8 ,-5 ,-8 ,12, 7 ,8])
    ##########################################
    # Cable points                           #
    ##########################################
    # Mappe points inside the meca, this points will be use for the bilateral mapping
    FEMpos = [" 10 10 0 25 10 0 40 10 0 55 10 0 70 10 0 76 10 0 91 10 0"]

    femPoints = femAnkle.addChild('femPoints')
    inputFEMCable = femPoints.addObject('MechanicalObject', name="pointsInFEM", position=FEMpos, showObject="1",
                                        showIndices="1")
    femPoints.addObject('BarycentricMapping')

    femAnkle.addObject('LinearSolverConstraintCorrection')



    # ###############
    # New adds to use the sliding Actuator
    ###############
    cableNode = rootNode.addChild('cableNode')
    cableNode.addObject('EulerImplicitSolver', firstOrder="0",
                        rayleighStiffness="0.1", rayleighMass='0.1')
    cableNode.addObject('SparseLUSolver', name='solver', template='CompressedRowSparseMatrixd')
    cableNode.addObject('GenericConstraintCorrection')

    beamGeometrie = {'init_pos': [10., 10., 0.], 'tot_length': 81, 'nbSectionS': 12,
                     'nbFramesF': 30, 'buildCollisionModel': 0, 'beamMass': 0.}
    cosserat = cableNode.addChild(Cosserat(parent=cableNode, cosseratGeometry=beamGeometrie, radius=0.5,
                                           useCollisionModel=False, name="cosserat", youngModulus=5e6, poissonRatio=0.4, translation=[10., 10., 0.]))

    cableNode.addObject(Animation(cosserat.rigidBaseNode.RigidBaseMO,
                                  cosserat.cosseratCoordinateNode.cosseratCoordinateMO))
    mappedFrameNode = cosserat.cosseratFrame

    #  This create a new node in the scene. This node is appended to the finger's node.
    slidingPoint = mappedFrameNode.addChild('slidingPoint')

    # This create a MechanicalObject, a componant holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    # the points where the cable is passing by.
    slidingPointMO = slidingPoint.addObject('MechanicalObject', name="cablePos",
                                            position=cosserat.frames3D, showObject="1", showIndices="0")
    slidingPoint.addObject('IdentityMapping')

    mappedPointsNode = slidingPoint.addChild('MappedPoints')
    femPoints.addChild(mappedPointsNode)
    mappedPoints = mappedPointsNode.addObject('MechanicalObject', template='Vec3d', position=FEMpos,
                                              name="FramesMO", showObject='1', showObjectScale='1')

    inputCableMO = slidingPointMO.getLinkPath()
    inputFEMCableMO = inputFEMCable.getLinkPath()
    outputPointMO = mappedPoints.getLinkPath()

    mappedPointsNode.addObject('QPSlidingConstraint', name="QPConstraint")
    mappedPointsNode.addObject('DifferenceMultiMapping', name="pointsMulti", input1=inputFEMCableMO,
                               input2=inputCableMO, output=outputPointMO, direction="@../../FramesMO.position")
    # Get the tree mstate links for the mapping

    # ##########################################
    # # Cable points                           #
    # ##########################################
    # # Mappe points inside the meca, this points will be use for the bilateral mapping
    # FEMpos = [" 10 10 0 25 10 0 40 10 0 55 10 0 70 10 0 76 10 0 91 10 0"]

    # femPoints = femAnkle.addChild('femPoints')
    # inputFEMCable = femPoints.addObject('MechanicalObject', name="pointsInFEM", position=FEMpos, showObject="1",
    #                                     showIndices="1")
    # #Mapping using barycentric coordinates of the child with respect to cells of its parent
    # femPoints.addObject('BarycentricMapping')
    # #As explained above, a ConstraintCorrection is required in the 
    # # simulation to 
    # # define the way the compliance matrix is computed in this case is LinearSolverConstraintCorrection
    # femAnkle.addObject('LinearSolverConstraintCorrection')
    
    # # #############
    # # New adds to use the sliding Actuator
    # ###############

    # cableNode = rootNode.addChild('cableNode')
    # cableNode.addObject('EulerImplicitSolver', firstOrder="0",
    #                                             rayleighStiffness="0.1", rayleighMass='0.1')
    # #All dynamic simulations assume to discretize the temporal evolution of the system through small time steps. 
    # #This time step is usually noted dt. An integration scheme is the numerical method describing how to find the approximate solution for ordinary differential equations (ODE).

    # cableNode.addObject('SparseLUSolver', name='solver', template='CompressedRowSparseMatrixd')
    # #As explained above, a ConstraintCorrection is required in the simulation to 
    # # define the way the compliance matrix is computed in this case is GenericConstraintCorrection
    # cableNode.addObject('GenericConstraintCorrection')



    # beamGeometrie = {'init_pos': [10., 10., 0.], 'tot_length': 81, 'nbSectionS': 12,
    #                  'nbFramesF': 30, 'buildCollisionModel': 0, 'beamMass': 0.}
    
    # cosserat = cableNode.addChild(Cosserat(parent=cableNode, cosseratGeometry=beamGeometrie, radius=0.5,
    #                                        useCollisionModel=True, name="cosserat", youngModulus=5e6, poissonRatio=0.4,translation=[10., 10., 0.]))

    # cableNode.addObject(Animation(cosserat.rigidBaseNode.RigidBaseMO,
    #                               cosserat.cosseratCoordinateNode.cosseratCoordinateMO))
    # mappedFrameNode = cosserat.cosseratFrame

    # #  This create a new node in the scene. This node is appended to the finger's node.
    # slidingPoint = mappedFrameNode.addChild('slidingPoint')

    # # This create a MechanicalObject, a componant holding the degree of freedom of our
    # # mechanical modelling. In the case of a cable it is a set of positions specifying
    # # the points where the cable is passing by.
    # slidingPointMO = slidingPoint.addObject('MechanicalObject', name="cablePos",
    #                                         position=cosserat.frames3D, showObject="1", showIndices="1")
    # #the identity function directly maps each element to itself without any modification.
    # slidingPoint.addObject('IdentityMapping')

    # mappedPointsNode = slidingPoint.addChild('MappedPoints')
    # femPoints.addChild(mappedPointsNode)
    # mappedPoints = mappedPointsNode.addObject('MechanicalObject', template='Vec3d', position=FEMpos,
    #                                           name="FramesMO", showObject='1', showObjectScale='1')

    # inputCableMO = slidingPointMO.getLinkPath()
    # inputFEMCableMO = inputFEMCable.getLinkPath()
    # outputPointMO = mappedPoints.getLinkPath()

    # mappedPointsNode.addObject('QPSlidingConstraint', name="QPConstraint")
    # mappedPointsNode.addObject('DifferenceMultiMapping', name="pointsMulti", input1=inputFEMCableMO,
    #                            input2=inputCableMO, output=outputPointMO, direction="@../../FramesMO.position")
    # # Get the tree mstate links for the mapping
    return rootNode
