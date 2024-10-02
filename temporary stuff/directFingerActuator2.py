# -*- coding: utf-8 -*-

"""_summary_ Basic scene using Cosserat in SofaPython3.
    The Finger is modeled usind FEM modèle while de cable is modeled using cosserat theory.
    The link between these two meachanical models is constraint based using Lagrangian Multiplier

Returns:
    _type_: _description_
"""


__authors__ = "younesssss"
__contact__ = "adagolodjo@protonmail.com, yinoussa.adagolodjo@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020,Inria"
__date__ = "March 8 2020"

import SofaRuntime
import Sofa
import os
from splib3.numerics import Quat
from cosserat.usefulFunctions import buildEdges, pluginList, BuildCosseratGeometry
from cosserat.cosseratObject import Cosserat
from controler import FingerController
# from stlib3.scene import Node



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

class Animation2(Sofa.Core.Controller):
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
        if ord(key) == 9:  # up
            with self.rigidBaseMO.rest_position.writeable() as posA:
                qOld = Quat()
                for i in range(4):
                    qOld[i] = posA[0][i+3]

                self._extracted_from_onKeypressedEvent_10(
                     qOld, posA, self.angularRate)

        if ord(key) == 1:  # down
            with self.rigidBaseMO.rest_position.writeable() as posA:
                qOld = Quat()
                for i in range(4):
                    qOld[i] = posA[0][i+3]

                self._extracted_from_onKeypressedEvent_10(
                    qOld, posA, -self.angularRate)

        if ord(key) == 19:  # left
            with self.rigidBaseMO.rest_position.writeable() as posA:
                posA[0][0] -= self.rate
        if ord(key) == 21:  # right
            with self.rigidBaseMO.rest_position.writeable() as posA:
                posA[0][0] += self.rate


def Finger(parentNode=None, name="Finger",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[-18, -15, -8, 2, -3, 8]):
    from stlib3.physics.deformable import ElasticMaterialObject
    from stlib3.physics.constraints import FixedBox
    Ankle = parentNode.addChild(name)
    femAnkle = ElasticMaterialObject(Ankle,
                                    volumeMeshFileName="finger.vtk",
                                    poissonRatio=0.3,
                                    youngModulus=18000,
                                    totalMass=1.0,
                                    surfaceColor=[0.0, 0.7, 0.7, 1.0],
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

def Cosseratcable(parentNode=None,attachTo=None,FEMpos=[],name="1",
                    cosseratGeometry={'init_pos': [0., 0., 0.], 'tot_length': 81, 'nbSectionS': 12,
                    'nbFramesF': 30, 'buildCollisionModel': 0, 'beamMass': 0.}
                    ,radius=0.5,
                    useCollisionModel=False, youngModulus=5e6, 
                    poissonRatio=0.4):
    ##########################################
    # Cable points                           #
    ##########################################

    femPoints = attachTo.addChild('femPoints'+name)
    inputFEMCable = femPoints.addObject('MechanicalObject', name="pointsInFEM", position=FEMpos, showObject="1",
                                        showIndices="1")
    femPoints.addObject('BarycentricMapping')

    # ###############
    # New adds to use the sliding Actuator
    ###############
    cableNode = parentNode.addChild('cableNode'+name)
    cableNode.addObject('EulerImplicitSolver', firstOrder="0",
                        rayleighStiffness="0.1", rayleighMass='0.1')
    cableNode.addObject('SparseLUSolver', name='solver', template='CompressedRowSparseMatrixd')
    cableNode.addObject('GenericConstraintCorrection')

    cosserat = cableNode.addChild(Cosserat(parent=cableNode, cosseratGeometry=cosseratGeometry, radius=0.5,
                                           useCollisionModel=useCollisionModel, name="cosserat", youngModulus=youngModulus, poissonRatio=poissonRatio, translation=cosseratGeometry["init_pos"]))
    if name=="2":
        # cableNode.addObject(Animation2(cosserat.rigidBaseNode.RigidBaseMO,
        #                           cosserat.cosseratCoordinateNode.cosseratCoordinateMO))
        ...
    else:
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

def createScene(rootNode):

    from stlib3.scene import MainHeader
    #agregamos los plugins necesarios
    m=MainHeader(rootNode, plugins=pluginList,
               repositoryPaths=[os.getcwd()])
    rootNode.addObject('RequiredPlugin', name='CSparseSolvers') # Needed to use components [SparseLUSolver]  
    #activamos algunos flag interesantes para la visualizacion de la simulacion, esto se puede cambiar en plena simulacion
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels "
    m.getObject("VisualStyle").displayFlags = 'showForceFields showBehaviorModels showInteractionForceFields showMechanicalMappings'
    #Premite el movimiento en la simulación
    rootNode.addObject('FreeMotionAnimationLoop')
    #Permite activar un solver para las restricciones y las colisiones
    rootNode.addObject('GenericConstraintSolver',
                       tolerance="1e-20", maxIterations="500", printLog="0")

    gravity = [0, 0, 0]
    #asignamos el valore  de la gravedad
    rootNode.gravity.value = gravity
    #configuración del fondo
    rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
    #permite la asignacion del frame principal
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    finger =Finger(rootNode,translation=[-7.5,-2.5,7.5],rotation=[0.0, 180.0, 0.0],fixingBox=[-8 ,-5 ,-8 ,7, 7 ,8])
    

    ##########################################
    # Cable points                           #
    ##########################################
    # Mappe points inside the meca, this points will be use for the bilateral mapping
    FEMpos = [" 10 10 0 25 10 0 40 10 0 55 10 0 70 10 0 76 10 0 91 10 0"]

    femPoints = finger.addChild('femPoints')
    inputFEMCable = femPoints.addObject('MechanicalObject', name="pointsInFEM", position=FEMpos, showObject="1",
                                        showIndices="1")
    femPoints.addObject('BarycentricMapping')

    beamGeometry = {'init_pos': [10., 10., 0.], 'tot_length': 81, 'nbSectionS': 12,
            'nbFramesF': 30, 'buildCollisionModel': 0, 'beamMass': 0.}
    
    Cosseratcable(FEMpos=FEMpos,parentNode=rootNode,attachTo=finger,cosseratGeometry=beamGeometry,radius=0.5,name="1")
 
    FEMpos2 = [" 10 0 0 25 0 0 40 0 0 55 0 0 70 0 0 76 0 0 91 0 0"]
    beamGeometry2 = {'init_pos': [10., 0., 0.], 'tot_length': 81, 'nbSectionS': 12,
            'nbFramesF': 30, 'buildCollisionModel': 0, 'beamMass': 0.}
    
    Cosseratcable(FEMpos=FEMpos2,parentNode=rootNode,attachTo=finger,cosseratGeometry=beamGeometry2,radius=0.5,name="2")


    # # ###############
    # # New adds to use the sliding Actuator
    # ###############
    # cableNode = rootNode.addChild('cableNode')
    # cableNode.addObject('EulerImplicitSolver', firstOrder="0",
    #                     rayleighStiffness="0.1", rayleighMass='0.1')
    # cableNode.addObject('SparseLUSolver', name='solver', template='CompressedRowSparseMatrixd')
    # cableNode.addObject('GenericConstraintCorrection')

    # beamGeometrie = {'init_pos': [10., 10., 0.], 'tot_length': 81, 'nbSectionS': 12,
    #                  'nbFramesF': 30, 'buildCollisionModel': 0, 'beamMass': 0.}
    # cosserat = cableNode.addChild(Cosserat(parent=cableNode, cosseratGeometry=beamGeometrie, radius=0.5,
    #                                        useCollisionModel=False, name="cosserat", youngModulus=5e6, poissonRatio=0.4, translation=[10., 10., 0.]))

    # cableNode.addObject(Animation(cosserat.rigidBaseNode.RigidBaseMO,
    #                               cosserat.cosseratCoordinateNode.cosseratCoordinateMO))
    # mappedFrameNode = cosserat.cosseratFrame

    # #  This create a new node in the scene. This node is appended to the finger's node.
    # slidingPoint = mappedFrameNode.addChild('slidingPoint')

    # # This create a MechanicalObject, a componant holding the degree of freedom of our
    # # mechanical modelling. In the case of a cable it is a set of positions specifying
    # # the points where the cable is passing by.
    # slidingPointMO = slidingPoint.addObject('MechanicalObject', name="cablePos",
    #                                         position=cosserat.frames3D, showObject="1", showIndices="0")
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