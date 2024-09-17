from math import sqrt, pi
from splib3.objectmodel import SofaPrefab, SofaObject
from splib3.numerics import Vec3, Quat
from splib3.animation import animate, AnimationManager
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
from stlib3.physics.rigid import Floor, Cube
# from stlib3.scene import node
#from cosseratUtilities import compute_BeamLenght, createCurvAbsOutput, createFramesList, extractFEMConstraintPoints
import Sofa
import os
from gripperController import GripperController
from stlib3.physics.collision import CollisionMesh


class CosseratCable(SofaObject):
    def __init__(self, parentNode, name, trans=[0.0,0.0,0.0], rot=[0.,0.,0.]):
        self.name=name
        self.node = parentNode #.createChild(self.name)             
        self.cableLenght = 81.0
        self.numberBeams = 6
        self.numberFrame = 15
        self.stiffness="50000"
        self.angularStiffness=50000
#        rotLoc = [0.0,180.,0.] + rot
        self.rot= rot
#        transLoc = [-17.5,12.5,7.5] + trans
        self.trans = trans
        self.listOfBeamslenght = None
        self.crossSectionShape='circular'
        self.radius=0.5
        self.youngModulus=5e6    
        
        self.position             = []
        self.frames               = []
        self.cableDofMO           = None
        self.framesDofMO          = []
        self.curv_abs_input       = []
        self.curv_abs_output      = []
        self.cable_position       = []
        self.lenghts              = []
        self.ratePosition         = []
        self.slidingPoint         = None
        self.outputPointMO        = None
        self.mappedPointsNode     = None
        self.framesMO             = None
        
        #self.cable = self.node.createChild(self.name)
        self.__computeFrame()
        self.__computeRate()
        self.__addCables()    
    def __computeRate(self):
        ratePosition = []
        lenghts = []
        
        if self.listOfBeamslenght != None:
            lenghts = self.listOfBeamslenght
        else:
            beamLenght = self.cableLenght/self.numberBeams 
            for i in range(0,self.numberBeams):
                lenghts.append([beamLenght])
                
        for i in range(0,self.numberBeams):
            ratePosition.append([0.,0.,0.])
        
        self.lenghts = lenghts
        self.ratePosition = ratePosition

    def __computeFrame(self):
        framLenght = self.cableLenght/self.numberFrame
        
        frame = []
        curv_abs_output = []
        cable_position = []
        for i in range(0,self.numberFrame):
            x = float(i) * framLenght
            curv_abs_output.append(x)
            frame.append([x,0.,0., 0.,0.,0.,1.])
            cable_position.append([x,0.,0.])
        
        frame.append([self.cableLenght,0.,0., 0.,0.,0.,1.])
        curv_abs_output.append(self.cableLenght)
        cable_position.append([self.cableLenght,0.,0.])
        
        self.frames = frame
        self.curv_abs_output = curv_abs_output
        self.cable_position = cable_position
    def __addCables(self):
        
        rigidBaseNode = self.node.addChild('cableNode')               
        RigidBaseMO = rigidBaseNode.addObject('MechanicalObject', template='Rigid3d', name="RigidBaseMO", 
                                                 position="0. 0. 0. 0. 0. 0. 1.", 
#                                                 rest_position="0. 0. 0. 0. 0. 0. 1.", 
                                                 translation=self.trans, rotation=self.rot, showObject='1', showObjectScale='0.6',showIndices='1' )
        rigidBaseNode.addObject('RestShapeSpringsForceField', name='spring', stiffness="500", angularStiffness="500",
                                   external_points="0", mstate="@RigidBaseMO", points="0", template="Rigid3d")
        
        #############################################
        # Rate of angular Deformation 
        #############################################        
        rateAngularDeformNode = self.node.addChild('rateAngularDeform')
        rateAngularDeformMO = rateAngularDeformNode.addObject('MechanicalObject', template='Vec3d', name='rateAngularDeformMO', position=self.ratePosition)
        BeamHookeLawForce = rateAngularDeformNode.addObject('BeamHookeLawForceField', crossSectionShape=self.crossSectionShape, length=self.lenghts, radius=self.radius, youngModulus=self.youngModulus)
        
        ##############
        #   Frames   #
        ##############
        # the node of the frame needs to inherit from rigidBaseMO and rateAngularDeform
        mappedFrameNode = rigidBaseNode.addChild('MappedFrames')
        rateAngularDeformNode.addChild(mappedFrameNode)
        framesMO = mappedFrameNode.addObject(
            'MechanicalObject', template='Rigid3d', name="FramesMO", position=self.frames, showObject='1', showObjectScale='1')

        # The mapping has two inputs: RigidBaseMO and rateAngularDeformMO
        #                 one output: FramesMO
        inputMO = rateAngularDeformMO.getLinkPath()
        inputMO_rigid = RigidBaseMO.getLinkPath()
        #outputMO = framesMO.getLinkPath()
        self.framesMO = framesMO.getLinkPath()

        curv_abs_input = '0 15 30 45 60 66 81'
        #curv_abs_output = '0.0 5 10 15 20 30 35 40 45 55 60 66 71 76 81'
        mappedFrameNode.addObject('DiscreteCosseratMapping', curv_abs_input=curv_abs_input,
                                    curv_abs_output=self.curv_abs_output, input1=inputMO, input2=inputMO_rigid, output=self.framesMO, debug='0')
        
        #actuators = mappedFrameNode.createChild('actuators')
        #  This create a new node in the scene. This node is appended to the finger's node.
        slidingPoint = mappedFrameNode.addChild('slidingPoint')

        #This create a MechanicalObject, a componant holding the degree of freedom of our
        # mechanical modelling. In the case of a cable it is a set of positions specifying
        #the points where the cable is passing by.
        slidingPointMO = slidingPoint.addObject('MechanicalObject', name="cablePos",
                                position=self.cable_position, showObject="1", showIndices="1")
        slidingPoint.addObject('IdentityMapping')
        
        self.cableDofMO = slidingPointMO.getLinkPath()
        self.slidingPoint = slidingPoint
        
        mappedPointsNode = slidingPoint.addChild('MappedPoints')
        diffPosition = []
        for l in range(0, 7):
            diffPosition.append([0,0,0])
            
        mappedPoints = mappedPointsNode.addObject('MechanicalObject', template='Vec3d', position=diffPosition, name="FramesMO", showObject='1', showObjectScale='1')
        mappedPointsNode.addObject('QPSlidingConstraint', name="QPConstraint")
        
        self.outputPointMO = mappedPoints.getLinkPath()
        self.mappedPointsNode = mappedPointsNode