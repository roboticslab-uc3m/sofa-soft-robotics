
import Sofa.Core
from splib3.animation import animate, AnimationManager
from stlib3.visuals import VisualModel

from stlib3.physics.constraints import FixedBox
from softrobots.actuators import PullingCable
from splib3.loaders import loadPointListFromFile


import os
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'
dirPath = os.path.dirname(os.path.abspath(__file__))+'/'

import json
data = json.loads(open("../config/Finger_LQ.json").read())
cable1_data = json.loads(open("../config/cable1.json").read())
cable2_data = json.loads(open("../config/cable2.json").read())

from Finger_Controller import FingerController


#This is the goal
def effectorTarget(parentNode, position=[0., 120., 6]):
    goal = parentNode.addChild('Target')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
    goal.addObject('MechanicalObject', name='goalMO', position=position, showObject=True)
    #, showObjectScale=8, drawMode=2, showColor=[1., 1., 1., 1.])
    goal.addObject('SphereCollisionModel', radius=5)
    goal.addObject('UncoupledConstraintCorrection', defaultCompliance=1e-5)
    return goal

class Finger:
    """ This prefab is implementing a soft robot inspired by the elephant's finger.
            The robot is entirely soft and actuated with 8 cables.

            The prefab is composed of:
            - a visual model
            - a collision model
            - a mechanical model for the deformable structure

            The prefab has the following parameters:
            - youngModulus
            - poissonRatio
            - totalMass

            Example of use in a Sofa scene:

            def createScene(root):
                ...
                finger = finger(root)

                ## Direct access to the components
                finger.displacements = [0., 0., 0., 0., 5., 0., 0., 0.]
        """
    
    def __init__(self, parentNode, volumeMeshFileName, youngModulus, poissonRatio, totalMass, surfaceColor, surfaceMeshFileName, surfaceMeshFileNameTop, surfaceMeshFileNameBot, rotation, translation, scale, inverseMode=False):

        self.inverseMode = inverseMode
        self.node = parentNode.addChild('finger')

        self.node.addObject('MeshGmshLoader', 
                            name='loader', filename=volumeMeshFileName,
                            rotation=rotation, translation=translation,
                            scale3d=scale)

        self.node.addObject('MeshTopology', src='@loader', name='container')
        self.node.addObject('MechanicalObject', name='dofs', template='Vec3', showIndices=False, showIndicesScale=4e-5)

        # Gives a mass to the model
        self.node.addObject('UniformMass', totalMass=totalMass)

        # Add a TetrahedronFEMForceField component which implement an elastic material model solved using the Finite Element Method on
        #  tetrahedrons.
        self.node.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=poissonRatio,
                        youngModulus=youngModulus)
        
        # Add a fixingBox to the base of the finger
        self.node.addObject('BoxROI', name='roi', box=[15, -2, -15, -15, 5, 15], drawBoxes=True)
        self.node.addObject('RestShapeSpringsForceField', points=self.node.roi.indices.getLinkPath(), stiffness=1e12)
        
        # Add the cable and collision models to the platform
        self.__addCables()
        self.addCollisionModel(surfaceMeshFileNameTop, surfaceMeshFileNameBot, rotation, translation, scale)
    
    def __addCables(self):

         #  This creates a new node in the scene. This node is appended to the finger's node.
        cable1 = self.node.addChild('cable1')
        cable2 = self.node.addChild('cable2')

        #  This creates a MechanicalObject, a component holding the degree of freedom of our
        # mechanical modelling. In the case of a cable it is a set of positions specifying
        #  the points where the cable is passing by.
        cable1.addObject('MechanicalObject', name='cable1', position=cable1_data)
        cable2.addObject('MechanicalObject', name='cable2', position=cable2_data)
           
        # Create a CableConstraint object with a name.
        # the indices are referring to the MechanicalObject's positions.
        # The last index is where the pullPoint is connected.
        cable1.addObject('CableConstraint' if not self.inverseMode else 'CableActuator', template='Vec3', name="acableConstraint",
                        indices=list(range(0, 14)),
                        pullPoint=[-10.392, 0, 6],
                        maxPositiveDisp=0.1,
                        maxDispVariation=0.05,
                        minForce=0)
        cable2.addObject('CableConstraint' if not self.inverseMode else 'CableActuator', template='Vec3', name="aCableActuator",
                        indices=list(range(0, 14)), #all are 30
                        # indices=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
                        minForce=0,  # Set that the cable can't push
                        maxPositiveDisp=0.1,
                        maxDispVariation=0.05,
                        pullPoint=[10.392, 0, 6]
                        #[0, -5, 6]
                        )

        # This creates a BarycentricMapping. A BarycentricMapping is a key element as it will create a bidirectional link
        #  between the cable's DoFs and the finger's one's so that movements of the cable's DoFs will be mapped
        #  to the finger and vice-versa;
        cable1.addObject('BarycentricMapping')
        cable2.addObject('BarycentricMapping')

        # This creates a PythonScriptController that permits to programmatically implement new behavior
        #  or interactions using the Python programming language. The controller is referring to a
        #  file named "controller.py".
        # cable1.addObject(FingerController(name="FingerController", node=cable1))
        # cable2.addObject(FingerController(name="FingerController", node=cable2))

    def addVisualModel(self, surfaceMeshFileName, color=[1., 1., 1., 1.]):
        fingerVisu = self.node.addChild('VisualModel')
        fingerVisu.addObject('MeshSTLLoader', filename=surfaceMeshFileName)
        fingerVisu.addObject('OglModel', color=color)
        fingerVisu.addObject('BarycentricMapping')

    def addCollisionModel(self, surfaceMeshFileNameTop, surfaceMeshFileNameBot, rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0], scale=[1.0, 1.0, 1.0], selfCollision=False):
        fingerColli = self.node.addChild('CollisionModel')
        for i in range(2):
            part = fingerColli.addChild('Part'+str(i+1))
            part.addObject('MeshSTLLoader', name='loader', filename=surfaceMeshFileNameTop if i==1 else surfaceMeshFileNameBot, rotation=rotation, translation=translation, scale3d=scale)
            part.addObject('MeshTopology', src='@loader')
            part.addObject('MechanicalObject')
            part.addObject('TriangleCollisionModel', group=1 if not selfCollision else i)
            part.addObject('LineCollisionModel', group=1 if not selfCollision else i)
            part.addObject('PointCollisionModel', group=1 if not selfCollision else i)
            part.addObject('BarycentricMapping')
                                                   
    def addEffectors(self, target, position=[0, 100, 6]):
        effectors = self.node.addChild('Effectors')
        # effectors.addObject('BoxROI', name='roi_fingertip', box=[-2, 96, -2, 2, 99, 2], drawBoxes=False)
        effectors.addObject('MechanicalObject', position=position)

        #PositionEffector is a component from the Inverse.SoftRobots plugin -> https://project.inria.fr/softrobot/documentation/constraint/position-effector/
        effectors.addObject('PositionEffector', indices=0, template = 'Vec3', effectorGoal=target)
        effectors.addObject('BarycentricMapping', mapForces=False, mapMasses=False)


def createScene(rootNode):

    # Choose your resolution mode
    # 1- inverseMode=True, solve the inverse problem and control the end effectors
    # 2- inverseMode=False, solve the direct problem and set the cable displacements by hand
    inverseMode = True

    from stlib3.scene import MainHeader, ContactHeader
    # ContactHeader(rootNode, alarmDistance=3, contactDistance=0.5, frictionCoef=1)
    
    #Collision related stuff
    rootNode.addObject('DefaultPipeline', name='CollisionPipeline')
    rootNode.addObject('BruteForceBroadPhase', name='BroadPhase')
    rootNode.addObject('BVHNarrowPhase', name='NarrowPhase')
    rootNode.addObject('DefaultContactManager', name='ContactManager', response='FrictionContactConstraint')
    # rootNode.addObject('DefaultCollisionGroupManager', name='GroupManager')
    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance=10, contactDistance=4)

    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('RequiredPlugin', pluginName=[
                            "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
                            "Sofa.Component.Constraint.Lagrangian.Correction",  # Needed to use components GenericConstraintCorrection
                            "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
                            "Sofa.Component.Constraint.Projective",  # Needed to use components PartialFixedConstraint
                            "Sofa.Component.Engine.Select",  # Needed to use components BoxROI
                            "Sofa.Component.IO.Mesh",  # Needed to use components MeshSTLLoader, MeshVTKLoader
                            "Sofa.Component.LinearSolver.Direct",  # Needed to use components SparseLDLSolver
                            "Sofa.Component.LinearSolver.Iterative",  # Needed to use components ShewchukPCGLinearSolver
                            "Sofa.Component.Mass",  # Needed to use components UniformMass
                            "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                            "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
                            "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
                            "Sofa.Component.Visual",  # Needed to use components VisualStyle
                            "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel
                            "Sofa.Component.Collision.Geometry", # Needed to use components [LineCollisionModel,PointCollisionModel,SphereCollisionModel,TriangleCollisionModel]  
                            "Sofa.Component.Mapping.Linear", # Needed to use components [BarycentricMapping]  
                            "Sofa.Component.SolidMechanics.Spring", # Needed to use components [RestShapeSpringsForceField]  
                            "Sofa.Component.StateContainer", # Needed to use components [MechanicalObject],
                            "Sofa.Component.Collision.Detection.Algorithm", # Needed to use components [BVHNarrowPhase,BruteForceBroadPhase,CollisionPipeline]
                            "Sofa.Component.Collision.Detection.Intersection", # Needed to use components [LocalMinDistance]
                            "Sofa.Component.Collision.Response.Contact", # Needed to use components [RuleBasedContactManager]"
                        ])
    AnimationManager(rootNode)
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('VisualStyle', displayFlags='showCollisionModels showForceFields showBehaviorModels showInteractionForceFields')
    rootNode.gravity = [0., -9810., 0.]

    rootNode.addObject('FreeMotionAnimationLoop')
    if inverseMode:
        # For inverse resolution, i.e control of effectors position
        rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
        rootNode.addObject('QPInverseProblemSolver', epsilon=1e-1)
    else:
        # For direct resolution, i.e direct control of the cable displacement
        rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance=1e-5)

    simulation = rootNode.addChild('Simulation')

    simulation.addObject('EulerImplicitSolver', name='odesolver', firstOrder=False, rayleighMass=0.1, rayleighStiffness=0.1)
    simulation.addObject('SparseLDLSolver', name='precond')
    simulation.addObject('GenericConstraintCorrection')

    finger = Finger(simulation, inverseMode=True,
                                volumeMeshFileName="../models/" + data["volumeMeshFileName"],
                                poissonRatio=data["poissonRatio"],
                                youngModulus=data["youngModulus"],
                                totalMass=data["totalMass"],
                                surfaceColor=data["surfaceColor"],
                                surfaceMeshFileName="../models/Low_quality_collision_mesh/" + data["surfaceMeshFileName"],
                                rotation=data["rotation"],  
                                translation=data["translation"],
                                scale=data["scale"],
                                surfaceMeshFileNameTop="../models/Low_quality_collision_mesh/" + data["CollisionMeshTop"],
                                surfaceMeshFileNameBot="../models/Low_quality_collision_mesh/" + data["CollisionMeshBot"])
    finger.addVisualModel(surfaceMeshFileName="../models/Low_quality_collision_mesh/" + data["surfaceMeshFileName"],
        color=data["surfaceColor"])
    
    if inverseMode:
        # For inverse resolution, set a constraint for the effector and a target

        goal = effectorTarget(rootNode)
        # finger.addEffectors(target=goal.goalMO.getData('position').getLinkPath(), position=[[0, 120, 6]])
        finger.addEffectors(target=goal.goalMO.getData('position').getLinkPath(), position=[[0, 120, 6]])


    # Use this in direct mode as an example of animation ############
    def cableanimation(target, factor):
         target.cable.value = factor*20
    
    # animate(cableanimation, {'target': finger.cableL0}, duration=2, )
    #################################################################

