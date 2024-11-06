import Sofa.Core
from splib3.animation import animate, AnimationManager
import json
data = json.loads(open("../config/Finger_LQ.json").read())
cable1_data = json.loads(open("../config/cable1.json").read())
cable2_data = json.loads(open("../config/cable2.json").read())

from Finger_Class import Finger

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
                                volumeMeshFileName="../models/Low_quality_collision_mesh/" + data["volumeMeshFileName"],
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