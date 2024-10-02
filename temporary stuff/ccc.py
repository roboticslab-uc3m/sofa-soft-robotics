# -*- coding: utf-8 -*-
# units seem to be mm, kg?

#in ccc the two cables are part of the same mechanical object with different indexes

import os
import sys
import Sofa
import Sofa.Core
import SofaRuntime
# from FingerController import FingerController
path = os.path.dirname(os.path.abspath(__file__)) 



from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile


def main():
        rootNode=Sofa.Core.Node("rootNode")
        createScene(rootNode)
        Sofa.Simulation.init(rootNode)

        Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(rootNode, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        Sofa.Gui.GUIManager.MainLoop(rootNode)
        Sofa.Gui.GUIManager.closeGUI()

if __name__ == '__main__':
        main()


class MoveControlPoint(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.finger = kwargs.get("finger")

    def onAnimateBeginEvent(self, event):
        ...
        print(self.finger.fingertip.dd.position.value)



def createScene(rootNode):
    rootNode.addObject("RequiredPlugin", name='SoftRobots')
    rootNode.addObject("RequiredPlugin", name='SoftRobots.Inverse')
    rootNode.addObject("RequiredPlugin", name='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='SofaValidation') # Needed to use components [Monitor]  
    rootNode.addObject('RequiredPlugin', pluginName=[
                            "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
                            "Sofa.Component.Constraint.Lagrangian.Correction",  # Needed to use components GenericConstraintCorrection
                            "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
                            "Sofa.Component.Engine.Select",  # Needed to use components BoxROI
                            "Sofa.Component.IO.Mesh",  # Needed to use components MeshSTLLoader, MeshVTKLoader
                            "Sofa.Component.LinearSolver.Direct",  # Needed to use components SparseLDLSolver
                            "Sofa.Component.Mass",  # Needed to use components UniformMass
                            "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                            "Sofa.Component.Setting",  # Needed to use components BackgroundSetting
                            "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
                            "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
                            "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
                            "Sofa.Component.Visual",  # Needed to use components VisualStyle
                            "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel, OglSceneFrame
                            "Sofa.Component.Collision.Geometry", # Needed to use components [SphereCollisionModel]  
                            "Sofa.Component.Mapping.Linear", # Needed to use components [BarycentricMapping] 
                            "Sofa.Component.LinearSolver.Iterative", # Needed to use components [CGLinearSolver]  
                            "Sofa.Component.StateContainer" # Needed to use components [MechanicalObject]  
                        ])
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    # Add a QPInverseProblemSolver to the scene if you need to solve inverse problem like the one that involved
    # when manipulating the robots by specifying their effector's position instead of by direct control
    #  of the actuator's parameters.
    rootNode.addObject('QPInverseProblemSolver', printLog=False)
    # Otherwise use a GenericConstraintSolver
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-5, maxIterations=100)

    #rootNode.gravity = [0, -9810, 0]
    rootNode.dt = 0.01

    ##########################################
    # FEM Model                              #
    ##########################################
    finger = rootNode.addChild('finger')
    finger.addObject('EulerImplicitSolver', name='odesolver', rayleighMass=0.1, rayleighStiffness=0.1)
    finger.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')

    # Add a component to load a VTK tetrahedral mesh and expose the resulting topology in the scene .
    finger.addObject('MeshVTKLoader', name='loader', filename=path + '/ankle.vtk')
    finger.addObject('MeshTopology', src='@loader', name='container')

    # Create a MechanicaObject component to stores the DoFs of the model
    finger.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=True, showIndicesScale=0.01)

    # Gives a mass to the model
    finger.addObject('UniformMass', totalMass=0.075)

    # Add a TetrahedronFEMForceField component which implement an elastic material model solved using the Finite Element Method on
    #  tetrahedrons.
    finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=18000)

    # To facilitate the selection of DoFs, SOFA has a concept called ROI (Region of Interest).
    #  The idea is that ROI component "select" all DoFS that are enclosed by their "region".
    # We use ROI here to select a group of finger's DoFs that will be constrained to stay
    # at a fixed position.
    # You can either use 'BoxROI'...
    finger.addObject('BoxROI', name='roi', box=[15, -2, -15, -15, 5, 15], drawBoxes=True)
    # Or 'SphereROI'...
    # finger.addObject('SphereROI', name='roi', centers=[0, 0, 0], radii=5)
    finger.addObject('BoxROI', name='roi_fingertip', box=[25, 102, 25, -25, 98, -25], drawBoxes=True)

    # RestShapeSpringsForceField is one way in Sofa to implement fixed point constraint.
    # Here the constraints are applied to the DoFs selected by the previously defined BoxROI
    finger.addObject('RestShapeSpringsForceField', points=finger.roi.indices.getLinkPath(), stiffness=1e12)

    # It is also possible to simply set by hand the indices of the points you want to fix.
    # finger.addObject('RestShapeSpringsForceField', points=[0, 1, 2, 11, 55], stiffness=1e12)

    finger.addObject('GenericConstraintCorrection')
    #finger.addObject('MechanicalObject', name="myParticle", position="0 100 0 0 0 0 1", showObject=True)

    # Monitoring the fingertip
    finger.addObject('Monitor', template = 'Vec3', name ='position_fingertip', listening='1', indices=finger.roi_fingertip.indices.getLinkPath(), showPositions='1',
                     PositionsColor="1 1 0 1", VelocitiesColor="1 1 0 1", ForcesColor="1 1 0 1",
                     showMinThreshold="0.01", TrajectoriesPrecision="0.1", TrajectoriesColor="1 1 0 1" 
                     #, ExportPositions="True"
                     )
    print("sdddddddddddddddd: ",finger.roi_fingertip.indices.value)
    # finger.addObject('Monitor', template = 'Vec3', name ='velocities_307', listening='1', indices=finger.roi_fingertip.indices.getLinkPath(), showPositions='0',
    #                  showVelocities='1', PositionsColor="1 1 0 1", VelocitiesColor="1 1 0 1", ForcesColor="1 1 0 1",
    #                  showMinThreshold="0.01", TrajectoriesPrecision="0.1", TrajectoriesColor="1 1 0 1" 
    #                  #, ExportPositions="True"
    #                  )
    
    # finger.addObject('Monitor', template = 'Vec3', name ='trajectories_307', listening='1', indices=finger.roi_fingertip.indices.getLinkPath(), showPositions='0',
    #                  showVelocities='0', showTrajectories='1', PositionsColor="1 1 0 1", VelocitiesColor="1 1 0 1", ForcesColor="1 1 0 1",
    #                  showMinThreshold="0.01", TrajectoriesPrecision="0.1", TrajectoriesColor="1 1 0 1" 
    #                  #, ExportPositions="True"
    #                  )


    ##########################################
    # Cable                                  #
    ##########################################

    #  This creates a new node in the scene. This node is appended to the finger's node.
    cables = finger.addChild('cables')

    #  This creates a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    #  the points where the cable is passing by.
    cables.addObject('MechanicalObject', template = "Vec3", 
        position= loadPointListFromFile("cable1_2.json") )
        
    

    # Create a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last index is where the pullPoint is connected.
    cables.addObject('CableActuator', template = "Vec3", name = "cable_motor_1",
                    indices=list(range(0, 3)),
                    pullPoint=[26, 3, 26],
                    #maxPositiveDisp=40,
                    maxDispVariation=0.5,
                    minForce=0,     #to prevent the cable from moving backwards
                    )
    
    cables.addObject('CableActuator', template = "Vec3", name = "cable_motor_2",
                    indices=list(range(4, 7)),
                    pullPoint=[26, 3, -26],
                    #maxPositiveDisp=40,
                    maxDispVariation=0.5,
                    minForce=0,     #to prevent the cable from moving backwards
                    )


    # This creates a BarycentricMapping. A BarycentricMapping is a key element as it will create a bidirectional link
    #  between the cable's DoFs and the finger's one's so that movements of the cable's DoFs will be mapped
    #  to the finger and vice-versa;
    cables.addObject('BarycentricMapping')

    # This creates a PythonScriptController that permits to programmatically implement new behavior
    #  or interactions using the Python programming language. The controller is referring to a
    #  file named "controller.py".
    #cables.addObject(FingerController(name="FingerController", node="cable_motor_1"))

    # #Internal collisions
    # collisionmodel = finger.addChild("collision")
    # collisionmodel.addObject('MeshSTLLoader', filename=path + "dedo2_2d.stl", name="loader", 
    #                             rotation=rotation, translation=translation)
    # collisionmodel.addObject('MeshTopology', src="@loader")
    # fingerVisu.addObject('MeshSTLLoader', filename=path + "dedo2_2d.stl", name="loader")

    # CollisionMesh(eobject,
    #               surfaceMeshFileName="data/mesh/finger.stl", name="part0", collisionGroup=[1, 2])

    # CollisionMesh(eobject,
    #               surfaceMeshFileName="data/mesh/fingerCollision_part1.stl",
    #               name="CollisionMeshAuto1", collisionGroup=[1])

    # CollisionMesh(eobject,
    #               surfaceMeshFileName="data/mesh/fingerCollision_part2.stl",
    #               name="CollisionMeshAuto2", collisionGroup=[2])

    ##########################################
    # Visualization                          #
    ##########################################
    # In Sofa, visualization is handled by adding a rendering model.
    #  Create an empty child node to store this rendering model.
    fingerVisu = finger.addChild('visu')

    # Add to this empty node a rendering model made of triangles and loaded from a stl file.
    fingerVisu.addObject('MeshSTLLoader', filename=path + "/ankle.stl", name="loader")
    fingerVisu.addObject('OglModel', src="@loader", color=[1.0, 0.0, 0.0, 1])

    # Add a BarycentricMapping to deform the rendering model in a way that follow the ones of the parent mechanical model.
    fingerVisu.addObject('BarycentricMapping')

    ##########################################
    # Effector goal for interactive control  # This is the goal to be reached by the fingertip
    ##########################################
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5) #iterations: maximum number of iterations for solving the conjugate gradient, tolerance: convergence value, threshold: accuracy value; both cases it is 10um
    goal.addObject('MechanicalObject', name='goalMO',
                   position=[26, 120, 26])
    goal.addObject('SphereCollisionModel', radius=5)
    goal.addObject('UncoupledConstraintCorrection')
    
    ##########################################
    # Effector                               # This is the element of the robot that should reach this target
    ##########################################
    effector = finger.addChild('fingertip')
    effector.addObject('MechanicalObject', position=([0, 100, 0]),name="dd")
    effector.addObject('PositionEffector', template='Vec3', name="Efector_final",
                       indices=0,
                       effectorGoal=goal.goalMO.position.value)
    effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    rootNode.addObject( MoveControlPoint(name="DOFController", finger=finger) )
  
    return rootNode
