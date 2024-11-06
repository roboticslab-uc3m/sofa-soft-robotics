# Import the required modules and libraries
import json
from softrobots.actuators import PullingCable
from splib3.loaders import loadPointListFromFile
from Finger_Controller import FingerController 
from stlib3.physics.collision import CollisionMesh

# Function to create the Finger model
def Finger(parentNode=None, name="Finger"):
    Finger = parentNode.addChild(name)
    from stlib3.physics.deformable import ElasticMaterialObject
    #from CUDAelasticmaterialobject import ElasticMaterialObject         #for trying the simulation with CUDA
    from stlib3.physics.constraints import FixedBox
    data = json.loads(open("../config/Finger_LQ.json").read())
    # if 'rotation' in data:
    #     data['rotation'] = [float(value) for value in data['rotation']]
    femFinger = ElasticMaterialObject(Finger,
                                    volumeMeshFileName="../models/" + data["volumeMeshFileName"],
                                    poissonRatio=data["poissonRatio"],
                                    youngModulus=data["youngModulus"],
                                    totalMass=data["totalMass"],
                                    surfaceColor=data["surfaceColor"],
                                    surfaceMeshFileName="../models/Low_quality_collision_mesh/" + data["surfaceMeshFileName"],
                                    rotation=data["rotation"],  
                                    translation=data["translation"],
                                    scale=data["scale"])
    Finger.addChild(femFinger)
    # Finger.addObject('GenericConstraintCorrection')
    # femFinger.addObject('GenericConstraintCorrection')

    # Finger.addObject('LinearSolverConstraintCorrection')

    CollisionMesh(femFinger, name="CollisionMesh",
                  surfaceMeshFileName="../models/Low_quality_collision_mesh/" + data["surfaceMeshFileName"],
                  rotation=data["rotation"], translation=data["translation"],
                  collisionGroup=[1, 2])

    CollisionMesh(femFinger, name="CollisionMeshAuto1",
                  surfaceMeshFileName="../models/Low_quality_collision_mesh/" + data["CollisionMeshTop"],
                  rotation=data["rotation"], translation=data["translation"],
                  collisionGroup=[1])

    CollisionMesh(femFinger, name="CollisionMeshAuto2",
                  surfaceMeshFileName="../models/Low_quality_collision_mesh/" + data["CollisionMeshBot"],
                  rotation=data["rotation"], translation=data["translation"],
                  collisionGroup=[2])
    FixedBox(femFinger,
             doVisualization=True,
             atPositions=data["fixingBox"])
    cable =femFinger.addChild("cable")
    cables=[]
    cables.append(PullingCable(cable, valueType="force", name="cable1", cableGeometry=loadPointListFromFile("../config/cable1.json"))) #, color=[1, 1, 1, 1]
    cables.append(PullingCable(cable, valueType="force", name="cable2", cableGeometry=loadPointListFromFile("../config/cable2.json"))) #, color=[0, 0, 1, 1]
    
    #Cable actuator 1
    cable.addObject('MechanicalObject', template='Vec3d', position=loadPointListFromFile("../config/cable1.json"), name="cable1")
    cable.addObject('CableActuator', indices=list(range(len(loadPointListFromFile("../config/cable1.json")))), pullPoint=[10.392, 2.0, 6.0], maxPositiveDisp=40, maxDispVariation=0.5, minForce=0)
    cable.addObject('BarycentricMapping')
    cables.append(cable)
    #Cable actuator 2   
    cable.addObject('MechanicalObject', template='Vec3d', position=loadPointListFromFile("../config/cable2.json"), name="cable2")
    cable.addObject('CableActuator', indices=list(range(len(loadPointListFromFile("../config/cable2.json")))), pullPoint=[-10.392, 2.0, 6.0], maxPositiveDisp=40, maxDispVariation=0.5, minForce=0)
    cable.addObject('BarycentricMapping')
    cables.append(cable)

    goal = parentNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
    goal.addObject('MechanicalObject', name='goalMO',
                   position=[0, 110, 0])
    goal.addObject('SphereCollisionModel', radius=5)
    goal.addObject('UncoupledConstraintCorrection')

    effector = femFinger.addChild('fingertip')
    effector.addObject('MechanicalObject', position=[0, 98, 0],name="endeffector") # The finger has a length of 98mm, so the end effector is placed slightly above it. Needs to be changed when a MOCAP marker is placed
    effector.addObject('PositionEffector', template='Vec3', name="Efector_final",
                       indices=0,
                       effectorGoal="@../../../goal/goalMO.position")
    # print(f"Target value: {target}")
    effector.addObject('BarycentricMapping', mapForces=True, mapMasses=False)

    # Finger.addObject(FingerController(cables,effector.endeffector,target))
    return femFinger