# Import the required modules and libraries
import json
from softrobots.actuators import PullingCable
from splib3.loaders import loadPointListFromFile
from Arm_Controller import ArmController 
from stlib3.physics.collision import CollisionMesh

# Function to create the Arm model
def Arm(parentNode=None, name="Arm"):
    Arm = parentNode.addChild(name)
    from stlib3.physics.deformable import ElasticMaterialObject
    from stlib3.physics.constraints import FixedBox
    data = json.loads(open("../config/Arm.json").read())
    # if 'rotation' in data:
    #     data['rotation'] = [float(value) for value in data['rotation']]
    femArm = ElasticMaterialObject(Arm,
                                    volumeMeshFileName="../models/" + data["volumeMeshFileName"],
                                    poissonRatio=data["poissonRatio"],
                                    youngModulus=data["youngModulus"],
                                    totalMass=data["totalMass"],
                                    surfaceColor=data["surfaceColor"],
                                    surfaceMeshFileName="../models/" + data["surfaceMeshFileName"],
                                    collisionMesh="../models/" + data["surfaceMeshFileName"],
                                    rotation=data["rotation"],  
                                    translation=data["translation"],
                                    scale=data["scale"])
    Arm.addChild(femArm)
    # CollisionMesh(femArm, name="CollisionMesh",
    #               surfaceMeshFileName="../models/" + data["surfaceMeshFileName"],
    #               rotation=data["rotation"], translation=data["translation"],
    #               collisionGroup=[1, 2])

    # CollisionMesh(femArm, name="CollisionMeshAuto1",
    #               surfaceMeshFileName="../models/" + data["surfaceMeshFileName"],
    #               rotation=data["rotation"], translation=data["translation"],
    #               collisionGroup=[1])

    # CollisionMesh(femArm, name="CollisionMeshAuto2",
    #               surfaceMeshFileName="../models/" + data["surfaceMeshFileName"],
    #               rotation=data["rotation"], translation=data["translation"],
    #               collisionGroup=[2])

    FixedBox(femArm,
             doVisualization=True,
             atPositions=data["fixingBox"])
    cable =femArm.addChild("cable")
    cables=[]
    cables.append(PullingCable(cable, valueType="force", name="cable1", cableGeometry=loadPointListFromFile("../config/cable1.json"))) #, color=[1, 1, 1, 1]
    cables.append(PullingCable(cable, valueType="force", name="cable2", cableGeometry=loadPointListFromFile("../config/cable2.json"))) #, color=[0, 0, 1, 1]

    effector = femArm.addChild('Armtip')
    effector.addObject('MechanicalObject', position=[0, 98, 0],name="endeffector") # The Arm has a length of 98mm, so the end effector is placed slightly above it. Needs to be changed when a MOCAP marker is placed
    # effector.addObject('PositionEffector', template='Vec3', name="Efector_final",
    #                    indices=0,
    #                    effectorGoal=goal.goalMO.position.value)
    effector.addObject('BarycentricMapping', mapForces=True, mapMasses=False)

    Arm.addObject(ArmController(cables,effector.endeffector))
    return femArm