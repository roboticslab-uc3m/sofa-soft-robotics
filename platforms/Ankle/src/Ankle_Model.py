# Import the required modules and libraries
import json
from softrobots.actuators import PullingCable
from splib3.loaders import loadPointListFromFile
from Ankle_Controller import AnkleController  
def Ankle(parentNode=None, name="Ankle"):
    Ankle = parentNode.addChild(name)
    from stlib3.physics.deformable import ElasticMaterialObject
    from stlib3.physics.constraints import FixedBox
    data = json.loads(open("../config/Ankle.json").read())
    femAnkle = ElasticMaterialObject(Ankle,
                                    volumeMeshFileName="../models/" + data["volumeMeshFileName"],
                                    poissonRatio=data["poissonRatio"],
                                    youngModulus=data["youngModulus"],
                                    totalMass=data["totalMass"],
                                    surfaceColor=data["surfaceColor"],
                                    surfaceMeshFileName="../models/" + data["surfaceMeshFileName"],
                                    rotation=data["rotation"],  
                                    translation=data["translation"],
                                    scale=data["scale"])
    Ankle.addChild(femAnkle)
    FixedBox(femAnkle,
             doVisualization=True,
             atPositions=data["fixingBox"])
    cable =femAnkle.addChild("cable")
    cables=[]
    cables.append(PullingCable(cable, valueType="force",name="cable1",cableGeometry=loadPointListFromFile("../config/cable1.json")))
    cables.append(PullingCable(cable, valueType="force",name="cable2", cableGeometry=loadPointListFromFile("../config/cable2.json")))
    cables.append(PullingCable(cable, valueType="force",name="cable3", cableGeometry=loadPointListFromFile("../config/cable3.json")))
    cables.append(PullingCable(cable, valueType="force",name="cable4", cableGeometry=loadPointListFromFile("../config/cable4.json")))

    effector = femAnkle.addChild('fingertip')
    effector.addObject('MechanicalObject', position=[0, 100, 0],name="endeffector")
    # effector.addObject('PositionEffector', template='Vec3', name="Efector_final",
    #                    indices=0,
    #                    effectorGoal=goal.goalMO.position.value)
    effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    Ankle.addObject(AnkleController(cables,effector.endeffector))
    return femAnkle