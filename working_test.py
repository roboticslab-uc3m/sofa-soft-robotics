from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
# -*- coding: utf-8 -*-
from softrobots.actuators import PullingCable
from splib3.loaders import loadPointListFromFile
import Sofa.Core
import Sofa.constants.Key as Key
import numpy as np
import socket
import json
import time
# Configuración del socket TCP
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('localhost', 12345))
server_socket.listen(1)

class AnkleController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0][0]
        self.cable2 = args[0][1]
        self.cable3 = args[0][2]
        self.cable4 = args[0][3]
        self.endeffector=args[1]
        self.dts=0.02
        self.t=0
        self.error=np.array([0,0,0])
        self.realPosition=np.array(self.endeffector.position.value)[0]
        self.rate=10
        self.name = "AnkleController"
        # self.start_conection()
        
    def start_conection(self):
        print("Esperando conexión de MATLAB...")
        self.conn, self.addr = server_socket.accept()
        print(f"Conectado a: {self.addr}")
    def send(self,positionEndEffector):
        
        posiciones_robot = {'posiciones': positionEndEffector}  # Aquí colocarías los datos reales de SOFA
        posiciones_json = json.dumps(posiciones_robot)
        # Enviar posiciones a MATLAB
        self.conn .sendall(posiciones_json.encode('utf-8'))
        
    def receive(self):
        datos_control = self.conn.recv(1024).decode('utf-8')
        controlSignal = json.loads(datos_control)
        print(f"Acciones de control recibidas: {controlSignal}")
        return controlSignal["actuadores"]
    
    # def onAnimateBeginEvent(self, event): # called at each begin of animation step
    #     self.t=self.t+self.dts
    #     self.realPosition=np.array(self.endeffector.position.value)[0]
    #     self.send(self.realPosition.tolist())
    #     self.error=self.receive()
    #     print(type(self.error),"-----------------------------------------------------------")
    #     if self.error[0]!=0:
    #         self.pitch(self.error[0])

    #     if self.error[2]!=0:
    #         self.roll(self.error[2])
    def pitch(self,value):#turn axe x
        displacement = self.cable.CableConstraint.value[0]
        displacement2 = self.cable2.CableConstraint.value[0]
        displacement3 = self.cable3.CableConstraint.value[0]
        displacement4 = self.cable4.CableConstraint.value[0]
        rate=abs(value)
        if value>0:
            displacement2 += rate
            displacement4 -= rate
        else:
            displacement4 += rate
            displacement2 -= rate
        self.cable.CableConstraint.value = [displacement]
        self.cable2.CableConstraint.value = [displacement2]
        self.cable3.CableConstraint.value = [displacement3]
        self.cable4.CableConstraint.value = [displacement4]
    def roll(self,value):#turn axe z
        displacement = self.cable.CableConstraint.value[0]
        displacement2 = self.cable2.CableConstraint.value[0]
        displacement3 = self.cable3.CableConstraint.value[0]
        displacement4 = self.cable4.CableConstraint.value[0]
        rate=abs(value)
        if value>0:
            displacement += rate
            displacement3 -= rate
        else:
            displacement3 += rate
            displacement -= rate
        self.cable.CableConstraint.value = [displacement]
        self.cable2.CableConstraint.value = [displacement2]
        self.cable3.CableConstraint.value = [displacement3]
        self.cable4.CableConstraint.value = [displacement4]
    
    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        displacement2 = self.cable2.CableConstraint.value[0]
        displacement3 = self.cable3.CableConstraint.value[0]
        displacement4 = self.cable4.CableConstraint.value[0]
        rate=100
        if e["key"] == Key.rightarrow:
            displacement += rate
            displacement3 -= rate

        elif e["key"] == Key.leftarrow:
            displacement3 += rate
            displacement -= rate

        if e["key"] == Key.uparrow:
            displacement2 += rate
            displacement4 -= rate

        elif e["key"] == Key.downarrow:
            displacement4 += rate
            displacement2 -= rate
        # print("C1",displacement)
        self.cable.CableConstraint.value = [displacement]
        self.cable2.CableConstraint.value = [displacement2]
        self.cable3.CableConstraint.value = [displacement3]
        self.cable4.CableConstraint.value = [displacement4]
        # if displacement >= 0:   
        #     self.cable.CableConstraint.value = [displacement]

def Ankle(parentNode=None, name="Ankle",
           rotation=[0.0, 45.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[-30, -10, -30, 30, 10, 30]):
    Ankle = parentNode.addChild(name)
    from stlib3.physics.deformable import ElasticMaterialObject
    from stlib3.physics.constraints import FixedBox
    femAnkle = ElasticMaterialObject(Ankle,
                                    volumeMeshFileName="ankle.vtk",
                                    poissonRatio=0.3,
                                    youngModulus=18000,
                                    totalMass=1.0,
                                    surfaceColor=[0.0, 0.8, 0.7, 1.0],
                                    surfaceMeshFileName="ankle.stl",
                                    rotation=rotation,
                                    translation=translation,
                                    scale=[1.0, 1.0, 1.0])
    
    Ankle.addChild(femAnkle)
    FixedBox(femAnkle,
             doVisualization=True,
             atPositions=fixingBox)
    cable =femAnkle.addChild("cable")
    cables=[]
    cables.append(PullingCable(cable, valueType="force",name="cable1",cableGeometry=loadPointListFromFile("cable1.json")))
    cables.append(PullingCable(cable, valueType="force",name="cable2", cableGeometry=loadPointListFromFile("cable2.json")))
    cables.append(PullingCable(cable, valueType="force",name="cable3", cableGeometry=loadPointListFromFile("cable3.json")))
    cables.append(PullingCable(cable, valueType="force",name="cable4", cableGeometry=loadPointListFromFile("cable4.json")))

    effector = femAnkle.addChild('fingertip')
    effector.addObject('MechanicalObject', position=[0, 100, 0],name="endeffector")
    # effector.addObject('PositionEffector', template='Vec3', name="Efector_final",
    #                    indices=0,
    #                    effectorGoal=goal.goalMO.position.value)
    effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    Ankle.addObject(AnkleController(cables,effector.endeffector))
    return femAnkle

def createScene(rootNode):
    # -*- coding: utf-8 -*-
    from stlib3.scene import MainHeader
    m = MainHeader(rootNode, plugins=["SoftRobots"])
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"

    m.getObject("VisualStyle").displayFlags = 'showForceFields showBehaviorModels showInteractionForceFields'
    m.addObject("FreeMotionAnimationLoop")
    m.addObject("GenericConstraintSolver", maxIterations=250, tolerance=1e-20)
    m.addObject('QPInverseProblemSolver', printLog=False)


    
    Ankle(rootNode)

    return rootNode

