# Import modules and libraries 
import socket
import json
import time
import numpy as np

# Import Sofa related modules
import Sofa.Core
import Sofa.constants.Key as Key

#Define the target position
target = np.array([-20.0, 40.0, 40.0])

# TCP socket configuration
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('localhost', 12345))    # Currently this port is used to communicate with Matlab
server_socket.listen(1)

class FingerController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0][0]
        self.cable2 = args[0][1]
        self.endeffector=args[1]
        # self.target= args[2][2][2]
        self.dts=0.02
        self.t=0
        self.error=np.array([0,0,0])
        self.realPosition=np.array(self.endeffector.position.value)[0]
        self.rate=10
        self.name = "FingerController"
        #self.recorded_positions = []  # Store positions for each frame (Recording the simulation)
        #self.frame_number = 0
     #  self.start_conection()  #This begins the communication with Matlab

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
    
    def onAnimateBeginEvent(self, event): # called at each begin of animation step
        self.t=self.t+self.dts
        #self.realPosition=np.array(self.endeffector.position.value)[0]
        #self.send(self.realPosition.tolist())
        self.error = 0.05*(target-self.realPosition) # kp=1 for the finger
        #self.error=self.receive()
        print(type(self.error),"-----------------------------------------------------------")
        if self.error[0]!=0:
            self.pitch(self.error[2])

        if self.error[2]!=0:
            self.roll(self.error[0])
        positions = np.array(self.mechanicalObject.position.array())
        self.recorded_positions.append(positions.copy())
        print(f"Recording frame {self.frame_number}")
        self.frame_number += 1

    def pitch(self,value):#turn axe x (difference in z cartesian values)
        displacement = self.cable.CableConstraint.value[0]
        displacement2 = self.cable2.CableConstraint.value[0]
        rate=abs(value)
        if value>0:
            displacement += rate
            displacement2 += rate
        else:
            displacement2 -= rate
            displacement -= rate
        self.cable.CableConstraint.value = [displacement]
        self.cable2.CableConstraint.value = [displacement2]
    def roll(self,value):#turn axe z (difference in x cartesian values)
        displacement = self.cable.CableConstraint.value[0]
        displacement2 = self.cable2.CableConstraint.value[0]
        rate=abs(value)
        if value>0:
            #displacement -= rate #think this needs to de deleted
            displacement2 += rate
        else:
            #displacement2 -= rate #think this needs to de deleted
            displacement += rate
        self.cable.CableConstraint.value = [displacement]
        self.cable2.CableConstraint.value = [displacement2]

    def save_to_file(self, filename="simulation_record.npz"):
        # Save the recorded positions to a file for later use
        np.savez_compressed(filename, positions=self.recorded_positions)
        print(f"Recorded simulation saved to {filename}")
    
    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        displacement2 = self.cable2.CableConstraint.value[0]
        rate=100
        if e["key"] == Key.S:
            self.save_to_file("simulation_record.npz")
        # if e["key"] == Key.rightarrow:
        #     displacement += rate
        #     displacement3 -= rate

        # elif e["key"] == Key.leftarrow:
        #     displacement3 += rate
        #     displacement -= rate

        # if e["key"] == Key.uparrow:
        #     displacement2 += rate
        #     displacement4 -= rate

        # elif e["key"] == Key.downarrow:
        #     displacement4 += rate
        #     displacement2 -= rate
        # # print("C1",displacement)
        # self.cable.CableConstraint.value = [displacement]
        # self.cable2.CableConstraint.value = [displacement2]
        # self.cable3.CableConstraint.value = [displacement3]
        # self.cable4.CableConstraint.value = [displacement4]
        # if displacement >= 0:   
        #     self.cable.CableConstraint.value = [displacement]