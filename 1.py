import Sofa
import Sofa.Gui

def main():
# Call the SOFA function to create the root node
    root = Sofa.Core.Node("root")

    # Call the createScene function, as runSofa does
    createScene(root)

    # Once defined, initialization of the scene graph
    Sofa.Simulation.init(root)

    # Launch the GUI (qt or qglviewer)
    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 800)

    # Initialization of the scene will be done here
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()

def createScene(rootNode):
    totalMass = 1.0
    volume = 1.0
    inertiaMatrix=[1., 0., 0., 0., 1., 0., 0., 0., 1.]
    rootNode.addObject("VisualGrid", nbSubdiv=10, size=1000)
    finger = rootNode.addChild("finger")
    finger.addObject('EulerImplicitSolver', name='odesolver')
    finger.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05)
    finger.addObject('MechanicalObject', name="mstate", template="Rigid3", translation2=[0., 0., 0.], rotation2=[0., 0., 0.], showObjectScale=50)
    finger.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    finger.addObject('UncoupledConstraintCorrection')


    fingerCollision=finger.addChild("collision")
    fingerCollision.addObject('MeshVTKLoader', name='loader', filename='finger.vtk')
    fingerCollision.addObject('OglModel', name="model", src="@loader", scale3d=[1.0, 1.0, 1.0], color=[0., 1., 0.], updateNormals=True)
    fingerCollision.addObject('MeshTopology', src="@loader")
    fingerCollision.addObject('MechanicalObject')
    fingerCollision.addObject('TriangleCollisionModel')
    fingerCollision.addObject('LineCollisionModel')
    fingerCollision.addObject('PointCollisionModel')
    fingerCollision.addObject('RigidMapping')
        #### Visualization subnode for the sphere
    fingerVisu = finger.addChild("VisualModel")
    fingerVisu.loader = fingerVisu.addObject('MeshSTLLoader', name="loader", filename="finger.stl")
    fingerVisu.addObject('OglModel', name="model", src="@loader", scale3d=[1.0, 1.0, 1.0], color=[0., 1., 0.], updateNormals=True)
    fingerVisu.addObject('RigidMapping')