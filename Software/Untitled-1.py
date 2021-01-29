import ikpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
import pybullet as p
import time
import pybullet_data
import pyglet
import _thread
import time
from subprocess import Popen
import pyglet
from pyglet import shapes

# vars

lf_curretPosition = [10,0,-80]
rf_curretPosition = [10,0,-80]
lb_curretPosition = [10,0,-80]
rb_curretPosition = [10,0,-80]


# GUI
window = pyglet.window.Window()
pyglet.gl.glClearColor(0.7,0.7,0.7,1)
bg = pyglet.graphics.Batch()
feet = pyglet.graphics.Batch()


body = shapes.Rectangle(window.width//2, window.height//2, 90, 180, color=(255, 255, 20), batch=bg)
body.opacity = 255
body.anchor_position = (45,90)
center = shapes.Circle(window.width//2, window.height//2,10, color=(0, 0, 255), batch=bg)
center.opacity = 200
center.anchor_position = (0,0)


lbFoot = shapes.Circle(x=window.width//2-(90/2)+lb_curretPosition[1], y=window.height//2-(180/2)+lb_curretPosition[0], radius=10, color=(0, 0, 0),batch=feet)
rbFoot = shapes.Circle(x=window.width//2+(90/2)+rb_curretPosition[1], y=window.height//2-(180/2)+rb_curretPosition[0], radius=10, color=(0, 0, 0),batch=feet)
lfFoot = shapes.Circle(x=window.width//2-(90/2)+lf_curretPosition[1], y=window.height//2+(180/2)+lf_curretPosition[0], radius=10, color=(0, 0, 0),batch=feet)
rfFoot = shapes.Circle(x=window.width//2+(90/2)+rf_curretPosition[1], y=window.height//2+(180/2)+rf_curretPosition[0], radius=10, color=(0, 0, 0),batch=feet)

lbFoot.opacity = 100
rbFoot.opacity = 100
lfFoot.opacity = 100
rfFoot.opacity = 100
##body.anchor_position = (5,5)

@window.event
def on_draw():
  window.clear()
  bg.draw()
  feet.draw()




#Popen(["python", "Documents\GitHub\RobotDog\Software\BalanceGUI.py"])

#leg 
leg = Chain(name='leg', links=[
    OriginLink(),
    URDFLink(
      name="theta3",
      translation_vector=[0, 0, 0],
      orientation=[0, 0, 0],
      rotation=[1, 0, 0],
    ),
    URDFLink(
      name="theta1",
      translation_vector=[0, 0, -0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="theta2",
      translation_vector=[0, 0, -50],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    )
    ,
    URDFLink(
      name="foot",
      translation_vector=[0, 0, -50],
      orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    )
])


# Set up phy engine
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally "C:\Users\YungThunder\Documents\GitHub\RobotDog\Software"
planeId = p.loadURDF("plane.urdf")


p.setAdditionalSearchPath( "C:/Users/YungThunder/Documents/GitHub/RobotDog/Software" ) #optionally
p.setGravity(0,0,-10)
#
startPos = [0,0,0.10]
startOrientation = p.getQuaternionFromEuler([0,0,0])
dogId = p.loadURDF("spotSimple.urdf",startPos, startOrientation)
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
#set up cam
p.resetDebugVisualizerCamera(0.4,50,-35,startPos)



# set motors



val = leg.inverse_kinematics(lf_curretPosition)
#print(val)
p.setJointMotorControl2(dogId, 0, controlMode=p.POSITION_CONTROL, force=999, targetPosition=val[1])
p.setJointMotorControl2(dogId, 1, controlMode=p.POSITION_CONTROL, force=999, targetPosition=val[2])
p.setJointMotorControl2(dogId, 3, controlMode=p.POSITION_CONTROL, force=999, targetPosition=val[3])

val = leg.inverse_kinematics(rf_curretPosition)
#print(val)
p.setJointMotorControl2(dogId, 5, controlMode=p.POSITION_CONTROL, force=999, targetPosition=val[1])
p.setJointMotorControl2(dogId, 6, controlMode=p.POSITION_CONTROL, force=999, targetPosition=val[2])
p.setJointMotorControl2(dogId, 8, controlMode=p.POSITION_CONTROL, force=999, targetPosition=val[3])

val = leg.inverse_kinematics(rb_curretPosition)
#print(val)
p.setJointMotorControl2(dogId, 10, controlMode=p.POSITION_CONTROL, force=999, targetPosition=val[1])
p.setJointMotorControl2(dogId, 11, controlMode=p.POSITION_CONTROL, force=999, targetPosition=val[2])
p.setJointMotorControl2(dogId, 13, controlMode=p.POSITION_CONTROL, force=999, targetPosition=val[3])

val = leg.inverse_kinematics(lb_curretPosition)
#print(val)
p.setJointMotorControl2(dogId, 15, controlMode=p.POSITION_CONTROL, force=999, targetPosition=val[1])
p.setJointMotorControl2(dogId, 16, controlMode=p.POSITION_CONTROL, force=999, targetPosition=val[2])
p.setJointMotorControl2(dogId, 18, controlMode=p.POSITION_CONTROL, force=999, targetPosition=val[3])





#step phy engine



def update(dt):
    p.stepSimulation()


pyglet.clock.schedule_interval(update,1./240.)
pyglet.app.run()
#execfile('BalanceGUI.py')
