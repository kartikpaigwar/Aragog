import pybullet as p
import time
import pybullet_data

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))  # parent of parent dir

os.sys.path.insert(0, parentdir)

print(parentdir)

from Aragog.envs import aragog

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")


urdfRootPath = "/home/kartik/quadruped/Aragog/aragog_urdf"
quad = aragog.Aragog(urdfRootPath)
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
quadPos, quadOrn = p.getBasePositionAndOrientation(quad)
print(quadPos,quadOrn)
p.disconnect()


