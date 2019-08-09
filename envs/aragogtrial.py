import pybullet as p
import time
import pybullet_data

import os, inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))  # parent of parent dir

os.sys.path.insert(0, parentdir)

print(parentdir)

from Aragog.envs import aragog

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

urdfRootPath = "/home/kartik/RBC/quadruped/Aragog/aragog_urdf"
quad = aragog.Aragog(urdfRootPath)
new_motorangles = quad.motor_angles
for i in range(10000):
    p.stepSimulation()
    # new_motorangles = [x+1 for x in new_motorangles]
    # quad.applyAction(new_motorangles)
    # time.sleep(3. / 240.)
    w,h,rgbimg,depthimg = quad.getCameraOutput()
    print(depthimg)

quadPos, quadOrn = p.getBasePositionAndOrientation(quad.quadruped)
euang = p.getEulerFromQuaternion(quadOrn)
print(euang)
p.disconnect()
