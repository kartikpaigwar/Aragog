import pybullet as p
import time
import pybullet_data
import os
from envs.aragog_morph import Aragog_morph

dirpath = os.getcwd()

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

# urdf_root_path = os.path.join(dirpath + "/aragog_urdf")
#
quad = Aragog_morph('Spider_high')
new_motorangles = quad.motor_angles
for i in range(1000000):
    p.stepSimulation()
    # new_motorangles = [x+1 for x in new_motorangles]
    # quad.applyAction(new_motorangles)
    time.sleep(3. / 240.)
    # w,h,rgbimg,depthimg = quad.getCameraOutput()
    # print(depthimg)

# quadPos, quadOrn = p.getBasePositionAndOrientation(quad.quadruped)
# euang = p.getEulerFromQuaternion(quadOrn)
# print(euang)
p.disconnect()