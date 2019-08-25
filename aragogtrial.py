import pybullet as p
import time
import pybullet_data
import os
from envs.AragogSimpleGaitsGenerator import Aragog_Gait_Generator

dirpath = os.getcwd()

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

# urdf_root_path = os.path.join(dirpath + "/aragog_urdf")
#
quad = Aragog_Gait_Generator()
new_motorangles = quad.motor_angles
omega = 0  # operating frequency
omega_t = 0  # Initiating target frequency
for i in range(1000000):
    keyspressed = p.getKeyboardEvents()
    quad.run_cycles(keyspressed)
    print(quad.omega)
    p.stepSimulation()
    # new_motorangles = [x+1 for x in new_motorangles]
    # quad.applyAction(new_motorangles)
    # time.sleep(1. / 240.)
    # w,h,rgbimg,depthimg = quad.getCameraOutput()
    # print(depthimg)

# quadPos, quadOrn = p.getBasePositionAndOrientation(quad.quadruped)
# euang = p.getEulerFromQuaternion(quadOrn)
# print(euang)
p.disconnect()
