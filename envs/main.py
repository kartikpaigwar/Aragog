import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
jointNameToId = {}
kp = 1
kd = 0.1
maxForce = 3.5
nMotors = 8
motorIdList = []
motorDir = [-1, -1, -1, -1, 1, 1, 1, 1]


def buildJointNameToIdDict(quadruped):
    nJoints = p.getNumJoints(quadruped)

    for i in range(nJoints):
        jointInfo = p.getJointInfo(quadruped, i)
        jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

def setMotorAngleByName( motorName, desiredAngle):
    setMotorAngleById(jointNameToId[motorName], desiredAngle)

def setMotorAngleById( motorId, desiredAngle):
    p.setJointMotorControl2(bodyIndex=quadruped,
                            jointIndex=motorId,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=desiredAngle,
                            positionGain=kp,
                            velocityGain=kd,
                            force=maxForce)
def resetPose(quadruped):
    halfpi = 1.57079632679
    kneeangle = 0.75  #halfpi - acos(upper_leg_length / lower_leg_length)

    #left front leg
    p.resetJointState(quadruped, jointNameToId['FLA_joint'], 0)
    p.resetJointState(quadruped, jointNameToId['FLH_joint'], halfpi)
    p.resetJointState(quadruped, jointNameToId['FLK_joint'], kneeangle)

    #
    setMotorAngleByName('FLA_joint',  0)
    setMotorAngleByName('FLH_joint', halfpi)
    setMotorAngleByName('FLK_joint', kneeangle)
    # p.setJointMotorControl2(bodyIndex=quadruped,
    #                         jointIndex=jointNameToId['knee_front_leftL_link'],
    #                         controlMode=p.VELOCITY_CONTROL,
    #                         targetVelocity=0,
    #                         force=kneeFrictionForce)
    # p.setJointMotorControl2(bodyIndex=quadruped,
    #                         jointIndex=jointNameToId['knee_front_leftR_link'],
    #                         controlMode=p.VELOCITY_CONTROL,
    #                         targetVelocity=0,
    #                         force=kneeFrictionForce)

    

INIT_POSITION = [0, 0, .2]
INIT_RACK_POSITION = [0, 0, 1]
urdfRootPath = "../aragog_urdf"
quadruped = p.loadURDF("/home/kartik/quadruped/Aragog/aragog_urdf/urdf/aragog.urdf", INIT_RACK_POSITION,useFixedBase=True)
buildJointNameToIdDict(quadruped)
print(jointNameToId)
resetPose(quadruped)
jointinfo = p.getJointState(quadruped, 3)
print(jointinfo[0])
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
quadPos, quadOrn = p.getBasePositionAndOrientation(quadruped)
print(quadPos,quadOrn)
p.disconnect()
