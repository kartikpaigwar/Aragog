"""This file implements the functionalities of a Aragog using pybullet.
"""

import os, inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)  # parent dir
# os.sys.path.insert(0, parentdir)

import collections
import copy
import math
import re
import numpy as np
import pybullet as p

# from quadruped.Aragog.envs import motor        #motor model


#::::::::::::::::::::Global Args:::::::::::::::::::::::#
INIT_POSITION = [0, 0, 0.5]
INIT_RACK_POSITION = [0, 0, 1]
init_motor_direction = [1]*14
init_motor_angles = [0, 0, -np.pi / 6, -np.pi / 2, 0, -np.pi / 6, -np.pi / 2, 0, 0, -np.pi / 6, -np.pi / 2, 0,
                     -np.pi / 6, -np.pi / 2]
urdf_root_path = os.path.join(parentdir + "/aragog_urdf")

class Aragog:

    def __init__(self, urdfRootPath=urdf_root_path, on_rack=False):

        self.urdfRootPath = urdfRootPath
        self.on_rack = on_rack
        self.num_motors = 14  # 8 Leg motors + 4 abduction motors + 2 FRONT and BACK module
        self.num_legs = 4
        self.leg_links_length = [0.15, 0.225]
        self.max_force = 10
        self.max_vel = 4
        self.kp = 2
        self.kd = 0.1

        self.motor_angles = init_motor_angles
        self.motor_direction = init_motor_direction
        self.buildfootIdList()


    def buildJointNameToIdDict(self):
        nJoints = p.getNumJoints(self.quadruped)
        self.jointNameToId = {}
        for i in range(nJoints):
            jointInfo = p.getJointInfo(self.quadruped, i)
            self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

    def buildJointNameToAngle(self):
        nJoints = p.getNumJoints(self.quadruped)
        self.jointNameToAngle = {}
        for i in range(nJoints):
            jointInfo = p.getJointInfo(self.quadruped, i)
            self.jointNameToAngle[jointInfo[1].decode('UTF-8')] = self.motor_direction[i] * self.motor_angles[i]

    def buildMotorIdList(self):
        self.motorIdList = []
        self.motorIdList.append(self.jointNameToId['FM_joint'])
        self.motorIdList.append(self.jointNameToId['FLA_joint'])
        self.motorIdList.append(self.jointNameToId['FLH_joint'])
        self.motorIdList.append(self.jointNameToId['FLK_joint'])
        self.motorIdList.append(self.jointNameToId['FRA_joint'])
        self.motorIdList.append(self.jointNameToId['FRH_joint'])
        self.motorIdList.append(self.jointNameToId['FRK_joint'])
        self.motorIdList.append(self.jointNameToId['BM_joint'])
        self.motorIdList.append(self.jointNameToId['BLA_joint'])
        self.motorIdList.append(self.jointNameToId['BLH_joint'])
        self.motorIdList.append(self.jointNameToId['BLK_joint'])
        self.motorIdList.append(self.jointNameToId['BRA_joint'])
        self.motorIdList.append(self.jointNameToId['BRH_joint'])
        self.motorIdList.append(self.jointNameToId['BRK_joint'])

    def buildfootIdList(self):
        self.footlinkIdList = [3, 6, 10, 13]


    def reset(self, orientation= "Forward", joint_angles= init_motor_angles):

        if self.on_rack:
            init_position = INIT_RACK_POSITION
        else:
            init_position = INIT_POSITION

        if orientation == "Forward":
            base_orientation = [0,0,1,0]
        elif orientation == "Reverse":
            base_orientation = [1,0,0,0]

        self.quadruped = p.loadURDF("%s/urdf/aragog.urdf" % self.urdfRootPath, init_position, baseOrientation=base_orientation, useFixedBase=self.on_rack)
        self.motor_angles = joint_angles
        self.buildJointNameToIdDict()
        self.buildJointNameToAngle()
        self.buildMotorIdList()
        self.resetPose()
        self.setCameraParam()
        for i in range(100):
            p.stepSimulation()

    def setMotorAngleById(self, motorId, desiredAngle):
        p.setJointMotorControl2(bodyIndex=self.quadruped,
                                jointIndex=motorId,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=desiredAngle,
                                positionGain=self.kp,
                                velocityGain=self.kd,
                                maxVelocity=self.max_vel,
                                force=self.max_force)

    def setAllMotorAngles(self, motoranglelist):
        p.setJointMotorControlArray(bodyIndex=self.quadruped,
                                    jointIndices=self.motorIdList,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=motoranglelist,
                                    positionGains=[self.kp]*self.num_motors,
                                    velocityGains=[self.kd]*self.num_motors,
                                    # maxVelocities=[self.max_vel]*self.num_motors,
                                    forces=[self.max_force]*self.num_motors)

    def setMotorAngleByName(self, motorName, desiredAngle):
        self.setMotorAngleById(self.jointNameToId[motorName], desiredAngle)

    def resetPose(self):
        for jointName in self.jointNameToAngle:
            p.resetJointState(self.quadruped, self.jointNameToId[jointName], self.jointNameToAngle[jointName],
                              targetVelocity=0)
            self.setMotorAngleByName(jointName, self.jointNameToAngle[jointName])

    def getBasePosition(self):
        position, orientation = p.getBasePositionAndOrientation(self.quadruped)
        return position

    def getBaseOrientation(self):
        position, orientation = p.getBasePositionAndOrientation(self.quadruped)
        orientation = p.getEulerFromQuaternion(orientation)
        return orientation

    def applyAction(self, motorCommands):
        motorCommandsWithDir = np.multiply(motorCommands, self.motor_direction)
        for i in range(self.num_motors):
            self.setMotorAngleById(self.motorIdList[i], motorCommandsWithDir[i])
        # self.setAllMotorAngles(motorCommandsWithDir)

    def getMotorAngles(self):
        motorAngles = []
        for i in range(self.num_motors):
            jointState = p.getJointState(self.quadruped, self.motorIdList[i])
            motorAngles.append(jointState[0])
        # motorAngles = np.multiply(motorAngles, self.motorDir)
        return motorAngles

    def getMotorVelocities(self):
        motorVelocities = []
        for i in range(self.num_motors):
            jointState = p.getJointState(self.quadruped, self.motorIdList[i])
            motorVelocities.append(jointState[1])
        # motorVelocities = np.multiply(motorVelocities, self.motorDir)
        return motorVelocities

    def getMotorTorques(self):
        motorTorques = []
        for i in range(self.num_motors):
            jointState = p.getJointState(self.quadruped, self.motorIdList[i])
            motorTorques.append(jointState[3])
        # motorTorques = np.multiply(motorTorques, self.motorDir)
        return motorTorques

    def SetFootFriction(self, foot_friction):
        """Set the lateral friction of the feet.
        Args: foot_friction: The lateral friction coefficient of the foot. This value is
        shared by all four feet.
        """
        for link_id in self.footlinkIdList:
            p.changeDynamics(self.quadruped, link_id, lateralFriction=foot_friction)


    """ Camera """

    def setCameraParam(self,fov=90, aspect=1.33, nearplane=0.01, farplane=100, width=640, height=480, camlink=0):
        """ Camera Parameters """

        self.projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
        self.imgWidth = width
        self.imgHeight = height
        self.cameralink = camlink     #front module

    def getCameraOutput(self):

        # Center of mass position and orientation (of link-7)
        com_p, com_o, local_p, _, _, _ = p.getLinkState(self.quadruped, 0)
        rot_matrix = p.getMatrixFromQuaternion(com_o)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # Initial vectors
        init_camera_vector = (-1, 0, 0)  # x-axis
        init_up_vector = (0, 0, 1)  # z-axis
        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)
        out = p.getCameraImage(self.imgWidth, self.imgHeight, view_matrix, self.projection_matrix,
                                                        renderer=p.ER_BULLET_HARDWARE_OPENGL,
                                                        flags=p.ER_NO_SEGMENTATION_MASK
                                                        )
        width = out[0]
        height = out[1]
        rgbImg = out[2]
        depthImg = out[3]

        return width,height,rgbImg,depthImg