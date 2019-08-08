"""This file implements the functionalities of a Aragog using pybullet.
"""

import os, inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))  # parent of parent dir
os.sys.path.insert(0, parentdir)

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


class Aragog:

    def __init__(self, urdfRootPath='', on_rack=False):
        self.urdfRootPath = urdfRootPath
        self.on_rack = on_rack
        self.num_motors = 14  # 8 Leg motors + 4 abduction motors + 2 FRONT and BACK module
        self.num_legs = 4
        self.max_force = 10
        self.max_vel = 4
        self.kp = 2
        self.kd = 0.1

        self.motor_direction = [1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1]
        self.motor_angles = [0, 0, np.pi/1.2, np.pi/2, 0, np.pi/1.2, np.pi/2, 0, 0, np.pi/1.2, np.pi/2, 0, np.pi/1.2, np.pi/2]
        # self.motor_angles = [0, 0, np.pi/2, 0, 0, np.pi/2, 0, 0, 0, np.pi/2, 0, 0, np.pi/2, 0]

        self.motorIdList = []
        self.reset()

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


    def reset(self):
        if self.on_rack:
            init_position = INIT_RACK_POSITION
        else:
            init_position = INIT_POSITION

        self.quadruped = p.loadURDF("%s/urdf/aragog.urdf" % self.urdfRootPath, init_position, useFixedBase=self.on_rack)
        self.buildJointNameToIdDict()
        self.buildJointNameToAngle()
        self.buildMotorIdList()
        self.resetPose()
        for i in range(100):
            p.stepSimulation()



    def setMotorAngleById(self, motorId, desiredAngle):
        p.setJointMotorControl2(bodyIndex=self.quadruped,
                                jointIndex=motorId,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=desiredAngle,
                                positionGain=self.kp,
                                velocityGain=self.kd,
                                maxVelocity = self.max_vel,
                                force=self.max_force)

    def setMotorAngleByName(self, motorName, desiredAngle):
        self.setMotorAngleById(self.jointNameToId[motorName], desiredAngle)

    def resetPose(self):
        for jointName in self.jointNameToAngle:
            p.resetJointState(self.quadruped, self.jointNameToId[jointName],self.jointNameToAngle[jointName], targetVelocity=0)
            self.setMotorAngleByName(jointName, self.jointNameToAngle[jointName])

    #
    #
    # def getBasePosition(self):
    #     position, orientation = p.getBasePositionAndOrientation(self.quadruped)
    #     return position
    #
    # def getBaseOrientation(self):
    #     position, orientation = p.getBasePositionAndOrientation(self.quadruped)
    #     return orientation
    #
    # def applyAction(self, motorCommands):
    #     motorCommandsWithDir = np.multiply(motorCommands, self.motorDir)
    #     for i in range(self.nMotors):
    #         self.setMotorAngleById(self.motorIdList[i], motorCommandsWithDir[i])
    #
    # def getMotorAngles(self):
    #     motorAngles = []
    #     for i in range(self.nMotors):
    #         jointState = p.getJointState(self.quadruped, self.motorIdList[i])
    #         motorAngles.append(jointState[0])
    #     motorAngles = np.multiply(motorAngles, self.motorDir)
    #     return motorAngles
    #
    # def getMotorVelocities(self):
    #     motorVelocities = []
    #     for i in range(self.nMotors):
    #         jointState = p.getJointState(self.quadruped, self.motorIdList[i])
    #         motorVelocities.append(jointState[1])
    #     motorVelocities = np.multiply(motorVelocities, self.motorDir)
    #     return motorVelocities
    #
    # def getMotorTorques(self):
    #     motorTorques = []
    #     for i in range(self.nMotors):
    #         jointState = p.getJointState(self.quadruped, self.motorIdList[i])
    #         motorTorques.append(jointState[3])
    #     motorTorques = np.multiply(motorTorques, self.motorDir)
    #     return motorTorques
