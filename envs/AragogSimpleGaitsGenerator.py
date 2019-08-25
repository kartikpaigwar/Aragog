import numpy as np
import pybullet as p
import math
from math import cos, sin, atan2

from envs.aragog_morph import Aragog_morph
import envs.AragogKinematicsDynamics as akd

class Aragog_Gait_Generator(Aragog_morph):
    def __init__(self, timestep = 1. / 240.):
        super().__init__()
        self.omega = 0
        self.omega_t = 0
        self.dt = timestep
        self.Phi = [0] * 4  # Phase difference for trot gait
        self.foot_pos = [0] * 4  # Initiating variables
        self.amp = [0.025, 0.025]  # Amplitude of the elipes
        self.theta = np.pi/2  # Phase of the internal clock variable
        self.theta0 = np.pi/2
        self.alpha = 1.5  # Hyper parameter
        self.joint_angles = self.getMotorAngles()
        self.R = [0] * 4
        self.dR = [0] * 4
        self.SetFootFriction(0.2)



    def run_cycles(self, inputkeys):
        self.Key_board_control(inputkeys)
        self.updatefrequency()
        self.updatePhaseDiff()
        robot_morph = self.Body_configurarion(inputkeys)
        robot_state = self.Body_Correction(inputkeys)
        config_solution, abd_off = self.Body_config_solution(robot_morph, robot_state)
        z_off = self.getZ_off(robot_state)
        foot_pos = self.updateFootPos(z_off)
        # print(foot_pos[0])
        target_joint_angles = self.CalculateTargetJointAngles(config_solution,abd_off,foot_pos)

        joint_angles_array = np.array(self.joint_angles)
        target_joint_angles_array = np.array(target_joint_angles)

        self.joint_angles = joint_angles_array + 0.0025 * (target_joint_angles_array - joint_angles_array)
        self.applyAction(self.joint_angles)

    def Key_board_control(self, inputkeys):
        if self.Keys[0] in inputkeys and inputkeys[self.Keys[0]] & p.KEY_WAS_TRIGGERED:
            self.omega_t = self.omega_t + 0.1
        elif self.Keys[1] in inputkeys and inputkeys[self.Keys[1]] & p.KEY_WAS_TRIGGERED:
            self.omega_t = self.omega_t - 0.1

    def updatefrequency(self):
        zai = 0.25 #constant
        self.omega = self.omega + zai * (self.omega_t - self.omega) * self.dt

    def updatePhaseDiff(self):
        if np.abs(self.omega) < 2.5e-1:
            Phi_t = np.array([0] * 4)
        else:
            Phi_t = np.array([0, np.pi, np.pi, 0])

        self.Phi = self.Phi + 0.125 * (Phi_t - self.Phi) * self.dt

    def updateTheta(self):
        if np.abs(self.omega) >= 2.5e-1:
            self.theta = self.omega * self.dt + self.theta0
            self.theta0 = self.theta

    def updateFootPos(self, z_off):
        self.updateTheta()
        for i_cpg in range(self.num_legs):
            Theta = (self.theta + self.Phi[i_cpg]) % (2 * np.pi)
            # print(Theta*180/np.pi)
            Theta = -1*Theta

            self.R[i_cpg] = [self.amp[0] * cos(Theta), 0, z_off + self.amp[1] * sin(Theta)]

            self.dR[i_cpg] = [-1*self.amp[0] * sin(Theta), 0, self.amp[1] * cos(Theta)]

        # ----------------- Final augmentation ----------------
        joint_angles = self.getMotorAngles()
        Rac = akd.fk_3d(joint_angles,self.leg_links_length)

        for i in range(self.num_legs):
            self.dR[i] = self.omega * np.array(self.dR[i]) + self.alpha * (self.R[i] - Rac[i])
            self.R[i] = self.dR[i] * self.dt + self.R[i]

        return self.R



