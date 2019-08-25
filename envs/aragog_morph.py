import numpy as np
import pybullet as p

from envs.aragog import Aragog
import envs.AragogKinematicsDynamics as akd
def from_minus_pi_to_pi(joint_angle):
    if joint_angle > np.pi:
        joint_angle = -np.pi + (joint_angle - np.pi)
    elif joint_angle < -np.pi:
        joint_angle = np.pi + (joint_angle + np.pi)
    return joint_angle

class Aragog_morph(Aragog):
    def __init__(self, initial_config="Dog_Normal", initial_state = "Forward"):
        super().__init__()
        self.robot_morph = initial_config
        self.robot_state = initial_state
        self.Keys, self.bodymorph_Keys = self.CreateKeyList()
        self.hip2foot_length = 0.25
        self.dog_configs = ['Dog_Normal', 'Dog_X', 'Dog_M', 'Dog_O']
        self.spider_configs = ['Spider_low', 'Spider_high', 'Tree_walker']
        self.reset_morphology()

    def CreateKeyList(self):
        key_list = [0] * 25
        # - Forward and reverse speed control -
        key_list[0] = ord('w')  # speed up / forward
        key_list[1] = ord('s')  # slow down / reverse

        # - Turning control -
        key_list[2] = ord('a')  # Turn left
        key_list[3] = ord('d')  # Turn right

        # - Sidewise stepping -
        key_list[4] = ord('q')  # Move left
        key_list[5] = ord('e')  # Move right

        # - Body morphing -
        key_list[6] = ord('u')  # Dog Normal pose
        key_list[7] = ord('i')  # Dog X pose
        key_list[8] = ord('o')  # Dog O pose
        key_list[9] = ord('p')  # Dog M pose
        key_list[10] = ord('j')  # Spider low pose
        key_list[11] = ord('k')  # Spider high pose
        key_list[12] = ord('l')  # Tree_walker

        # - Body Roll -
        key_list[13] = ord('m')  # Body roll morphology

        # - Gait transition -
        key_list[14] = ord('x')  # Trot
        key_list[15] = ord('z')  # Walk
        key_list[16] = ord('c')  # Bound
        key_list[17] = ord('v')  # Bound with spine

        # - BRAKE -
        key_list[18] = ord('b')  # Brake the robot:

        # - Manual reverse -
        key_list[19] = ord('r')  # Body configuration reverse

        # This brings it back to its stand posture from any movement
        
        #Seperate out body_morphologies keys from all keys
        body_morph_key_list = key_list[6:14]
        body_morph_key_list.append(key_list[19])
        
        return key_list, body_morph_key_list


    def Body_configurarion(self, inputkeys):

        if self.bodymorph_Keys[0] in inputkeys and inputkeys[self.bodymorph_Keys[0]] & p.KEY_WAS_TRIGGERED:
            self.robot_morph = 'Dog_Normal'
        elif self.bodymorph_Keys[1] in inputkeys and inputkeys[self.bodymorph_Keys[1]] & p.KEY_WAS_TRIGGERED:
            self.robot_morph = 'Dog_X'
        elif self.bodymorph_Keys[2] in inputkeys and inputkeys[self.bodymorph_Keys[2]] & p.KEY_WAS_TRIGGERED:
            self.robot_morph = 'Dog_O'
        elif self.bodymorph_Keys[3] in inputkeys and inputkeys[self.bodymorph_Keys[3]] & p.KEY_WAS_TRIGGERED:
            self.robot_morph = 'Dog_M'
        elif self.bodymorph_Keys[4] in inputkeys and inputkeys[self.bodymorph_Keys[4]] & p.KEY_WAS_TRIGGERED:
            self.robot_morph = 'Spider_low'
        elif self.bodymorph_Keys[5] in inputkeys and inputkeys[self.bodymorph_Keys[5]] & p.KEY_WAS_TRIGGERED:
            self.robot_morph = 'Spider_high'
        elif self.bodymorph_Keys[6] in inputkeys and inputkeys[self.bodymorph_Keys[6]] & p.KEY_WAS_TRIGGERED:
            self.robot_morph = 'Tree_walker'
        elif self.bodymorph_Keys[7] in inputkeys and inputkeys[self.bodymorph_Keys[7]] & p.KEY_WAS_TRIGGERED:
            self.robot_morph = 'Roll'

        return self.robot_morph

    def Body_Correction(self, inputkeys):

        if self.bodymorph_Keys[8] in inputkeys and inputkeys[self.bodymorph_Keys[8]] & p.KEY_WAS_TRIGGERED and self.robot_state == 'Forward':
            self.robot_state = 'Reverse'
        elif self.bodymorph_Keys[8] in inputkeys and inputkeys[self.bodymorph_Keys[8]] & p.KEY_WAS_TRIGGERED and self.robot_state == 'Reverse':
            self.robot_state = 'Forward'

        return self.robot_state

    # ============================== Body configuration ======================================
    def Body_config_solution(self, x = "Dog_Normal", body_state="Forward"):
        if x == 'Dog_Normal':
            fac_solution = [1, 1, 1, 1, 0, 0]
            abd = [0, 0, 0, 0]
        elif x == 'Dog_X':
            fac_solution = [2, 2, 1, 1, 0, 0]
            abd = [0, 0, 0, 0]
        elif x == 'Dog_M':
            fac_solution = [1, 2, 1, 2, 0, 0]
            abd = [0, 0, 0, 0]
        elif x == 'Dog_O':
            fac_solution = [1, 1, 2, 2, 0, 0]
            abd = [0, 0, 0, 0]
        elif x == 'Spider_low':
            fac_solution = [1, 1, 2, 2, 1.57, -1.57]
            abd = [0.15, -0.15, 0.15, -0.15]
        elif x == 'Spider_high':
            fac_solution = [1, 1, 2, 2, 1.57, -1.57]
            abd = [0, 0, 0, 0]
        elif x == 'Tree_walker':
            fac_solution = [1, 1, 2, 2, 1.57, -1.57]
            abd = [0.1, -0.1, 0.1, -0.1]
        else:
            print("Sorry that configuration doesn't exist")

        if body_state == "Reverse":
            for i in range(4):
                if fac_solution[i] == 1:
                    fac_solution[i] = 2
                elif fac_solution[i] == 2:
                    fac_solution[i] = 1

        return fac_solution, abd

    # -------------------- Body configuration: Fall correction ----------------------------
    def getZ_off(self, body_state='Forward'):
        if body_state == 'Reverse':
            z_off = self.hip2foot_length
        elif body_state == 'Forward':
            z_off = -1* self.hip2foot_length
        return z_off

    def CalculateTargetJointAngles(self,config_solution,abd_off,foot_pos):
        joint_angles = [0] * 14
        leg_joint_angles = [0] * 12
        abd_hip_knee_angles = [0] * 4
        for i in range(self.num_legs):
            if i < 2:
                Spine_off = config_solution[4]
                foot_xoff = -0.1
            else:
                Spine_off = config_solution[5]
                foot_xoff = 0.1

            if self.robot_morph in self.dog_configs:
                abd_hip_knee_angles[i] = akd.ik_3d(foot_pos[i], self.leg_links_length, config_solution[i])
            elif self.robot_morph in self.spider_configs:
                foot_pos[i][0] += foot_xoff
                foot_pos[i][1] = abd_off[i]
                abd_hip_knee_angles[i] = akd.ik_3d_spider(foot_pos[i], self.leg_links_length, config_solution[i])

            leg_joint_angles[3 * i] = from_minus_pi_to_pi(abd_hip_knee_angles[i][0])
            leg_joint_angles[3 * i + 1] = from_minus_pi_to_pi(abd_hip_knee_angles[i][1] + Spine_off)
            leg_joint_angles[3 * i + 2] = from_minus_pi_to_pi(abd_hip_knee_angles[i][2])

        joint_angles[0] = config_solution[4]
        joint_angles[1:7] = leg_joint_angles[0:6]
        joint_angles[7] = config_solution[5]
        joint_angles[8:14] = leg_joint_angles[6:12]

        return joint_angles

    def reset_morphology(self):
        robot_morph = self.robot_morph
        robot_state = self.robot_state
        config_solution, abd_off = self.Body_config_solution(robot_morph,robot_state)
        z_off = self.getZ_off(robot_state)
        foot_pos = np.array([ [0, 0, z_off], [0, 0, z_off], [0, 0, z_off], [0, 0, z_off] ])
        joint_angles = self.CalculateTargetJointAngles(config_solution,abd_off,foot_pos)
        self.reset(robot_state,joint_angles)




    
        


