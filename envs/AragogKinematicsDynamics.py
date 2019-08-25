import numpy as np
import time
import math
from math import cos, sin, atan2
from numpy.linalg import inv
import numpy.linalg as la

pi = math.pi

import pybullet as p
from envs.aragog import Aragog


# %% -------------------------------- Body Orientation ----------------------------
def Rotation_matrix(roll, yaw, pitch):
    # Roll, yaw, pitch follow these sequences
    # The axis of rotations are:  roll along X axis, yaw along Z axis, pitch
    # along Y axis. This convention may differ in the convention of axis under
    # consideration

    Rr = Rotx(roll)
    Ry = Rotz(yaw)
    Rp = Roty(pitch)
    R = np.dot(Rr, np.dot(Ry, Rp))
    return R

def Body_orientation(Gb, PHI):
    # Details about the reference
    # Gb: contains the zero condition address of the nodes in 3X4 matrix. The
    # address is written as follows: front-left, front-right, hind-left,
    # hind-right.
    # PHI: [roll; pitch ; yaw];

    rb = [0] * 4
    R = Rotation_matrix(PHI[0], PHI[2], PHI[1])
    for i in range(4):
        rb[i] = np.dot(R, Gb[i])

    return rb

# %% --------------------------------Jacobian----------------------------------

def Jacobian_calculation(q, Gl, n):
    J = np.zeros([3, 3])

    if n == 0 or n == 3:
        f = -1
    else:
        f = -1
    Ra = Rotx(f * q[0, n])
    dRa = dRotx(f * q[0, n])
    Rh = Rotz(f * q[1, n])
    dRh = dRotz(f * q[1, n])
    Rk = Rotz(f * q[2, n])
    dRk = dRotz(f * q[2, n])

    # Jacobian

    J0 = np.dot(dRa, Rh).dot(Gl[:, n]) + np.dot(dRa, Rk).dot(Gl[:, n + 4])
    J1 = np.dot(Ra, dRh).dot(Gl[:, n])
    J2 = np.dot(Ra, dRk).dot(Gl[:, n + 4])

    J[:, 0] = J0
    J[:, 1] = J1
    J[:, 2] = J2
    return J

# %% Rotation matrix about a single axis

def Rotx(x):  # Rotation along X axis
    R = np.array([[1, 0, 0], [0, cos(x), sin(x)], [0, -sin(x), cos(x)]])
    return R

def Roty(x):  # Rotation along Y axis
    R = np.array([[cos(x), 0, -sin(x)], [0, 1, 0], [sin(x), 0, cos(x)]])
    return R

def Rotz(x):  # Rotation along Z axis
    R = np.array([[cos(x), sin(x), 0], [-sin(x), cos(x), 0], [0, 0, 1]])
    return R

def dRotx(x):  # Partial derivative of the Rotation matrix, along X axis
    R = np.array([[0, 0, 0], [0, -sin(x), cos(x)], [0, -cos(x),
                                                    -sin(x)]])
    return R

def dRoty(x):  # Partial derivative of the Rotation matrix, along X axis
    R = np.array([[-sin(x), 0, -cos(x)], [0, 0, 0], [cos(x), 0,
                                                     -sin(x)]])
    return R

def dRotz(x):  # Partial derivative of the Rotation matrix, along X axis
    R = np.array([[-sin(x), cos(x), 0], [-cos(x), -sin(x), 0], [0, 0,
                                                                0]])
    return R

# Forward kinematics in 3d
def fk_3d(Joint_angles, l):
    R = [0] * 4
    # Forward kinematics
    # l = link lengths
    abd = [Joint_angles[1], Joint_angles[4], Joint_angles[8], Joint_angles[11]]
    hip = [Joint_angles[2], Joint_angles[5], Joint_angles[9], Joint_angles[12]]
    knee = [Joint_angles[3], Joint_angles[6], Joint_angles[10], Joint_angles[13]]

    x = [0] * 3
    for i in range(4):
        R_abd = Rotx(abd[i])
        R_hip = Roty(hip[i])
        R_knee = Roty(knee[i])

        x[0] = np.dot(R_abd, np.array([0, 0, 0]))
        x[1] = np.dot(R_abd, np.dot(R_hip, np.array([l[0], 0, 0])))
        x[2] = np.dot(R_abd, np.dot(R_hip, np.dot(R_knee, np.array([l[1], 0, 0]))))
        R[i] = x[0] + x[1] + x[2]

    return np.array(R)

# Inverse kinematics in 3d with choice of solution
def ik_3d(p1, l, Solution):
    # x, z on horizontal plane
    # y on along vertical
    # print "Inputs to ik_3d()"
    # print p0, p1, l, Soution
    p0 = [0]*3
    if Solution == 1:
        c_phi = 1
    elif Solution == 2:
        c_phi = -1

    r = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]

    # print("r = p1 - p0 = ", r) # <checked>
    beta = math.atan2(r[2], r[0])  # Angular rotation
    # print("Calculated beta=  math.atan2(r[2], abs(r[0])) = ", beta) #<checked>

    L = np.linalg.norm(r, 2)  # total length
    Z = (L ** 2 - l[0] ** 2 - l[1] ** 2) / (2 * l[0] * l[1])
    if Z > 1:
        Z = 1
    elif Z < -1:
        Z = -1

    alpha = math.atan2(r[1], abs(r[2]))  # Abduction angle
    # print("Abduction angle: alpha= math.atan2(r[2][0], abs(r[1][0])) = ", alpha*180/pi)

    phi = math.acos(Z)
    # print("Knee angle: phi = acos(Z) ", phi *180/pi)
    theta = beta - c_phi * math.asin(l[1] * sin(pi - phi) / L)
    # print("Hip angle: theta = beta + math.asin(l2 * sin(pi - phi) / l) =  ", theta* 180./pi)
    Q = np.array([-alpha, theta, c_phi * phi])
    # time.sleep(1000)
    return Q

#### ====================== IK: Spider Config =======================================
def ik_3d_spider(p1, l, Solution):
    # x, z on horizontal plane
    # y on along vertical
    #print "Inputs to ik_3d()"
    #print p0, p1, l, Soution
    p0 = [0]*3
    if Solution == 1:
        c_phi = 1
    elif Solution == 2:
        c_phi = -1

    r = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]
    # print("r = p1 - p0 = ", r) # <checked>

    beta = math.atan2(r[2], r[0])  # Rotation
    # print("Calculated beta=  math.atan2(r[2], abs(r[0])) = ", beta) #<checked>

    L = np.linalg.norm(r, 2)  # total length
    Z = (L ** 2 - l[0] ** 2 - l[1] ** 2) / (2 * l[0] * l[1])
    if Z > 1:
        Z = 1
    elif Z <-1:
        Z = -1

    alpha = math.atan2(r[1], abs(r[2]))  # Abduction angle
    # print("Abduction angle: alpha= math.atan2(r[2][0], abs(r[1][0])) = ", alpha*180/pi)

    phi = math.acos(Z)
    # print("Knee angle: phi = acos(Z) ", phi *180/pi)
    theta = beta - c_phi * math.asin(l[1] * sin(pi - phi) / L)
    # print("Hip angle: theta = beta + math.asin(l2 * sin(pi - phi) / l) =  ", theta* 180./pi)
    Q = np.array([-alpha, theta, c_phi *  phi])
    # time.sleep(1000)
    return Q


def RodriRot(P1, P2):
    p1 = P1 / la.norm(P1, 2)
    p2 = P2 / la.norm(P2, 2)
    phi = np.arccos(np.dot(p1, p2))
    v = np.cross(p1, p2)
    vl = la.norm(v, 2)
    if vl == 0:
        w = v  # / la.norm(v, 2)
    else:
        w = v / la.norm(v, 2)
    W = np.array([[0, w[2], -w[1]], [-w[2], 0, w[0]], [w[1], -w[0], 0]])
    R = np.eye(3) + np.sin(phi) * W + (1 - np.cos(phi)) * np.dot(W, W)

    return R

#### ======================= Filter =================================================

# Low pas filter
def LowPassFilter(x0, dt, ud, xd, alpha):
    alpha = 1.25
    u  = ud + alpha*(xd - x0)
    x = u * dt + x0
    return x


# %% --------------------------- Slope estimation ----------------------------------------

def slope_estimator(F, rl):
    alpha = 100
    F_th = 0.5
    C = np.zeros([4, 1])
    for i in range(0, 4):
        C[i] = 1 / (1 + np.exp(-0.00001 - alpha * (F[i] - F_th)))
    Cs = np.sum(C)

    if Cs == 0 or Cs == 1:
        roll_grd = 0
        pitch_grd = 0
    elif Cs == 2:
        j = 0
        r = np.zeros([3, 2])
        for i in range(0, 4):
            if C[i] > 0.25:
                r[:, j] = rl[:, i + 4]
                j = j + 1
        r1 = r[:, 1] - r[:, 0]
        r2 = np.array([0, 0, 1])

        P1 = np.cross(r1, r2)
        P2 = np.array([0, 1, 0])

        R = RodriRot(P1, P2)
        roll_grd = 0
        pitch_grd = np.arctan(R[0, 1] / R[0, 0])

    elif Cs == 3:
        j = 0
        r = np.zeros([3, 3])
        for i in range(0, 4):
            if C[i] > 0.25:
                r[:, j] = rl[:, i + 4]
                j = j + 1

        r1 = r[:, 0] - r[:, 2]
        r2 = r[:, 1] - r[:, 2]

        P1 = np.cross(r1, r2)
        P2 = np.array([0, 1, 0])
        R = np.array(RodriRot(P1, P2))
        roll_grd = np.arctan(-R[1, 2] / R[2, 2])
        pitch_grd = np.arctan(-R[0, 1] / R[0, 0])
    else:
        roll_grd = 0
        pitch_grd = 0

    return [roll_grd, -pitch_grd]
    # the negetive value indicates actual ground ground elevation from the base of the slope
