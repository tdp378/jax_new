import numpy as np
from numpy.linalg import inv, norm
from numpy import asarray, matrix
from math import *
# Internal Jax imports
from jax_control.util import RotMatrix3D, point_to_rad
from transforms3d.euler import euler2mat

def leg_explicit_inverse_kinematics(r_local_foot, leg_index, config):
    # Determine leg side (1 and 3 are Right)
    is_right = (leg_index == 1 or leg_index == 3)
    
    # We use local coordinates (already relative to shoulder)
    x, y, z = r_local_foot[0], r_local_foot[1], r_local_foot[2]
    
    # Mirroring: Only flip Y for the Right side to match the single-leg IK model
    if is_right:
        y = -y

    # Calculate distance in the Y-Z plane (the "hip-to-foot" reach)
    R = sqrt(y**2 + z**2)
    
    # Math Safety: Prevent square root of negative numbers (The crash culprit)
    if R < config.L1:
        return np.array([0.0, 0.6, 1.2]) 

    # --- IK Math ---
    # Theta 1: Hip Abduction
    # max(0, ...) ensures we never take the sqrt of a negative number
    theta_1 = atan2(y, -z)
    # D: Distance from the hip joint to the foot
    D_sq = R**2 - config.L1**2 + x**2
    D = sqrt(max(0, D_sq))
    
    # Theta 3: Knee
    cos_theta_3 = (config.L2**2 + config.L3**2 - D**2) / (2 * config.L2 * config.L3)
    theta_3 = acos(np.clip(cos_theta_3, -1, 1))

    # Theta 2: Thigh
    alpha = atan2(x, sqrt(max(0, R**2 - config.L1**2)))
    cos_beta = (config.L2**2 + D**2 - config.L3**2) / (2 * config.L2 * D)
    theta_2 = alpha + acos(np.clip(cos_beta, -1, 1))

    return np.array([theta_1, theta_2 - 1.57, theta_3 - .785])

def leg_forward_kinematics(angles, config, is_right):
    """Calculates foot position from joint angles for Jax."""
    x = config.L3*sin(angles[1]+angles[2]) - config.L2*cos(angles[1])
    
    # Y and Z depend on the hip offset and leg side
    y = 0.5*config.L2*cos(angles[0]+angles[1]) - config.L1*cos(angles[0]+(403*pi)/4500) - 0.5*config.L2*cos(angles[0]-angles[1]) - config.L3*cos(angles[1]+angles[2])*sin(angles[0])
    z = 0.5*config.L2*sin(angles[0]-angles[1]) + config.L1*sin(angles[0]+(403*pi)/4500) - 0.5*config.L2*sin(angles[0]+angles[1]) - config.L3*cos(angles[1]+angles[2])*cos(angles[0])
    
    if not is_right:
        y = -y
    return np.array([x,y,z])

def angle_corrector(angles=[0,0,0]):
    """Adjusts angles for hardware/simulation offsets if necessary."""
    return angles