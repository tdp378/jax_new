import numpy as np
from numpy.linalg import inv, norm
from numpy import asarray, matrix
from math import *
# Internal Jax imports
from jax_control.util import RotMatrix3D, point_to_rad
from transforms3d.euler import euler2mat

def leg_explicit_inverse_kinematics(r_body_foot, leg_index, config):
    """Find the joint angles corresponding to the given body-relative foot position 
    for a given leg and configuration for Jax.
    
    Parameters
    ----------
    r_body_foot : numpy array (3)
        The x,y,z co-ordinates of the foot relative to the first leg frame
    leg_index : int
        0 = Front left, 1 = Front right, 2 = Rear left, 3 = Rear right
    config : Configuration class
        Configuration class containing Jax link lengths and parameters
    
    Returns
    -------
    angles : numpy array (3)
        Array of calculated joint angles (theta_1, theta_2, theta_3)
    """

    # Determine if leg is a right or a left leg
    if leg_index == 1 or leg_index == 3:
        is_right = 0
    else:
        is_right = 1
    
    # Flip the y axis if the foot is a right foot to make calculation correct
    x,y,z = r_body_foot[0], r_body_foot[1], r_body_foot[2]
    if not is_right:
        y = -y

    # Distance from the hip to the foot projected onto the Y-Z plane
    R = sqrt(y**2 + z**2)
    
    # Distance from the hip joint to the foot
    D = sqrt(R**2 - config.L1**2 + x**2)

    # Calculate theta_1 (Hip Roll)
    theta_1 = atan2(y, -z) - atan2(config.L1, sqrt(R**2 - config.L1**2))
    
    # Calculate theta_3 (Knee)
    # Using law of cosines
    cos_theta_3 = (config.L2**2 + config.L3**2 - D**2) / (2 * config.L2 * config.L3)
    cos_theta_3 = np.clip(cos_theta_3, -1, 1) # Prevent NaN from float errors
    theta_3 = acos(cos_theta_3)

    # Calculate theta_2 (Hip Pitch)
    alpha = atan2(x, sqrt(R**2 - config.L1**2))
    cos_beta = (config.L2**2 + D**2 - config.L3**2) / (2 * config.L2 * D)
    cos_beta = np.clip(cos_beta, -1, 1)
    beta = acos(cos_beta)
    theta_2 = alpha + beta

    return np.array([theta_1, theta_2, theta_3])

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