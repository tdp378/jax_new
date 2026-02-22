import numpy as np
from numpy.linalg import norm
from math import pi, sin, cos, asin, acos, atan2

def point_to_rad(y, z):
    """Calculates the angle from the positive y-axis to the point (y, z)"""
    return atan2(z, y)

def RotMatrix3D(angles, is_radians=True):
    """Generates a rotation matrix around the X-axis for hip calculations"""
    if not is_radians:
        angles = np.radians(angles)
    # Rotation about X-axis as used in your IK logic
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(angles[0]), -np.sin(angles[0])],
                    [0, np.sin(angles[0]), np.cos(angles[0])]])
    return R_x

def angle_corrector(angles=[0,0,0]):
    # 1. Hip Roll (theta_1) - No change
    angles[0] = angles[0]

    # 2. Thigh (theta_2)
    # Changed from (+ 0.6) to (- 0.8) to pull the legs toward the REAR.
    # If they are STILL too far forward, try -1.0. 
    # If they go too far back, try -0.4.
    angles[1] = -(angles[1] - pi - 1.5) 

    # 3. Calf/Knee (theta_3)
    # Keeping this negated to ensure the knee bends "inward" toward the body.
    angles[2] = -(angles[2] - 0.5)

    # Normalize...
    for index, theta in enumerate(angles):
        while angles[index] > pi: angles[index] -= 2 * pi
        while angles[index] < -pi: angles[index] += 2 * pi
        
    return angles

def leg_explicit_inverse_kinematics(r_body_foot, leg_index, config):
    """Find the joint angles for a single leg"""
    # 0=FR, 1=FL, 2=RR, 3=RL -> 0 & 2 are Right legs
    if leg_index == 0 or leg_index == 2:
        is_right = 1
    else:
        is_right = 0
    
    # Flip the y axis if the foot is a right foot to make calculation consistent
    x, y, z = r_body_foot[0], r_body_foot[1], r_body_foot[2]
    if is_right: 
        y = -y
    
    # Rotate the origin frame to be in-line with config.L1
    R1 = pi/2 - config.phi 
    rot_mtx = RotMatrix3D([-R1, 0, 0], is_radians=True)
    r_body_foot_ = rot_mtx @ np.array([x, y, z])
    
    x_rot, y_rot, z_rot = r_body_foot_[0], r_body_foot_[1], r_body_foot_[2]
    len_A = max(norm([0, y_rot, z_rot]), 0.0001)   
    
    a_1 = point_to_rad(y_rot, z_rot)                     
    a_2 = asin(sin(config.phi) * config.L1 / len_A)
    a_3 = pi - a_2 - config.phi               

    theta_1 = a_1 + a_3
    
    # Translate frame to the hip joint
    offset = np.array([0.0, config.L1 * cos(theta_1), config.L1 * sin(theta_1)])
    translated_frame = r_body_foot_ - offset
    
    # Rotation correction for theta_2 and theta_3 plane
    R2 = theta_1 + config.phi - pi/2
    rot_mtx_2 = RotMatrix3D([-R2, 0, 0], is_radians=True)
    j4_2_vec_ = rot_mtx_2 @ translated_frame
    
    x_final, z_final = j4_2_vec_[0], j4_2_vec_[2]
    len_B = norm([x_final, 0, z_final])
    
    # Reach limit check
    if len_B >= (config.L2 + config.L3): 
        len_B = (config.L2 + config.L3) * 0.98
    
    b_1 = point_to_rad(x_final, z_final)  
    b_2 = acos((config.L2**2 + len_B**2 - config.L3**2) / (2 * config.L2 * len_B)) 
    b_3 = acos((config.L2**2 + config.L3**2 - len_B**2) / (2 * config.L2 * config.L3))  
    
    theta_2 = b_1 - b_2
    theta_3 = pi - b_3

    # Apply offsets and inversions
    return np.array(angle_corrector(angles=[theta_1, theta_2, theta_3]))

def four_legs_inverse_kinematics(r_body_foot, config):
    """Calculates joint angles for all 4 legs in [FR, FL, RR, RL] order"""
    alpha = np.zeros((3, 4))
    for i in range(4):
        body_offset = config.LEG_ORIGINS[:, i]
        alpha[:, i] = leg_explicit_inverse_kinematics(
            r_body_foot[:, i] - body_offset, i, config
        )
    return alpha

def forward_kinematics(angles, config, is_right=0):
    """Find the foot position relative to the hip for verification"""
    x = config.L3*sin(angles[1]+angles[2]) - config.L2*cos(angles[1])
    y = (0.5*config.L2*cos(angles[0]+angles[1]) - 
         config.L1*cos(angles[0]+(403*pi)/4500) - 
         0.5*config.L2*cos(angles[0]-angles[1]) - 
         config.L3*cos(angles[1]+angles[2])*sin(angles[0]))
    z = (0.5*config.L2*sin(angles[0]-angles[1]) + 
         config.L1*sin(angles[0]+(403*pi)/4500) - 
         0.5*config.L2*sin(angles[0]+angles[1]) - 
         config.L3*cos(angles[1]+angles[2])*cos(angles[0]))
    if not is_right:
        y = -y
    return np.array([x, y, z])