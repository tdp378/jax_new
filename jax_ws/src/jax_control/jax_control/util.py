from math import atan2, pi, radians, cos, sin
import numpy as np

def point_to_rad(p1, p2): 
    """Converts 2D cartesian points to polar angles in range 0 - 2pi."""
    # Simplified logic: atan2 handles the quadrants automatically
    theta = atan2(p2, p1)
    theta = (theta + 2*pi) % (2*pi)
    return theta
    
def RotMatrix3D(rotation=[0,0,0], is_radians=True, order='xyz'):
    """Generates a 3D rotation matrix for Jax based on roll, pitch, and yaw."""
    
    roll, pitch, yaw = rotation[0], rotation[1], rotation[2]

    # Convert to radians if the input is in degrees
    if not is_radians: 
        roll = radians(roll)
        pitch = radians(pitch)
        yaw = radians(yaw)
    
    # Rotation matrix about each axis
    rotX = np.matrix([[1, 0, 0], [0, cos(roll), -sin(roll)], [0, sin(roll), cos(roll)]])
    rotY = np.matrix([[cos(pitch), 0, sin(pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]])
    rotZ = np.matrix([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    
    # Rotation matrix order (Default for Jax: Z * Y * X)
    if order == 'xyz': rotationMatrix = rotZ * rotY * rotX
    elif order == 'xzy': rotationMatrix = rotY * rotZ * rotX
    elif order == 'yxz': rotationMatrix = rotZ * rotX * rotY
    elif order == 'yzx': rotationMatrix = rotX * rotZ * rotY
    elif order == 'zxy': rotationMatrix = rotY * rotX * rotZ
    elif order == 'zyx': rotationMatrix = rotX * rotY * rotZ
    
    return rotationMatrix