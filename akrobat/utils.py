import numpy as np
from geometry_msgs.msg import Point

def rotate_point_around_point(point_x: Point, rotation_point: Point, radians: float) -> Point:

    # Translate point_x to the origin relative to point_o
    translated_x = point_x.x - rotation_point.x
    translated_y = point_x.y - rotation_point.y

    # Create the rotation matrix
    rotation_matrix = np.array([
        [np.cos(radians), -np.sin(radians)],
        [np.sin(radians), np.cos(radians)]
    ])

    # Create the point as a numpy array
    point = np.array([translated_x, translated_y])

    # Perform the rotation
    rotated_point = np.dot(rotation_matrix, point)

    # Translate the point back
    final_x = rotated_point[0] + rotation_point.x
    final_y = rotated_point[1] + rotation_point.y

    # Create a new Point for the result
    rotated_point_msg = Point()
    rotated_point_msg.x = final_x
    rotated_point_msg.y = final_y
    rotated_point_msg.z = point_x.z  # Assuming rotation in the XY plane, Z remains the same

    return rotated_point_msg

def calculate_angle_between_vectors(v1: Point, v2: Point):
    # Convert the vectors to numpy arrays
    v1 = np.array([v1.x, v1.y])
    v2 = np.array([v2.x, v2.y])
    
    # Calculate the dot product
    dot_product = np.dot(v1, v2)
    
    # Calculate the magnitudes (norms) of the vectors
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    
    # Calculate the angle in radians
    angle_rad = np.arccos(dot_product / (norm_v1 * norm_v2))
    
    # Calculate the cross product (in 2D, it's a scalar representing the z-component)
    cross_product_z = np.cross(v1, v2)
    
    # Determine the sign of the angle
    if cross_product_z < 0:
        angle_rad = -angle_rad
    
    return angle_rad