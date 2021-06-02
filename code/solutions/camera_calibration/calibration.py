import numpy as np
import random

def get_y(x, m, c):
    """ evaluates one degree polynomial at given points.
    Parameters
    ----------
    x : np.array, shape (M)
        List of points to evaluate the polynomial on    
    m: float
        Slope of the line        
    c: float
        y-intercept of the line
    
    Returns:
    --------
    numpy.array, shape (M)
        evaluted y at given points.
    """
    return m*x + c

def get_intersection(coordinates1, coordinates2):
    """ returns intersection of two lines.
    Parameters
    ----------
    coordinates1 : np.array, shape (M, 2)
        List of points of line 1 
    coordinates2 : np.array, shape (M, 2)
        List of points of line 2     
    Returns:
    --------
    tuple, shape (2)
        point of intersection of two lines
    """
    m1, c1 = np.polyfit(coordinates1[:, 0], coordinates1[:, 1], 1)
    m2, c2 = np.polyfit(coordinates2[:, 0], coordinates2[:, 1], 1)
    
    x_i = (c2 - c1) / (m1 - m2)
    y_i = get_y(x_i, m1, c1)
    
    return (x_i, y_i)
    
def get_py_from_vp(x_i, y_i, K):
    """ returns pitch and yaw from vanisging point
    Parameters
    ----------
    x_i : float 
        x coordinate of vanishing point in image space.
    y_i : float 
        y coordinate of vanishing point in image space.    
    K : np.array shape (3, 3) 
        intrinsic matrix of camera
    Returns:
    --------
    tuple, shape (2)
        pitch and yaw
    """
    p_infinity = np.array([x_i, y_i, 1])
    K_inv = np.linalg.inv(K)
    K_inv_p_infinity = K_inv @ p_infinity
    K_inv_p_infinity
    
    r3 = K_inv_p_infinity / np.linalg.norm(K_inv_p_infinity)
    yaw = np.arctan2(r3[0], r3[2])
    pitch = np.arccos(r3[1])
    
    return (pitch, yaw)
