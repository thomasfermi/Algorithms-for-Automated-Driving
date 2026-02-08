
import numpy as np

def get_target_point(lookahead, polyline):
    """ Determines the target point for the pure pursuit controller
    
    Parameters
    ----------
    lookahead : float
        The target point is on a circle of radius `lookahead`
        The circle's center is (0,0)
    poyline: array_like, shape (M,2)
        A list of 2d points that defines a polyline.
    
    Returns:
    --------
    target_point: numpy array, shape (,2)
        Point with positive x-coordinate where the circle of radius `lookahead`
        and the polyline intersect. 
        Return None if there is no such point.  
        If there are multiple such points, return the one that the polyline
        visits first.
    """
    # Hint: A polyline is a list of line segments. 
    # The formulas for the intersection of a line segment and a circle are given
    # here https://mathworld.wolfram.com/Circle-LineIntersection.html
    raise NotImplementedError