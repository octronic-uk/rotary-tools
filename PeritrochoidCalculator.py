import math
import numpy as np
from matplotlib import pyplot as plt

class Point:
    """
        A point in cartesian space
    """
    x = 0.0
    y = 0.0

    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


class Circle:
    """
        A cartesian circle
    """
    origin = Point()
    radius = 1.0

    def __init__(self, origin=Point(), radius=1.0):
        self.origin = origin
        self.radius = radius

    def concentric_point(self, theta, radius):
        return Point(
            self.origin.x+(radius * math.cos(theta)),
            self.origin.y+(radius * math.sin(theta))
        )


class PeritrochoidCalculator:

    """
        This class calculates the peritrochoid (inner envelope) of the wankel engine.

        Calculations are derived from Kenici Yamamoto's Rotary Engine Whitepaper. [1]

        Kenichi Yamamoto's notation will be marked as KYN herein.

        References
            [1] http://scottishnationalstandardbearer.000webhostapp.com/re/Trochoids/Trochoids.htm

    """

    """ 
        Base circle
            origin = A in KYN 
            radius = p in KYN 
    """
    base_circle = None

    """
        Cartesian revolving circle
            origin = B in KYN 
            radius = q in KYN
    """
    revolving_circle = None

    # N.B.
    #   * The following constants are arbitrary
    #   * All thetas are in RADIANS, I'm not a savage...

    """
        Initial angle of revolving circle
            alpha in KYN
    """
    revolving_circle_theta = 0

    """ 
        Arm Length 
            R in KYN 
    """
    generating_radius = 20

    """
        Eccentricity e in KYN
    """
    eccentricity = 5

    """
        Trochoid 'm' value
            m in KYN
    """
    trochoid_m = 3

    """
        Amount to increment theta in radians per iteration during generation.
        
        This variable will essentially determine the plot's accuracy.
        There will be 
        
            (PI*(2*trochoid_m))*theta_increment
            
        plots points made during generation.
    """
    theta_increment = 0.001

    def __init__(self,
                 base=Circle(Point(0, 0), radius=10),
                 revolving_circle_radius=20):

        # Set the base circle origin/radius
        self.base_circle = base

        # Calculate the revolving circle origin from the base
        self.revolving_circle = Circle(
            self.base_circle.concentric_point(
                self.revolving_circle_theta,
                self.eccentricity
            ), revolving_circle_radius
        )

    def generate(self):

        # Array of points generated
        points = []

        for next_theta in np.arange(0, math.pi*(self.trochoid_m*2), self.theta_increment):

            # Get the next theta
            self.revolving_circle_theta = next_theta

            # set the origin of the revolving circle
            self.revolving_circle.origin = self.base_circle.concentric_point(
                self.revolving_circle_theta,
                self.eccentricity
            )

            next_point = self.revolving_circle.concentric_point(
                next_theta/self.trochoid_m,
                self.generating_radius
            )

            points.append([next_point.x, next_point.y])

        return np.array(points)

    def set_kyn_e(self, e=1.0):
        self.eccentricity = e

    def set_kyn_R(self, r=1.0):
        self.generating_radius = r


    def set_kyn_m(self, m=3.0):
        self.trochoid_m = m


if __name__ == "__main__":
    pc = PeritrochoidCalculator()

    pc.set_kyn_e(17.5)
    pc.set_kyn_R(116)
    pc.set_kyn_m(3)

    generated_points = pc.generate()

    # debug array size
    print("Generated %d points" % len(generated_points))

    x, y = generated_points.T
    plt.scatter(x, y)
    plt.show()
