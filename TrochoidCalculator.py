import math
import numpy as np
from matplotlib import pyplot as plt
import svgwrite


class Point:
    """
        A point in cartesian space
    """
    x = 0.0
    y = 0.0

    def __init__(self, _x=0.0, _y=0.0):
        self.x = _x
        self.y = _y


class Circle:
    """
        A cartesian-friendly circle
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


class TrochoidCalculator:

    """
        This class calculates the Trochoid (inner envelope) of the wankel engine.

        Calculations are derived from Kenichi Yamamoto's Rotary Engine book. [1]

        Kenichi Yamamoto's notation will be marked as KYN herein.

        References
            [1] A Great source of rotary goodness
                http://scottishnationalstandardbearer.000webhostapp.com/re/Trochoids/Trochoids.htm
    """

    """ 
        Base circle
            origin = OA in KYN 
            radius = p in KYN 
    """
    base_circle = None

    """
        Cartesian revolving circle
            origin = OB in KYN 
            radius = q in KYN
    """
    revolving_circle = None

    # N.B.
    #   * The following initial variables are arbitrary.
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
    theta_increment = 0.01

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

    def generate_peritrochoid(self):

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

            points.append(next_point)

        return points

    """
        Naively implemented from [1] using
        
        X = R ×	( cos 2v - (1/R) (3e^2/2R) × (cos 4v - cos 8v) + (1/R) e √( 1 - 9 (e/R)^2 sin^2 3v ) × (cos 5v + cos v) )
        Y = R ×	( sin 2v + (1/R) (3e^2/2R) × (sin 8v + sin 4v) + (1/R) e √( 1 - 9 (e/R)^2 sin^2 3v ) × (sin 5v - sin v) )
    """
    def generate_rotor(self):
        # Array of points generated
        points = []

        for next_v in np.arange((1.0/6.0)*math.pi, (1.0/2.0)*math.pi, self.theta_increment):
            print("NextV = %f" % next_v)
            next_point = self.generate_rotor_point(next_v)
            points.append(next_point)

        for next_v in np.arange((5.0/6.0)*math.pi, (7.0/6.0)*math.pi, self.theta_increment):
            next_point = self.generate_rotor_point(next_v)
            points.append(next_point)

        for next_v in np.arange((3.0/2.0)*math.pi, (11.0/6.0)*math.pi, self.theta_increment):
            next_point = self.generate_rotor_point(next_v)
            points.append(next_point)

        return points

    """
        Holy crap!
    """
    def generate_rotor_point(self, v):

        R = self.generating_radius
        e = self.eccentricity

        x = R * math.cos(2*v) + \
            (3*(e**2) / (2*R)) * \
            (math.cos(8*v) - math.cos(4*v)) + \
            e * (1 - (9*(e**2) / R**2) * (math.sin(3*v)**2))**0.5 * \
            (math.cos(5*v) + math.cos(v))

        y = R * math.sin(2*v) + \
            (3*(e**2) / (2*R)) * \
            (math.sin(8*v) + math.sin(4*v)) + \
            e * (1 - (9*(e**2) / R**2) * (math.sin(3*v)**2))**0.5 * \
            (math.sin(5*v) - math.sin(v))

        return Point(x, y)

    def set_kyn_e(self, e=1.0):
        """
            :param e: Set eccentricity value (mm)
        """
        self.eccentricity = e

    def set_kyn_r(self, r=1.0):
        """
            :param r: Set Generating Radius (mm)
        """
        self.generating_radius = r

    def set_kyn_m(self, m=3.0):
        self.trochoid_m = m

    def set_theta_increment(self, inc):
        self.theta_increment = inc


def export_svg(generated_points, name):
    dwg = svgwrite.Drawing(name, profile='tiny')

    num_points = len(generated_points)

    for index in range(num_points):

        current_point = generated_points[index]
        next_point = generated_points[(index+1) % num_points]

        dwg.add(dwg.line(
            (current_point.x, current_point.y),
            (next_point.x, next_point.y),
            stroke=svgwrite.rgb(0, 0, 0, '%'))
        )

    dwg.save()


def show_graph(generated_points):
    # Show as a graph

    refactored_points = np.array([[p.x, p.y] for p in generated_points])
    x, y = refactored_points.T

    # Tart up the plot
    ax = plt.gca()

    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    ax.xaxis.set_ticks_position('bottom')
    ax.spines['bottom'].set_position(('data', 0))
    ax.yaxis.set_ticks_position('left')
    ax.spines['left'].set_position(('data', 0))

    ax.grid(which='major', axis='x', linewidth=0.75, linestyle='-', color='0.75')
    ax.grid(which='minor', axis='x', linewidth=0.25, linestyle='-', color='0.75')
    ax.grid(which='major', axis='y', linewidth=0.75, linestyle='-', color='0.75')
    ax.grid(which='minor', axis='y', linewidth=0.25, linestyle='-', color='0.75')

    ax.xaxis.set_major_locator(plt.MultipleLocator(10))
    ax.xaxis.set_minor_locator(plt.MultipleLocator(1))
    ax.yaxis.set_major_locator(plt.MultipleLocator(10))
    ax.yaxis.set_minor_locator(plt.MultipleLocator(1))

    plt.axis('equal')

    plt.scatter(x, y)
    plt.show()


if __name__ == "__main__":
    pc = TrochoidCalculator()

    # Arguments from page 17 of KY's RE
    pc.set_theta_increment(0.001)
    pc.set_kyn_e(17.5)
    pc.set_kyn_r(116)
    pc.set_kyn_m(3)

    pt_points = pc.generate_peritrochoid()
    export_svg(pt_points, "peritrochoid.svg")
    show_graph(np.array(pt_points))

    rotor_points = pc.generate_rotor()
    show_graph(np.array(rotor_points))
    export_svg(rotor_points, "rotor.svg")

