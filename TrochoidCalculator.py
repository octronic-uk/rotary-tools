import math
import numpy as np
from matplotlib import pyplot as plt
import svgwrite
from dxfwrite import DXFEngine as dxf


class TrochoidCalculator:

    """
        This class calculates the Trochoid (inner envelope) of the wankel engine.

        Calculations are derived from Kenichi Yamamoto's Rotary Engine book. [1]

        Kenichi Yamamoto's notation will be marked as KYN herein.



        You can use the well proofed epitrochoid-parameters of the 294 ccm engines: [2]
            R = 71mm
            e = 11.6mm
            a = 0.5mm

        References
            [1] A Great source of rotary goodness
                http://scottishnationalstandardbearer.000webhostapp.com/re/Trochoids/Trochoids.htm

            [2] CNCZone thread discussing Wankel Engine Builds
                https://www.cnczone.com/forums/i-c-engines/206328-cnc.html
    """

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
        The radius that produces the parallel rotor housing trochoid
        a in KYN
    """
    a = 1.0

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

    """
        Radius of the shaft ratio to e
    """
    shaft_to_e_ratio = 1.5

    """
        Radius of the rotor_bore ratio to e
    """
    rotor_bore_to_e_ratio = 2.5

    """
        Using R+a for parallel trochoid envelope
        
        x = e sin alpha + R+a cos alpha/3
        y = e sin alpha + R+a sin alpha/3
    """
    def generate_trochoid(self):
        # Array of points generated
        points = []

        R = self.generating_radius+self.a

        for angle in np.arange(0, math.pi*(self.trochoid_m*2), self.theta_increment):
            points.append([
                self.eccentricity * math.cos(angle) + R * math.cos(angle/self.trochoid_m),
                self.eccentricity * math.sin(angle) + R * math.sin(angle/self.trochoid_m)
            ])

        return points

    """
        Naively implemented KY's RE
    """
    def generate_rotor(self):
        # Array of points generated
        points = []

        # Using self.theta_increment/2 for a bit more precision

        for v in np.arange((1.0/6.0)*math.pi, (3.0/6.0)*math.pi, self.theta_increment/2):
            next_point = self.generate_rotor_point(v)
            points.append(next_point)

        for v in np.arange((9.0/6.0)*math.pi, (11.0/6.0)*math.pi, self.theta_increment/2):
            next_point = self.generate_rotor_point(v)
            points.append(next_point)

        for v in np.arange((5.0/6.0)*math.pi, (7.0/6.0)*math.pi, self.theta_increment/2):
            next_point = self.generate_rotor_point(v)
            points.append(next_point)

        return points

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

        return [x, y]

    def generate_rotor_bore(self):

        points = []
        bore_radius = self.eccentricity * self.rotor_bore_to_e_ratio

        for theta in np.arange(0, math.pi*2, self.theta_increment):
            points.append([
                math.cos(theta)*bore_radius,
                math.sin(theta)*bore_radius,
            ])

        return points

    def generate_shaft(self):

        points = []

        radius = self.eccentricity*self.shaft_to_e_ratio

        for theta in np.arange(0, math.pi*2, self.theta_increment):
            points.append([
                math.cos(theta)*radius,
                math.sin(theta)*radius,
            ])

        return points

    def generate_shaft_rotor_bore(self):

        points = []

        bore_radius = self.eccentricity * self.rotor_bore_to_e_ratio

        for theta in np.arange(0, math.pi*2, self.theta_increment):
            points.append([
                self.eccentricity - (math.cos(theta)*bore_radius),
                math.sin(theta)*bore_radius,
            ])

        return points

    def set_e(self, e):
        """
            :param e: Set eccentricity value (mm)
        """
        self.eccentricity = e

    def set_r(self, r):
        """
            :param r: Set Generating Radius (mm)
        """
        self.generating_radius = r

    def set_m(self, m=3.0):
        self.trochoid_m = m

    def set_a(self, a):
        self.a = a

    def set_theta_increment(self, inc):
        self.theta_increment = inc

    def set_rotor_bore_to_e_ratio(self, ratio):
        self.rotor_bore_to_e_ratio = ratio

    def set_shaft_to_e_ratio(self,ratio):
        self.shaft_to_e_ratio = ratio


def append_svg(points, name="output.svg", drawing=None):

    if drawing is None:
        drawing = svgwrite.Drawing(name, profile='tiny')

    num_points = len(points)

    for index in range(num_points):

        current_point = points[index]
        next_point = points[(index+1) % num_points]

        drawing.add(drawing.line(
            (current_point[0], current_point[1]),
            (next_point[0], next_point[1]),
            stroke=svgwrite.rgb(0, 0, 0, '%'))
        )

    return drawing


def append_dxf(points, name="output.dxf", drawing=None):

    if drawing is None:
        drawing = dxf.drawing(name)
        # set unit of measurement to mm
        drawing.header['$LUNITS'] = 4

    num_points = len(points)

    for index in range(num_points):

        current_point = points[index]
        next_point = points[(index+1) % num_points]

        drawing.add(dxf.line(
            (current_point[0], current_point[1]),
            (next_point[0], next_point[1]),
            color=7)
        )

    return drawing


def show_graph(t_points, r_points, s_points):
    # Show as a graph

    refac_t_points = np.array([p for p in t_points])
    t_x, t_y = refac_t_points.T

    refac_r_points = np.array([p for p in r_points])
    r_x, r_y = refac_r_points.T

    refac_s_points = np.array([p for p in s_points])
    s_x, s_y = refac_s_points.T

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

    plt.scatter(t_x, t_y)
    plt.scatter(r_x, r_y)
    plt.scatter(s_x, s_y)

    plt.show()


if __name__ == "__main__":
    # Arguments from page 17 of KY's RE
    rotary_calc = TrochoidCalculator()

    rotary_calc.set_r(71)
    rotary_calc.set_e(11.6)
    rotary_calc.set_a(0.5)
    rotary_calc.set_shaft_to_e_ratio(1.5)
    rotary_calc.set_rotor_bore_to_e_ratio(3.25)

    rotary_calc.set_theta_increment(0.05)

    print("Trochoid Constant K = %f" % (rotary_calc.generating_radius / rotary_calc.eccentricity))

    # Housing Trochoid
    trochoid_points = rotary_calc.generate_trochoid()
    trochoid_drawing = append_dxf(trochoid_points, name="TrochoidHousing.dxf")
    trochoid_drawing.save()

    # Rotor
    rotor_points = rotary_calc.generate_rotor()
    rotor_bore_points = rotary_calc.generate_rotor_bore()
    rotor_all = []
    rotor_all.extend(rotor_points)
    rotor_all.extend(rotor_bore_points)
    rotor_drawing = append_dxf(rotor_points, name="Rotor.dxf")
    append_dxf(rotor_bore_points, drawing=rotor_drawing)
    rotor_drawing.save()

    # Shaft
    shaft_points = rotary_calc.generate_shaft()
    shaft_bore_points = rotary_calc.generate_shaft_rotor_bore()
    shaft_all = []
    shaft_all.extend(shaft_points)
    shaft_all.extend(shaft_bore_points)
    shaft_drawing = append_dxf(shaft_points, name="E-Shaft.dxf")
    append_dxf(shaft_bore_points, drawing=shaft_drawing)
    shaft_drawing.save()

    show_graph(np.array(trochoid_points), np.array(rotor_all), np.array(shaft_all))
