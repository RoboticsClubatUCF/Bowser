# !/usr/bin/env python

# Jade Zsiros
# Marcus Simmonds
# Alexandra French
# Jonathan Conrad

# Cylinder detection by random sample vote

import numpy as np


class DetectCones:

    # This runs for every scan
    def on_scan(self, cloud):
        PointCloud2.angle_min = -1.5708
        PointCloud2.angle_max = 1.5708
        # Take in our x, y, z, values and store them in the generator
        self.xyzi_generator = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z", "i"))
        # Create an iterable array out of the generator
        # 0 = x, 1 = y, 2 = z
        iterable = [np.array([i[0], i[1], i[2], i[3]]) for i in self.xyzi_generator]
        # Create a numpy array out of the iterable
        self.points = np.array(iterable)
        # Update the current scan counter
        currScan += 1;

    # **Edit later**
    def __init__(self, num):
        self.a = num

    #  Completes calculation for 3D circle based on three points
    def cone_calculations(self):
        #  Each arrays with floats x,y,z representing the point
        point1 = np.array([2.0, 1.5, 0.0])
        point2 = np.array([6.0, 4.5, 0.0])
        point3 = np.array([11.75, 6.25, 0.0])

        # Algorithm for calculating circle modified from:
        # https://stackoverflow.com/questions/20314306/find-arc-circle-equation-given-three-points-in-space-3d



        # Take normalized length of line between each two points
        a = np.linalg.norm(point3 - point2)
        b = np.linalg.norm(point3 - point1)
        c = np.linalg.norm(point2 - point1)

        # Find radius of circle with these points
        radius = (a * b * c) / np.sqrt(2.0 * a ** 2 * b ** 2 +
                                       2.0 * b ** 2 * c ** 2 +
                                       2.0 * a ** 2 * c ** 2 -
                                       a ** 4 - b ** 4 - c ** 4)

        # Find barcyntric coordinates of center
        b1 = a * a * (b * b + c * c - a * a)
        b2 = b * b * (a * a + c * c - b * b)
        b3 = c * c * (a * a + b * b - c * c)

        # Create a center point as three values
        p = np.column_stack((point1, point2, point3)).dot(np.hstack((b1, b2, b3)))
        p /= b1 + b2 + b3

        # Print to screen the values after calculation
        print("----------------------------------------------------------")
        print('a:', a, 'b:', b, 'c:', c)
        print()
        print('radius:', radius)
        print()
        print('b1:', b1, 'b2:', b2, 'b3:', b3)
        print()
        print('p:', p)
        print("----------------------------------------------------------")


# **Edit later**
lidar = DetectCones(2.4)
lidar.cone_calculations()


# Math notes:
# Within one plane, sample three points and calculate the circle that they occupy
# Check the radius of this circle, and if it is within acceptable bounds, it is viable
# In order to find the circle, draw a triangle between the three points
# The perpendicular bisectors of the three sides of the triangle intersect at the center
# The radius is the distance between this center and one of the points
