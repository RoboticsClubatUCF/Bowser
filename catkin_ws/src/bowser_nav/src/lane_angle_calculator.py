# Peter Dorsaneo

# lane_angle_calculator.py
# ========================

# Built on python version 3.7
# Implementation tested using webcam images from the provided Logitech webcam 
# to be used on Bowser. 
# Ideally this camera will be oriented on the right side of Bowser and facing  
# in the forward direction.

# Usage: python3 lane_angle_calculator.py

import cv2, math, os
import numpy as np
import matplotlib.pyplot as plt

from image_editor import ImageEditor

class LaneAngleCalculator(ImageEditor):
    """
    This class takes in a path to a lane image (preferably a white lane on a 
    green grass background). It will calculate the angle of the lane relative to
    the orientation of the camera. 
    """
    def __init__(self, image_path):
        super(LaneAngleCalculator, self).__init__(image_path)
        self.radians = 0.0
        self.degrees = 0.0
        self.original = self.image.copy()
        self.lines = None
        
    def preProcessImage(self):
        self.image = self.resize_image(self.image)
        self.image = self.decrease_image_brightness(self.image) 
        self.image = self.applyGaussianBlur(self.image)

        return self.image  

    # Uses the distance formula for line segments. 
    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def getCannyEdges(self):
        self.image = self.preProcessImage()
        
        return cv2.Canny(self.image, 30, 120)

    def drawHoughLinesP(self):
        edges = self.getCannyEdges()

        return cv2.HoughLinesP(edges, 
                                2, 
                                np.pi/180, 
                                50, 
                                minLineLength=10, 
                                maxLineGap=250)

    # Gets and returns the strongest edge line found in the image, this is 
    # expected to be and edge corresponding to the white lane.
    def filterHoughLines(self):

        self.lines = self.drawHoughLinesP()

        if self.lines is None: 
            print("No HoughLines were able to be drawn in image.")
            print("Troubleshooting required.")
            return None

        # We might get multiple Hough Lines on one edge so we will want to 
        # filter out and get at most two lines. 
        k = 0
        strong_lines = np.zeros([2, 1, 4], dtype=int)

        strong_lines[k] = self.lines[k]

        for line in self.lines: 
            for l in line:
                rho = l[0]
                theta = l[1]

                # Does some math. 
                if rho < 0: 
                    rho *= -1
                    theta -= np.pi

                # Some more math. 
                closeness_rho = np.isclose(rho, \
                    strong_lines[0:k,0,0], atol = 10)
                closeness_theta = np.isclose(theta, \
                    strong_lines[0:k,0,1], atol = np.pi/36)
                closeness = np.all([closeness_rho, closeness_theta], axis=0)

                # Checks if the line is not close to any others. 
                # If not then we have a distinguished strong line in the image. 
                # We will add that to our strong_lines. 
                if not any(closeness) and k < 2:
                    strong_lines[k] = line
                    k += 1

                if (k >= 2): 
                    break

        return strong_lines

    def getStrongestHoughLine(self):
        strong_lines = self.filterHoughLines();
        # Now we filter out between the two to get the line with a longer 
        # distance. 
        strongest = (0,0,0,0)

        for strong_line in strong_lines: 
            x1, y1, x2, y2 = strong_line[0]

            d1 = self.distance(x1, y1, x2, y2)
            d2 = self.distance(strongest[0], strongest[1], strongest[2], strongest[3])

            if (d1 > d2):
                strongest = x1, y1, x2, y2

        # Store our strongest line back in the usable variables. 
        # x1, y1, x2, y2 = strongest
        return strongest

    def calculateLaneAngle(self):
        '''
        Ideally, our lane will be slanted and the image will have to be thought 
        of as a triangle where the hypotenuse is the line drawn on the lane from 
        doing the Hough Lines and getting the strongest two of the Hough Lines 
        in the image. 

        Ideally, the two strongest Hough Lines will correspond to the lane in 
        the image.


        An example of this described using the common right triangle! 

                |\
                | \
              A |  \
                |   \  C (The lane in the image.)
                |    \
                |     \
                |  a / \ 
                |___/___\
                    B

        We just need to get the angle corresponding the 'a' in the image above, 
        and since we already have the measurements for A, B, and C edges we can 
        use something like the law of cosines. 

        '''
        x1, y1, x2, y2 = self.getStrongestHoughLine()

        hypotenuse = self.distance(x1, y1, x2, y2)

        # Drawing the strongest line on the image. Edge corresponding to C. 
        cv2.line(self.image, (x1, y1), (x2, y2), (0, 0, 255), 5)

        # Draw a line that crosses the diagonal of the entire image. 
        # mid = (int(x1+x2/2), int(y1+y2/2))
        # cv2.line(cv_image, (0, dim[1]), (dim[0], 0), (0, 255, 255), 5)

        # Draw a line that corresponds with the bottom of the image.
        y_max = max(y1, y2)
        y_min = min(y1, y2)
        x_min = min(x1, x2)
        x_max = max(x1, x2)

        # This is the edge corresponding to A (see above reference diagram).
        cv2.line(self.image, (x_min, y_min), (x_min, y_max), (255, 255, 0), 5)
        opposite = self.distance(x_min, y_min, x_min, y_max)

        # This is the edge corresponding to B (see above reference diagram).
        cv2.line(self.image, (x_min, y_max), (x_max, y_max), (0, 255, 0), 5)
        adjacent = self.distance(x_min, y_max, x_max, y_max)

        # Calculate the angle for 'a' by using the Law of Cosines. 
        # opposite^2 = hypotenuse^2 + adjacent^2 - 2(hypotenuse)(adjacent)cos(a)
        x = (hypotenuse**2) + (adjacent**2) - (opposite**2)
        y = 2 * hypotenuse * adjacent

        self.radians = math.acos((x) / (y)) 
        self.degrees = self.radians * 180 / math.pi

# Helper function for building a matplot image. 
def build_and_show_plot(imgs_to_plot, img_titles):
    # Build the matplot lib image here and do plt.show(). 
    rows = 3
    cols = int(len(imgs_to_plot) / rows)
    print(cols)

    fig = plt.figure(figsize=(10,10))

    for index, img in enumerate(imgs_to_plot):
        a = fig.add_subplot(rows, cols, index + 1)
        plt.imshow(img)
        plt.axis('off')

        a.set_title(img_titles[index]) 

    plt.show()


def main():

    lane_images_dir = "lane_images/"
    image_files = os.listdir(lane_images_dir);

    images_to_plot = list()
    image_titles = list()

    for image_file in image_files:
        image_path = os.path.join(lane_images_dir, image_file)
        lac = LaneAngleCalculator(image_path)
        lac.calculateLaneAngle()

        images_to_plot.append(lac.image)
        image_titles.append("Degrees: {:.2f}".format(lac.degrees))

    build_and_show_plot(images_to_plot, image_titles)

if __name__ == '__main__':
    main()
