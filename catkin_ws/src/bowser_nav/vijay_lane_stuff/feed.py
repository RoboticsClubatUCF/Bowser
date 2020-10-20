"""feed.py contains a class Feed that is used for all the data feed transforms
in order for Bowser to see lane markings.

"""

import cv2
import numpy as np


class Feed:
    """Class that initializes an image and then preforms transforms on the
    image to make it desirable for lane detection.

    Parameters
    ----------
    raw_feed : string
        Path to the feed
    feed_type : string
        Type of the feed (i.e. picture, video, stream, or camera)

    Notes
    -----
    To determine lanes, we first convert the feed to a single channel array,
    then determine the change in pixel value intensity to show an edge.

    Examples
    --------
    >>> import cv2, numpy as np
    >>> from feed import Feed
    >>> feed = Feed('image.jpg')
    >>> feed.robo_vis()
    >>> feed.show_lanes()

    Revisions
    ---------
    2020-10-15 Vijay Stroup created Feed class with show_image and to_grey
               methods.
    2020-10-16 Vijay Stroup created to_blur, to_canny, and roi methods.
    2020-10-19 Vijay Stroup created get_lanes and show_lanes methods.
    2020-10-20 Vijay Stroup created robo_vis and make_lane_coords methods. He
               also edited the get_lanes method to average out the lanes to get
               one single right and left lane to make the lanes smoother.

    """

    WHITE = (255, 255, 255)
    BLUE = (255, 0, 0)

    def __init__(self, raw_feed, feed_type):
        self.feed_type = feed_type
        if self.feed_type == 'picture': self.image = cv2.imread(raw_feed)
        elif self.feed_type == 'video': self.image = raw_feed
        self.height, self.width, _ = self.image.shape
        self.image_copy = np.copy(self.image)
        self.lanes = None

    def to_grey(self):
        """Convert the image to grey scale so our color channel will only be 1
        with values ranging from 0 to 255 to make it faster and eaiser to
        determine change in intensity of pixel values.

        """

        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

    def to_blur(self):
        """We use a Gaussian Blur to smoothen hard edges that could give us
        false positives on edges, but keep the very dramatic average change in
        pixel values we need to see the lane markings.

        """

        kernal = (5, 5)
        deviation = 0

        self.image = cv2.GaussianBlur(self.image, kernal, deviation)

    def to_canny(self):
        """Canny determines the relative intensity in pixel values within a
        particular region. The Canny function preforms a derivative with
        respect to x and y - f(x, y) to determine intensity with respect to
        adjacent pixels (the gradient).

        Notes
        -----
        The underlying function cv2.Canny() preforms a Gaussian Blur.

        """

        low_threshold = 50
        high_threshold = 150

        self.image = cv2.Canny(self.image, low_threshold, high_threshold)

    def roi(self):
        """The region of intrest allows us to only use part of the feed to
        preform manipulations on to improve effency. The region of intrest we
        care about for our cause is just the lane Bowser is in, not the other
        side of the lane or what is to it's far right or left.

        Notes
        -----
        We use bitwise & with our mask and feed to only pick out the white that
        is in both our mask and our feed.

        """

        mask = self.make_mask()

        self.image = cv2.bitwise_and(self.image, mask)

    def make_mask(self):
        """To make a mask we define an array of points on our feed, and then on
        our mask, we fill in the area of our mask by our points with white as
        to use bitwise & on our mask and feed.

        Returns
        -------
        mask: tuple
            a black image with the same dimensions as the feed with white only
            in our defined roi.

        """

        lower_left = (0, self.height)
        lower_right = (self.width, self.height)
        middle = (self.width / 2, self.height / 2)

        points = np.array([
            [lower_left, lower_right, middle]
        ], dtype=np.int32)

        mask = np.zeros_like(self.image)
        cv2.fillPoly(mask, points, self.WHITE)

        return mask

    def get_lanes(self):
        """The Hough Space will allow us to find intersectional points of the
        different slopes from the Canny edge finder that will allow us to find
        what might be a stright line.

        Notes
        -----
        The Hough Space is a 2D array with the rows being theata in raidans and
        rho as the columns.
        When determing the slopes, the left lane would have a negative slope
        whereas the right lane would have a positive slope.
        After getting the Hough lines, we then average out the arrays in order
        to form one single lane so the lane lines are not as choppy.

        """

        rho = 2
        theata = np.pi / 180
        threshold = 100 # how many intersecting points must there to be
                        # considered a stright line
        lines = np.array([])
        minLineLength = 40 # any lines detected that are less than 40px are
                           # rejected
        maxLineGap = 35 # max distance two detected lines can be a part from
                        # each other and then combine

        hough_lanes = cv2.HoughLinesP(
            self.image,
            rho,
            theata,
            threshold,
            lines,
            minLineLength,
            maxLineGap
        )

        left_hough = []
        right_hough = []

        if hough_lanes is not None:
            # get the slopes and y-intercepts of each Hough Line
            for lane in hough_lanes:
                x1, y1, x2, y2 = lane.reshape(4)
                slope, intercept = np.polyfit((x1, x2), (y1, y2), 1)
                if slope < 0:
                    left_hough.append((slope, intercept))
                else:
                    right_hough.append((slope, intercept))

            # average out the left_hough and right_hough to form single lanes
            left_average = np.average(left_hough, axis = 0)
            right_average = np.average(right_hough, axis = 0)

            # get coordinates of left and right average arrays
            left_lane = self.make_lane_coords(left_average)
            right_lane = self.make_lane_coords(right_average)
            self.lanes = np.array([left_lane, right_lane])
        else:
            print('hough_lanes is None')
            exit(-1)

    def make_lane_coords(self, lane_average):
        """Make coordinates for plotting the lanes from the hough lane
        averages.

        Notes
        -----
        The image_height_ratio is how far up we will look in the feed. Example,
        3 / 5 would be looking from the top.

        Returns
        -------
        numpy array
            an array that holds the coordinates of each of the lanes.

        """

        image_height_ratio = 3 / 5

        slope, intercept = lane_average

        y1 = self.image.shape[0]
        y2 = int(y1 * image_height_ratio)
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)

        return np.array([x1, y1, x2, y2])

    def show_lanes(self):
        """Show lanes on feed.

        Returns
        -------
        -2 : int
            return code to break out of loop

        Notes
        -----
        When preforming cv2.addWeighted, it is the same thing as doing
        cv2.bitwise_or on the two images. The weighted just lets us see the
        lanes better as it will be more prominent.

        """

        lanes_image = np.zeros_like(self.image_copy)

        if self.lanes is not None:
            for x1, y1, x2, y2 in self.lanes:
                cv2.line(lanes_image, (x1, y1), (x2, y2), self.BLUE, 5)

        lanes_overlay = cv2.addWeighted(self.image_copy, .8, lanes_image, 1, 1)
        # lanes_overlay = cv2.bitwise_or(self.image_copy, lanes_image)

        print('Press a key to exit, don\'t press the x on the window!')
        cv2.imshow('', lanes_overlay)
        if self.feed_type == 'image': cv2.waitKey()
        elif self.feed_type == 'video':
            if cv2.waitKey(1000) & 0xFF == ord('q'): return -2
        cv2.destroyAllWindows()

    def show_image(self):
        """Show image and wait for keystroke."""

        print('Press a key to exit, don\'t press the x on the window!')
        cv2.imshow('', self.image)
        cv2.waitKey()
        cv2.destroyAllWindows()

    def robo_vis(self):
        """Call other methods to transform feed for Bowser."""

        self.to_grey()
        self.to_blur()
        self.to_canny()
        self.roi()
        self.roi()
        self.get_lanes()
