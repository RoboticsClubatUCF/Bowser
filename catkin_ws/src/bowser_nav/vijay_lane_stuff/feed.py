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
    image_path : string
        Path to the feed 

    Returns
    -------
    shows image
        if you call the show_image method, the image will be shown and wait for
        a keystroke to close.

    Notes
    -----
    To determine lanes, we first convert the feed to a single channel array,
    then determine the change in pixel value intensity to show an edge.

    Examples
    --------
    >>> import cv2, numpy as np
    >>> from feed import Feed
    >>> feed = Feed('image.jpg')
    >>> feed.show_image()

    Revisions
    ---------
    2020-10-15 Vijay Stroup created Feed class with show_image and to_grey
               methods.
    2020-10-16 Vijay Stroup created to_blur, to_canny, and roi methods.

    """

    WHITE = 255

    def __init__(self, image_path):
        self.image = cv2.imread(image_path)
        self.height, self.width, _ = self.image.shape

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

    def show_image(self):
        """Show image and wait for keystroke."""

        print('Press a key to exit, don\'t press the x on the window!')
        cv2.imshow('', self.image)
        cv2.waitKey()
        cv2.destroyAllWindows()

    def make_mask(self):
        """To make a mask we define an array of points on our feed, and then on
        our mask, we fill in the area of our mask by our points with white as
        to use bitwise & on our mask and feed
        
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
