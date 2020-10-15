"""Lane.py is responsible for the lane detection part of Bowser and will
determine where the lanes are and also send signals to Bowser telling it
whether it should move left or right to stay within the lines. 

"""

import cv2
import numpy as np
import os


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
    >>> import cv2, numpy as np, os
    >>> from lane import Feed
    >>> feed = Feed('image.jpg')
    >>> feed.show_image()

    Revisions
    ---------
    2020-10-15 Vijay Stroup created Feed class with show_image and to_grey
               methods.

    """

    def __init__(self, image_path):
        self.image = cv2.imread(image_path)

    def to_grey(self):
        """Convert the image to grey scale so our color channel will only be 1
        with values ranging from 0 to 255 to make it faster and eaiser to
        determine change in intensity of pixel values."""

        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

    def show_image(self):
        """Show image and wait for keystroke."""

        print('Press a key to exit, don\'t press the x on the window!')
        cv2.imshow('', self.image)
        cv2.waitKey()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raw_feed = 'road1.jpg'

    if os.path.isfile(raw_feed):
        feed = Feed(raw_feed)
        feed.to_grey()
        feed.show_image()
    else:
        print('Image does not exist.')
        exit(-1)
