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

    Examples
    --------
    >>> import cv2, numpy as np, os
    >>> from lane import Feed
    >>> feed = Feed('image.jpg')
    >>> feed.show_image()

    Revisions
    ---------
    2020-10-15 Vijay Stroup created Feed class with show_image method.

    """

    def __init__(self, image_path):
        self.image = cv2.imread(image_path)

    def show_image(self):
        """show image and wait for keystroke"""

        cv2.imshow('', self.image)
        cv2.waitKey()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    image_path = 'road1.jpg'

    if os.path.isfile(image_path):
        feed = Feed(image_path)
        feed.show_image()
    else:
        print('Image does not exist.')
        exit(-1)
