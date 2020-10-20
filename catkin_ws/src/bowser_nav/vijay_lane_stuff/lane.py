"""lane.py is responsible for the lane detection part of Bowser and will
determine where the lanes are and also send signals to Bowser telling it
whether it should move left or right to stay within the lines.

"""

import os
from feed import Feed


if __name__ == '__main__':
    raw_feed = 'road1.jpg'

    if os.path.isfile(raw_feed):
        feed = Feed(raw_feed)
        feed.robo_vis()
        feed.show_lanes()
    else:
        print('Image does not exist.')
        exit(-1)
