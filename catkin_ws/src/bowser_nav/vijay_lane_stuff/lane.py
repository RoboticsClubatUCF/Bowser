"""lane.py is responsible for the lane detection part of Bowser and will
determine where the lanes are and also send signals to Bowser telling it
whether it should move left or right to stay within the lines.

"""

import os
import cv2
from feed import Feed


if __name__ == '__main__':
    feed_types = ['image', 'video', 'stream', 'camera']

    raw_feed = 'road2.mp4'
    raw_feed_type = feed_types[1]

    if not os.path.isfile(raw_feed):
        print('feed does not exist.')
        exit(-1)

    if raw_feed_type == feed_types[0]: # image
        feed = Feed(raw_feed, raw_feed_type)
        feed.robo_vis()
        feed.show_lanes()
    elif raw_feed_type == feed_types[1]: # video
        cap = cv2.VideoCapture(raw_feed)
        while(cap.isOpened()):
            _, frame = cap.read()
            feed = Feed(frame, raw_feed_type)
            feed.robo_vis()
            if feed.show_lanes() == -2: break
    elif raw_feed_type == feed_types[2]: # stream
        pass
    else: # camera
        pass
