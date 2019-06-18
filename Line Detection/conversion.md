# Conversion
Creates a publisher topic ```LineDistance``` and subscriber topic ``` image_raw```.

Converts an image message to open cv2.

Scans the image and creates a mask that leaves only the color white behind.

Greyscales the image and applies canny edge detection.

Finds the intersection between the greyscalewd image and the masked image.

Removes glare using HSV.

Extract lines using the hough transform.

Finds parallel lines.

Publishing the distance from the closest line.
