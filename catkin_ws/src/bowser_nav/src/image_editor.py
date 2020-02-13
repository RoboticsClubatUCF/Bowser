# Peter Dorsaneo

# image_editor.py
# ===============
# This class was implemented to work best on the Logitech camera that will be 
# used for Bowsers side facing camera. Preferably on Bowser's right side, facing
# in the forward direction.

import cv2

class ImageEditor(object):
    """
    This is just a helper class for loading and editing a provided image. 
    Taking in the path to the image and then editing as needed.  
    """
    def __init__(self, image_path):
        try:
            self.image = self.read_image(image_path)
        except Exception as e:
            print("Bad image path.")
            raise e
        
    def read_image(self, image_path, color = cv2.IMREAD_COLOR): 
        return cv2.imread(image_path, color)

    def resize_image(self, image, new_dim = (128, 128)):
        return cv2.resize(image, new_dim)

    def applyGaussianBlur(self, image):
        return cv2.GaussianBlur(image, (13,13), 0)

    def decrease_image_brightness(self, image):
        value = 30 
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        lim = 0 + value
        v[v < lim] = 0
        v[v >= lim] -= value
        final_hsv = cv2.merge((h, s, v))
        image = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)

        return image
