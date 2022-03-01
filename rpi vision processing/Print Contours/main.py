# Import the camera server
from cscore import CameraServer

# Import OpenCV and NumPy
import cv2
import numpy as np

# Import networktables
from networktables import NetworkTables

w = 160
h = 120

NetworkTables.initialize(server='10.32.6.2')

def drawRectangle(image,rect,color):
    cv2.rectangle(image,(int(rect[0]),int(rect[1])),(int(rect[0]+rect[2]),int(rect[1]+rect[3])),color,2)

def main():

    w = 160
    h = 120

    cs = CameraServer.getInstance()
    cs.enableLogging()

    # Capture from the first USB Camera on the system
    camera = cs.startAutomaticCapture()
    camera.setResolution(160, 120)

    # Get a CvSink. This will capture images from the camera
    cvSink = cs.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = cs.putVideo("Name", 160, 120)

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(160, 120, 3), dtype=np.uint8)

    # get range table
    colors = NetworkTables.getTable("BallLocator")

    colors.putNumber("lowh", 0)
    colors.putNumber("lows", 57)
    colors.putNumber("lowv", 78)

    colors.putNumber("highh", 11)
    colors.putNumber("highs", 255)
    colors.putNumber("highv", 255)

    colors.putNumber("kernelw", 3)
    colors.putNumber("kernelh", 3)

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, img = cvSink.grabFrame(img)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue

        #
        # Insert your image processing logic here!
        #

        lowh = colors.getNumber("lowh", 0)
        lows = colors.getNumber("lows", 57)
        lowv = colors.getNumber("lowv", 78)

        highh = colors.getNumber("highh", 11)
        highs = colors.getNumber("highs", 255)
        highv = colors.getNumber("highv", 255)


        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_threshold = cv2.inRange(img_hsv, (lowh, lows, lowv), (highh, highs, highv))

        kernel = np.ones((
            int(colors.getNumber("kernelw", 3)), 
            int(colors.getNumber("kernelh", 3))
        ), np.uint8)
        binary_img = cv2.morphologyEx(img_threshold, cv2.MORPH_OPEN, kernel)

        _, contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            _, _, w, h = cv2.boundingRect(contour)
            if not (w-3 <= w+3 and h-3 <= h+3):
                contours.remove(contour)

        if len(contours) > 0:
            largest = contours[0]
            for contour in contours:
                if cv2.contourArea(contour) > cv2.contourArea(largest):
                    largest = contour
            
            x, y, w, h = cv2.boundingRect(largest)
            drawRectangle(binary_img, [x, y, w, h], (255, 0, 255))
            d = (x + (w / 2)) - (160 / 2)
            print(d)
            colors.putNumber("Distance from center", d)
        else:
            d = 0
            colors.putNumber("Distance from center", 0)


            
        
        drawRectangle(binary_img, [160, 120, 50, 50], (255, 0, 255))

        outputStream.putFrame(binary_img)