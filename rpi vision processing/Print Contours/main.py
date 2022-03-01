# Import the camera server
from cscore import CameraServer

# Import OpenCV and NumPy
import cv2
import numpy as np

# Import networktables
from networktables import NetworkTables

NetworkTables.initialize(server="10.32.6.2")
locator_table = NetworkTables.getTable("BallLocator")

lowh = locator_table.getEntry("lowh")
lows = locator_table.getEntry("lows")
lowv = locator_table.getEntry("lowv")

highh = locator_table.getEntry("highh")
highs = locator_table.getEntry("highs")
highv = locator_table.getEntry("highv")


w = 160
h = 120
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
    thresholdStream = cs.putVideo("Thresholded image", 160, 120)

    # Allocating new images is very expensive, always try to preallocate
    image = np.zeros(shape=(w, h, 3), dtype=np.uint8)

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, image = cvSink.grabFrame(image)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #red
        img_threshold = cv2.inRange(img_hsv, 
            (lowh.getNumber(0), lows.getNumber(0), lowv.getNumber(0)), 
            (highh.getNumber(0), highs.getNumber(0), highv.getNumber(0))
        )
        #blue
        #img_threshold = cv2.inRange(img_hsv, (161, 56, 203), (172, 156, 255))
        _, contours, _ = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        for contour in contours[:5]:
            drawRectangle(image, cv2.boundingRect(contour),(255,255,255))

        outputStream.putFrame(image)
        thresholdStream.putFrame(img_threshold)
