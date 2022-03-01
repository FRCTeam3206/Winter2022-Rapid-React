# Import the camera server
from cscore import CameraServer

# Import OpenCV and NumPy
import cv2
import numpy as np

w = 160
h = 120
def drawRectangle(image,rect,color):
    cv2.rectangle(image,(int(rect[0]),int(rect[1])),(int(rect[0]+rect[2]),int(rect[1]+rect[3])),color,2)
def main():
    cs = CameraServer.getInstance()
    cs.enableLogging()

    # Capture from the first USB Camera on the system
    camera = cs.startAutomaticCapture()
    camera.setResolution(w, h)

    # Get a CvSink. This will capture images from the camera
    cvSink = cs.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = cs.putVideo("Name", w, h)

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
        img_threshold = cv2.inRange(img_hsv, (88, 70, 118), (100, 144, 255))
        #blue
        #img_threshold = cv2.inRange(img_hsv, (161, 56, 203), (172, 156, 255))
        contours, _ = cv2.findContours(img_threshold, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        if len(contours)>0:
            drawRectangle(img,cv2.boundingRect(contours[0]),(255,255,255))
            print(cv2.boundingRect(contours[0]))
        outputStream.putFrame(img)