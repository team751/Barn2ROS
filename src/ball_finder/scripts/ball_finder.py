#!/usr/bin/env python
import roslib
import sys
import rospy
import copy
import cv2
import numpy as np;
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      input = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    output = copy.copy(input)

    input3 = input;
    input3 = cv2.cvtColor(input3, cv2.COLOR_BGR2HSV)
    input = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(input, np.array([0, 0, 0],np.uint8), np.array([180, 180, 255],np.uint8))

    # input = cv2.bitwise_and(input, input, mask = mask)


    input2 = cv2.cvtColor(input, cv2.COLOR_HSV2BGR)
    input2 = cv2.cvtColor(input2, cv2.COLOR_BGR2GRAY)

    mask = cv2.adaptiveThreshold(input2,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,39,125)
    input = cv2.bitwise_and(input, input, mask = mask)
     
    input = cv2.cvtColor(input, cv2.COLOR_HSV2BGR)
    input = cv2.cvtColor(input, cv2.COLOR_BGR2GRAY)

    # input = cv2.equalizeHist(input)

    input = cv2.bilateralFilter(input, 9, 5000,5000)

    # input = cv2.Canny(input,35,30)

    # input = cv2.bilateralFilter(input, 3, 5000,5000)

    circles = cv2.HoughCircles(input, cv2.cv.CV_HOUGH_GRADIENT, 3, 200, param1 = 100, param2 = 100, minRadius = 25, maxRadius = 500)

    cimg = output

    if circles != None:
      circles = np.uint16(np.around(circles))
      j = 0
      for i in circles[0,:]:
          if j < 50:
            # draw the outer circle
            cv2.circle(cimg,(i[0],i[1]),i[2],(0 if j == 0 else 255, 255 if j == 0 else 0,0),2)
            # draw the center of the circle
            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
            j = j+1

    cv2.imshow("input", input);
    cv2.imshow("cimg", cimg);

    cv2.waitKey(3);


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

