#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, CameraInfo

class CamCapture():
    def __init__(self):
        self.camID = rospy.get_param("~camID",0)
        self.width = rospy.get_param("~width",320)
        self.height = rospy.get_param("~height",240)
        self.rate = rospy.get_param("~rate",30)
        rospy.loginfo("use camera id=%d, w=%d, h=%d, rate=%f" % (self.camID,self.width,self.height,self.rate))
        
        self.cap = cv2.VideoCapture(self.camID)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.br = CvBridge()
        self.pub = rospy.Publisher("/topower_v1/camera/image_raw/compressed",CompressedImage,queue_size=1)

    def Run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            msg = self.br.cv2_to_compressed_imgmsg(frame)
            self.pub.publish(msg)
            #cv2.imshow('frame',frame)
            #cv2.waitKey(1)
            rate.sleep() 

if __name__ == '__main__':
    rospy.init_node('cam_capture_node')
    rospy.loginfo("cam_capture_node started")
    cam = CamCapture()
    cam.Run()