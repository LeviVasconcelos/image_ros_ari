import time, sys, os
import numpy as np
from ros import rosbag
import roslib, rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
import cv2

TOPIC = '/head_front_camera/image_raw/compressed'

def CreateVideoBag(videopath, bagname):
    '''Creates a bag file with a video file'''
    bag = rosbag.Bag(bagname, 'w')
    cap = cv2.VideoCapture(videopath)
    cb = CvBridge()
    prop_fps = cap.get(cv2.CAP_PROP_FPS)
    if prop_fps != prop_fps or prop_fps <= 1e-2:
        print "Warning: can't get FPS. Assuming 24."
        prop_fps = 24
    ret = True
    frame_id = 0
    while(ret):
        ret, frame = cap.read()
        if not ret:
            break
        stamp = rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
        frame = cv2.flip(frame, -1)
        frame_id += 1
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        #image = cb.cv2_to_imgmsg(frame, encoding='bgr8')
        #image.header.stamp = stamp
        #image.header.frame_id = "camera"
        #bag.write(TOPIC, image, stamp)
        bag.write(TOPIC, msg, stamp)
    cap.release()
    bag.close()

if __name__ == "__main__":
    if len( sys.argv ) == 3:
        CreateVideoBag(*sys.argv[1:])
    else:
        print( "Usage: video2bag videofilename bagfilename")
