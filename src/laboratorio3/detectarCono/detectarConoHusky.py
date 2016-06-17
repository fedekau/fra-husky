#!/usr/bin/env python

'''
Feature homography
==================

Example of using features2d framework for interactive video homography matching.
ORB features and FLANN matcher are used. The actual tracking is implemented by
PlaneTracker class in plane_tracker.py

Inspired by http://www.youtube.com/watch?v=-ZNYoL8rzPY

video: http://www.youtube.com/watch?v=FirtmYcC0Vc

Usage
-----
feature_homography.py [<video source>]

Keys:
   SPACE  -  pause video

Select a textured planar object to track by drawing a box with a mouse.
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2

# local modules
import video
from video import presets
import common
from common import getsize, draw_keypoints
from plane_tracker import PlaneTracker

from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image

class App:
    def __init__(self, src):
        #self.cap = video.create_capture(src, presets['book'])
	self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
	self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
	self.bridge = CvBridge()
	#dtype, n_channels = br.encoding_as_cvtype2('8UC3')
	#this.im = np.ndarray(shape=(480, 640, n_channels), dtype=dtype)
	self.cv_image = None
        self.frame = None
        self.paused = False
        self.tracker = PlaneTracker()

        cv2.namedWindow('plane')
        self.rect_sel = common.RectSelector('plane', self.on_rect)

    def callback(self,data):
        try:
	    self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	
	    #print(self.cv_image)
        except CvBridgeError as e:
            print(e)
	#(rows,cols,channels) = self.cv_image.shape
        #if cols > 60 and rows > 60 :
        #    cv2.circle(self.cv_image, (50,50), 10, 255)

        #cv2.imshow("Image window", self.cv_image)
        #cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
    
    def on_rect(self, rect):
        self.tracker.clear()
        self.tracker.add_target(self.frame, rect)

    def run(self):
        while not rospy.is_shutdown():
            playing = not self.paused and not self.rect_sel.dragging
            if playing or self.frame is None:
		#print(self.cv_image)
                ret = True
		frame = self.cv_image
		#print(self.cv_image)
		#print("**************************** frame")
		#print(frame)
		#ret, frame = self.cap.read()
		if not ret:
                    break
                self.frame = frame.copy()
	    
            w, h = getsize(self.frame)
            vis = np.zeros((h, w*2, 3), np.uint8)
            vis[:h,:w] = self.frame
            if len(self.tracker.targets) > 0:
                target = self.tracker.targets[0]
                vis[:,w:] = target.image
                draw_keypoints(vis[:,w:], target.keypoints)
                x0, y0, x1, y1 = target.rect
		print(x0,y0,x1,y1)
                cv2.rectangle(vis, (x0+w, y0), (x1+w, y1), (0, 255, 0), 2)
	    
            if playing:
                tracked = self.tracker.track(self.frame)
                if len(tracked) > 0:
                    tracked = tracked[0]
		    #Aca se imprimen los 4 puntos que genera
		    print(str(np.int32(tracked.quad[0])))
		    print(str(np.int32(tracked.quad[1])))
		    print(str(np.int32(tracked.quad[2])))
		    print(str(np.int32(tracked.quad[3])))
		    #Este es el punto medio del poligono que genera. 
		    print((np.int32(tracked.quad[0]) + np.int32(tracked.quad[1]) + np.int32(tracked.quad[2]) + np.int32(tracked.quad[3]))/4)
                    cv2.polylines(vis, [np.int32(tracked.quad)], True, (255, 255, 255), 2)
                    for (x0, y0), (x1, y1) in zip(np.int32(tracked.p0), np.int32(tracked.p1)):
                        cv2.line(vis, (x0+w, y0), (x1, y1), (0, 255, 0))
            draw_keypoints(vis, self.tracker.frame_points)
	    
            self.rect_sel.draw(vis)
            cv2.imshow('plane', vis)
            ch = cv2.waitKey(1)
            if ch == ord(' '):
                self.paused = not self.paused
            if ch == 27:
                break
	    


if __name__ == '__main__':
    print(__doc__)

    import sys
    try:
        video_src = sys.argv[1]
    except:
        video_src = 0
    rospy.init_node('image_converter', anonymous=True)
    app = App(video_src)
    app.run()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()	
#    App(video_src).run()
