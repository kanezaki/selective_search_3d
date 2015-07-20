# 
#  Software License Agreement (BSD License)
# 
#   Copyright (c) 2015, Asako Kanezaki <kanezaki@mi.t.u-tokyo.ac.jp>
#   Tatsuya Harada <harada@mi.t.u-tokyo.ac.jp>
#   All rights reserved.
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#      copyright notice, this list of conditions and the following
#      disclaimer in the documentation and/or other materials provided
#      with the distribution.
#    * Neither the name of Intelligent Systems and Informatics Lab.
#      nor the names of its contributors may be used to endorse or
#      promote products derived from this software without specific
#      prior written permission.
# 
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('selective_search_3d')
import sys
import rospy
import cv2
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.bbox_sub = rospy.Subscriber("/bbox",UInt16MultiArray,self.callback2,queue_size = 1)
    self.image_sub = rospy.Subscriber("image_topic",Image,self.callback1)

  def callback1(self,data):
    global cv_image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

  def callback2(self,bbox):
    global cv_image, bbox_num_max
    bbox_num = min( bbox_num_max, len(bbox.data) / 4 );
    print "bbox_num:", bbox_num
    for i in range( 0, bbox_num ):
      cv2.rectangle(cv_image, (bbox.data[ 4 * i ], bbox.data[ 4 * i + 2 ]),(bbox.data[ 4 * i + 1 ], bbox.data[ 4 * i + 3 ]),(0,0,255),2)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
  global bbox_num_max
  bbox_num_max = 10
  for i in range(0,len(sys.argv)):
    if (sys.argv[i].find('-n')==0):
      bbox_num_max = int(sys.argv[i+1])
      break
  main(sys.argv)
