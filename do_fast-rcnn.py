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

# Please change the following line.
fast_rcnn_path = '/home/kanezaki/git/fast-rcnn/'

import os.path as osp
import sys
caffe_path = osp.join(fast_rcnn_path, 'caffe-fast-rcnn', 'python')
lib_path = osp.join(fast_rcnn_path, 'lib')
if caffe_path not in sys.path:
    sys.path.insert(0, caffe_path)
if lib_path not in sys.path:
    sys.path.insert(0, lib_path)
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from utils.cython_nms import nms
from utils.timer import Timer
import numpy as np
import scipy.io as sio
import caffe, os, cv2
import argparse

import rospy
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

CLASSES = ('__background__',
           'aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor')

NETS = {'vgg16': ('VGG16',
                  'vgg16_fast_rcnn_iter_40000.caffemodel'),
        'vgg_cnn_m_1024': ('VGG_CNN_M_1024',
                           'vgg_cnn_m_1024_fast_rcnn_iter_40000.caffemodel'),
        'caffenet': ('CaffeNet',
                     'caffenet_fast_rcnn_iter_40000.caffemodel')}

class show_bbox:

  def __init__(self,args):
    cpu_mode = False
    gpu_id = 0;
    demo_det = 'vgg16'
    self.CONF_THRESH = 0.7
    self.NMS_THRESH = 0.3
    for i in range(0,len(args)):
      if (args[i].find('--gpu')==0):
        gpu_id = int(sys.argv[i+1])
      if (args[i].find('--cpu')==0):
        cpu_mode = True
      if (args[i].find('--net')==0):
        demo_net = sys.argv[i+1]
      if (args[i].find('--conf')==0):
        self.CONF_THRESH = float(sys.argv[i+1])
      if (args[i].find('--nms')==0):
        self.NMS_THRESH = float(sys.argv[i+1])

    prototxt = os.path.join(cfg.ROOT_DIR, 'models', NETS[demo_net][0],
                            'test.prototxt')
    caffemodel = os.path.join(cfg.ROOT_DIR, 'data', 'fast_rcnn_models',
                              NETS[demo_net][1])
    if not os.path.isfile(caffemodel):
      raise IOError(('{:s} not found.\nDid you run ./data/script/'
                     'fetch_fast_rcnn_models.sh?').format(caffemodel))
    if cpu_mode:
      caffe.set_mode_cpu()
    else:
      caffe.set_mode_gpu()
      caffe.set_device(gpu_id)
    self.net = caffe.Net(prototxt, caffemodel, caffe.TEST)
    print '\n\nLoaded network {:s}'.format(caffemodel)

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
    global cv_image

    # obj_proposals
    bbox_num = len(bbox.data) / 4;
    print "bbox_num:", bbox_num
    obj_proposals = np.array( [ [0 for i in range(0,4)] for j in range(0,bbox_num) ] )
    for i in range( 0, bbox_num ):
      obj_proposals[ i ][ 0 ] = bbox.data[ 4 * i ]
      obj_proposals[ i ][ 1 ] = bbox.data[ 4 * i + 2 ]
      obj_proposals[ i ][ 2 ] = bbox.data[ 4 * i + 1 ]
      obj_proposals[ i ][ 3 ] = bbox.data[ 4 * i + 3 ]
    
    ##############################################
    # Detect all object classes and regress object bounds
    timer = Timer()
    timer.tic()
    scores, boxes = im_detect(self.net, cv_image, obj_proposals)
    timer.toc()
    print ('Detection took {:.3f}s for '
           '{:d} object proposals').format(timer.total_time, boxes.shape[0])

    # insert predicted class label to each box
    labels = np.array([0 for i in range(0,boxes.shape[0])])
    for i in range(0,boxes.shape[0]):
        tmpscores = scores[i, :]
        labels[ i ] = np.argmax(tmpscores)
        
    # Visualize detections for each class
    output_image = cv_image
    for cls_ind in range(1,len(CLASSES)):
        if cls_ind not in labels:
            continue
        inds = np.where( labels == cls_ind )
        cls_boxes = boxes[inds, 4*cls_ind:4*(cls_ind + 1)]
        cls_scores = scores[inds, cls_ind]
        cls_boxes = cls_boxes[ 0 ]
        cls_scores = cls_scores[ 0 ]
        dets = np.hstack((cls_boxes,
                          cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, self.NMS_THRESH)
        dets = dets[keep, :]
        for i in range(0,dets.shape[0]):
            if dets[i, -1] > self.CONF_THRESH:
                bbox_ = dets[i, :4]
                print "      DETECTED! ", CLASSES[ cls_ind ], dets[i, :]
                cv2.rectangle(output_image, (bbox_[ 0 ], bbox_[ 1 ]),(bbox_[ 2 ], bbox_[ 3 ]),(0,0,255),2)
                cv2.putText(output_image,CLASSES[ cls_ind ],(bbox_[ 0 ], bbox_[ 1 ]),cv2.FONT_HERSHEY_COMPLEX, 1.,(0,0,255),2)

    cv2.imshow("Image window", output_image)
    cv2.waitKey(3)

if __name__ == '__main__':
  sb = show_bbox(sys.argv)
  rospy.init_node('show_bbox', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()
  
