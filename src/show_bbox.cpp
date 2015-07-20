/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Asako Kanezaki <kanezaki@mi.t.u-tokyo.ac.jp>
 *  Tatsuya Harada <harada@mi.t.u-tokyo.ac.jp>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Intelligent Systems and Informatics Lab.
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/io/pcd_io.h>
#include <std_msgs/UInt16MultiArray.h>
#include <pcl/console/parse.h>

class ViewImage {
private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub1, _sub2;
  std_msgs::UInt16MultiArray bbox;
  int bbox_num_max;
public:
  ViewImage( int argc, char** argv ) : bbox_num_max( 10 ) {
    pcl::console::parse (argc, argv, "-n", bbox_num_max);
    _sub1 = _nh.subscribe ("/bbox", 1,  &ViewImage::bbox_cb, this);
    _sub2 = _nh.subscribe ("/image", 1,  &ViewImage::image_cb, this);
    ROS_INFO ("Listening for incoming data on topic /image ..." );
  }

  ~ViewImage() {}

  void bbox_cb( const std_msgs::UInt16MultiArrayConstPtr& msg ){
    bbox = *msg;
  }

  //* show color img
  void image_cb( const sensor_msgs::ImageConstPtr& msg ){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::Mat img = cv_ptr->image;
    const int bbox_num = std::min( bbox_num_max, (int)(bbox.data.size() / 4) );
    std::cout << "bbox_num: " << bbox_num << std::endl;
    for( int i = 0; i < bbox_num; ++i )
      cv::rectangle( img, cv::Point( bbox.data[ 4 * i ], bbox.data[ 4 * i + 2 ] ), cv::Point( bbox.data[ 4 * i + 1 ], bbox.data[ 4 * i + 3 ] ), cv::Scalar(0,0,255), 2, 2 );
    cv::imshow("img", img);
    cv::waitKey(3);
  }
  
};

int main( int argc, char** argv ){
  ros::init(argc,argv,"show_bbox");
  ViewImage view(argc,argv);
  ros::spin();

  return(0);
}
