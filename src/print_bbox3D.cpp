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
#include <std_msgs/Float32MultiArray.h>
#include <pcl/console/parse.h>

class PrintBbox {
private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub;
  int bbox_num_max;
public:
  PrintBbox( int argc, char** argv ) : bbox_num_max( 10 ) {
    pcl::console::parse (argc, argv, "-n", bbox_num_max);
    _sub = _nh.subscribe ("/bbox3D", 1,  &PrintBbox::bbox_cb, this);
    ROS_INFO ("Listening for incoming data on topic /bbox3D ..." );
  }

  ~PrintBbox() {}

  void bbox_cb( const std_msgs::Float32MultiArrayConstPtr& msg ){
    std_msgs::Float32MultiArray bbox = *msg;
    const int bbox_num = std::min( bbox_num_max, (int)(bbox.data.size() / 6) );
    std::cout << "bbox_num: " << bbox_num << std::endl;
    for( int i = 0; i < bbox_num; ++i )
      printf( "(%d) x: (%.3f, %.3f), y: (%.3f, %.3f), z: (%.3f, %.3f)\n", i, bbox.data[ 6 * i ], bbox.data[ 6 * i + 1 ], bbox.data[ 6 * i + 2 ], bbox.data[ 6 * i + 3 ], bbox.data[ 6 * i + 4 ], bbox.data[ 6 * i + 5 ] );
  }
  
};

int main( int argc, char** argv ){
  ros::init(argc,argv,"print_bbox3D");
  PrintBbox pb(argc,argv);
  ros::spin();

  return(0);
}
