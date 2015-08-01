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
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloudT;

class SP{
public:
  int label;
  int label_ID_parent; // Not label.
  int num;
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float min_z;
  float max_z;
  int min_im_x;
  int max_im_x;
  int min_im_y;
  int max_im_y;
  float nx;
  float ny;
  float nz;
  std::vector<int> hist_r;
  std::vector<int> hist_g;
  std::vector<int> hist_b;
  float sim;
  SP():label_ID_parent(-1),sim(0){
    hist_r.resize( 25 );
    hist_g.resize( 25 );
    hist_b.resize( 25 );
  };
  ~SP(){};
};

float calcSim( SP sp1, SP sp2, const float volume_all ){
  float sim = 0;

  // // Normal
  // sim += std::max( 0., double(sp1.nx * sp2.nx + sp1.ny * sp2.ny + sp1.nz * sp2.nz) ); // larger than 0.

  // Size
  const float min_x = std::min( sp1.min_x, sp2.min_x );
  const float max_x = std::max( sp1.max_x, sp2.max_x );
  const float min_y = std::min( sp1.min_y, sp2.min_y );
  const float max_y = std::max( sp1.max_y, sp2.max_y );
  const float min_z = std::min( sp1.min_z, sp2.min_z );
  const float max_z = std::max( sp1.max_z, sp2.max_z );
  const float volume = (max_x-min_x) * (max_y-min_y) * (max_z-min_z);
  sim += 1 - volume / volume_all;

  // Color Hist.
  for( size_t i = 0; i < 25; ++i ){
    sim += std::min( sp1.hist_r[ i ] / (float)sp1.num, sp2.hist_r[ i ] / (float)sp2.num );
    sim += std::min( sp1.hist_g[ i ] / (float)sp1.num, sp2.hist_g[ i ] / (float)sp2.num );
    sim += std::min( sp1.hist_b[ i ] / (float)sp1.num, sp2.hist_b[ i ] / (float)sp2.num );
  }

  return sim;
}

bool myFunc( const std::pair< std::pair< int, int >, float > &a, const std::pair< std::pair< int, int >, float > &b ){
  return ( a.second > b.second );
}

class SelectiveSearch3D {
private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub;
  pcl::PointCloud<PointT> input_cloud;
  std_msgs::UInt16MultiArray msg1;
  std_msgs::Float32MultiArray msg2;
  ros::Publisher pub1;
  ros::Publisher pub2;
  std::vector< SP > sps;
  float voxel_resolution;
  float seed_resolution;
  float color_importance;
  float spatial_importance;
  float normal_importance;
  float depth_limit; // meter.

public:
  SelectiveSearch3D( int argc, char** argv ) :
    voxel_resolution ( 0.008f ),
    seed_resolution ( 0.08f ),
    color_importance ( 0.2f ),
    spatial_importance ( 0.4f ),
    normal_importance ( 1.0f ),
    depth_limit ( 1.5f )
  {
    pcl::console::parse (argc, argv, "-v", voxel_resolution);
    pcl::console::parse (argc, argv, "-s", seed_resolution);
    pcl::console::parse (argc, argv, "-c", color_importance);
    pcl::console::parse (argc, argv, "-z", spatial_importance);
    pcl::console::parse (argc, argv, "-n", normal_importance);
    pcl::console::parse (argc, argv, "-d", depth_limit);
    _sub = _nh.subscribe ("/points", 1,  &SelectiveSearch3D::points_cb, this);
    ROS_INFO ("Listening for incoming data on topic /points ..." );
    pub1 = _nh.advertise<std_msgs::UInt16MultiArray>("bbox", 1);
    pub2 = _nh.advertise<std_msgs::Float32MultiArray>("bbox3D", 1);
  }

  ~SelectiveSearch3D() {}

  void publish_zero_bbox(){
    msg1.data.resize( 1 );
    msg1.data[ 0 ] = 0;
    pub1.publish(msg1);
    msg2.data.resize( 1 );
    msg2.data[ 0 ] = 0;
    pub2.publish(msg2);
  }

  void points_cb( const sensor_msgs::PointCloud2ConstPtr& cloud_ ){
    /////////////////////////
    //// get point cloud //// 
    /////////////////////////
    
    if ((cloud_->width * cloud_->height) == 0){
      publish_zero_bbox();
      return;
    }
    pcl::fromROSMsg (*cloud_, input_cloud);

    PointCloudT::Ptr cloud = boost::make_shared < PointCloudT >();
    cloud->width = cloud_->width;
    cloud->height = cloud_->height;
    cloud->points = input_cloud.points;

    //* limit depth
    for (PointCloudT::iterator cloud_itr = cloud->begin (); cloud_itr != cloud->end (); ++cloud_itr)
      if (cloud_itr->z > depth_limit)
	cloud_itr->z = NAN;

    /////////////////////
    //// supervoxels //// 
    /////////////////////
    
    // If we're using the single camera transform no negative z allowed since we use log(z)
    bool disable_transform = false;
    // if (!disable_transform)
    //   {
    // 	for (PointCloudT::iterator cloud_itr = cloud->begin (); cloud_itr != cloud->end (); ++cloud_itr)
    // 	  if (cloud_itr->z < 0)
    // 	    {
    // 	      PCL_ERROR ("Points found with negative Z values, this is not compatible with the single camera transform!\n");
    // 	      PCL_ERROR ("Set the --NT option to disable the single camera transform!\n");
    // 	      publish_zero_bbox();
    // 	      return ;
    // 	    }
    // 	std::cout <<"You have the single camera transform enabled - this should be used with point clouds captured from a single camera.\n";
    // 	std::cout <<"You can disable the transform with the --NT flag\n";    
    //   }
    
    pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution,!disable_transform);
    super.setInputCloud (cloud);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);
    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
    
    std::cout << "Extracting supervoxels!\n";
    super.extract (supervoxel_clusters);
    std::cout << "Found " << supervoxel_clusters.size () << " Supervoxels!\n";
    if( supervoxel_clusters.size () < 10 ){
      publish_zero_bbox();
      return;
    }
    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > refined_supervoxel_clusters;
    std::cout << "Refining supervoxels \n";
    super.refineSupervoxels (3, refined_supervoxel_clusters);
    if( refined_supervoxel_clusters.size () < 10 ){
      publish_zero_bbox();
      return;
    }

    PointLCloudT::Ptr refined_labeled_voxel_cloud = super.getLabeledVoxelCloud ();
    PointNCloudT::Ptr refined_sv_normal_cloud = super.makeSupervoxelNormalCloud (refined_supervoxel_clusters);
    PointLCloudT::Ptr refined_full_labeled_cloud = super.getLabeledCloud ();
    
    std::cout << "Getting supervoxel adjacency\n";
    std::multimap<uint32_t, uint32_t> label_adjacency;
    super.getSupervoxelAdjacency (label_adjacency);
    
    /////////////////////////////
    //// 3D selective search //// 
    /////////////////////////////
    
    int max_label = -1;
    for( size_t i = 0; i < refined_labeled_voxel_cloud->points.size(); ++i )
      if( max_label < (int)refined_labeled_voxel_cloud->points[ i ].label )
    	max_label = (int)refined_labeled_voxel_cloud->points[ i ].label;

    std::vector<int> label_num( max_label+1, 0 );
    std::vector< std::vector<int> > label_colors_R( max_label+1, std::vector<int>( 25, 0 ) );
    std::vector< std::vector<int> > label_colors_G( max_label+1, std::vector<int>( 25, 0 ) );
    std::vector< std::vector<int> > label_colors_B( max_label+1, std::vector<int>( 25, 0 ) );
    std::vector< float > label_bbox_X_min( max_label+1, std::numeric_limits<float>::max() );
    std::vector< float > label_bbox_X_max( max_label+1, -std::numeric_limits<float>::max() );
    std::vector< float > label_bbox_Y_min( max_label+1, std::numeric_limits<float>::max() );
    std::vector< float > label_bbox_Y_max( max_label+1, -std::numeric_limits<float>::max() );
    std::vector< float > label_bbox_Z_min( max_label+1, std::numeric_limits<float>::max() );
    std::vector< float > label_bbox_Z_max( max_label+1, -std::numeric_limits<float>::max() );
    std::vector< int > label_bbox_x_min( max_label+1, std::numeric_limits<int>::max() );
    std::vector< int > label_bbox_x_max( max_label+1, -std::numeric_limits<int>::max() );
    std::vector< int > label_bbox_y_min( max_label+1, std::numeric_limits<int>::max() );
    std::vector< int > label_bbox_y_max( max_label+1, -std::numeric_limits<int>::max() );
    int idx = 0;
    int r, g, b, color;
    for( size_t j = 0; j < cloud->height; ++j ){
      for( size_t i = 0; i < cloud->width; ++i ){
	int l = refined_full_labeled_cloud->points[ idx ].label;
	label_num[ l ]++;
	color = *reinterpret_cast<const int*>(&(cloud->points[idx].rgb));
	r = (0xff0000 & color) >> 16;
	g = (0x00ff00 & color) >> 8;
	b =  0x0000ff & color;
	label_colors_R[ l ][ std::min( 24, (int)floor( (r - 3) / 10 ) ) ]++;
	label_colors_G[ l ][ std::min( 24, (int)floor( (g - 3) / 10 ) ) ]++;
	label_colors_B[ l ][ std::min( 24, (int)floor( (b - 3) / 10 ) ) ]++;
	label_bbox_X_min[ l ] = std::min( label_bbox_X_min[ l ], cloud->points[idx].x );
	label_bbox_X_max[ l ] = std::max( label_bbox_X_max[ l ], cloud->points[idx].x );
	label_bbox_Y_min[ l ] = std::min( label_bbox_Y_min[ l ], cloud->points[idx].y );
	label_bbox_Y_max[ l ] = std::max( label_bbox_Y_max[ l ], cloud->points[idx].y );
	label_bbox_Z_min[ l ] = std::min( label_bbox_Z_min[ l ], cloud->points[idx].z );
	label_bbox_Z_max[ l ] = std::max( label_bbox_Z_max[ l ], cloud->points[idx].z );
	label_bbox_x_min[ l ] = std::min( label_bbox_x_min[ l ], (int)i );
	label_bbox_x_max[ l ] = std::max( label_bbox_x_max[ l ], (int)i );
	label_bbox_y_min[ l ] = std::min( label_bbox_y_min[ l ], (int)j );
	label_bbox_y_max[ l ] = std::max( label_bbox_y_max[ l ], (int)j );
	idx++;
      }
    }
    
    sps.resize( 0 );
    SP tmp;
    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator sv_itr = refined_supervoxel_clusters.begin();
    for( size_t i = 0; i < refined_sv_normal_cloud->points.size(); ++i ){
      size_t l = (sv_itr++)->first;
      if( (label_num[ l ] == 0) || (std::isnan(refined_sv_normal_cloud->points[ i ].normal_x)) ){ // invalid superpixel!
	for( std::multimap<uint32_t, uint32_t>::iterator itr = label_adjacency.begin(); itr != label_adjacency.end(); ++itr )
	  if( (itr->first == l) || (itr->second == l) ){
	    label_adjacency.erase( itr );
	    itr--;
	  }
	continue;
      }
      tmp.label = l;
      tmp.num = label_num[ l ];
      tmp.min_x = label_bbox_X_min[ l ];
      tmp.max_x = label_bbox_X_max[ l ];
      tmp.min_y = label_bbox_Y_min[ l ];
      tmp.max_y = label_bbox_Y_max[ l ];
      tmp.min_z = label_bbox_Z_min[ l ];
      tmp.max_z = label_bbox_Z_max[ l ];
      //tmp.nx = refined_sv_normal_cloud->points[ i ].normal_x;
      //tmp.ny = refined_sv_normal_cloud->points[ i ].normal_y;
      //tmp.nz = refined_sv_normal_cloud->points[ i ].normal_z;
      tmp.min_im_x = label_bbox_x_min[ l ];
      tmp.max_im_x = label_bbox_x_max[ l ];
      tmp.min_im_y = label_bbox_y_min[ l ];
      tmp.max_im_y = label_bbox_y_max[ l ];
      for( size_t j = 0; j < 25; ++j ){
	tmp.hist_r[ j ] = label_colors_R[ l ][ j ];
	tmp.hist_g[ j ] = label_colors_G[ l ][ j ];
	tmp.hist_b[ j ] = label_colors_B[ l ][ j ];
      }
      sps.push_back( tmp );
    }

    //* volume_all
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    for( size_t i = 0; i < sps.size(); ++i ){
      min_x = std::min( min_x, sps[ i ].min_x );
      max_x = std::max( max_x, sps[ i ].max_x );
      min_y = std::min( min_y, sps[ i ].min_y );
      max_y = std::max( max_y, sps[ i ].max_y );
      min_z = std::min( min_z, sps[ i ].min_z );
      max_z = std::max( max_z, sps[ i ].max_z );
    }
    const float volume_all = (max_x-min_x) * (max_y-min_y) * (max_z-min_z);
    
    //* labels, label2ID
    std::vector< std::vector<int> > labels( sps.size() );
    std::vector< int > label2ID( max_label + 1 );
    for( size_t i = 0; i < sps.size(); ++i ){
      labels[ i ].push_back( sps[ i ].label );
      label2ID[ sps[ i ].label ] = i;
    }
  
    //* neighbors
    std::vector< std::pair< std::pair< int, int >, float > > neighbors;
    std::pair< std::pair< int, int >, float > neighbor;
    for (std::multimap<uint32_t, uint32_t>::iterator itr = label_adjacency.begin (); itr != label_adjacency.end (); ++itr ){
      neighbor.first.first = itr->first;
      neighbor.first.second = itr->second;
      if( neighbor.first.first < neighbor.first.second ){
	neighbor.second = calcSim( sps[ label2ID[ neighbor.first.first ] ], sps[ label2ID[ neighbor.first.second ] ], volume_all );
	neighbors.push_back( neighbor );
      }
    }

    //* processing
    int id1, id2;
    while( neighbors.size() != 0 ){
      std::sort( neighbors.begin(), neighbors.end(), myFunc );
      
      // GATTAI!!!
      tmp.label = ++max_label;
      id1 = label2ID[ neighbors[0].first.first ];
      id2 = label2ID[ neighbors[0].first.second ];
      tmp.num = sps[ id1 ].num + sps[ id2 ].num;
      tmp.min_x = std::min( sps[ id1 ].min_x, sps[ id2 ].min_x );
      tmp.max_x = std::max( sps[ id1 ].max_x, sps[ id2 ].max_x );
      tmp.min_y = std::min( sps[ id1 ].min_y, sps[ id2 ].min_y );
      tmp.max_y = std::max( sps[ id1 ].max_y, sps[ id2 ].max_y );
      tmp.min_z = std::min( sps[ id1 ].min_z, sps[ id2 ].min_z );
      tmp.max_z = std::max( sps[ id1 ].max_z, sps[ id2 ].max_z );
      tmp.min_im_x = std::min( sps[ id1 ].min_im_x, sps[ id2 ].min_im_x );
      tmp.max_im_x = std::max( sps[ id1 ].max_im_x, sps[ id2 ].max_im_x );
      tmp.min_im_y = std::min( sps[ id1 ].min_im_y, sps[ id2 ].min_im_y );
      tmp.max_im_y = std::max( sps[ id1 ].max_im_y, sps[ id2 ].max_im_y );
      tmp.nx = ( sps[ id1 ].num * sps[ id1 ].nx + sps[ id2 ].num * sps[ id2 ].nx ) / tmp.num;
      tmp.ny = ( sps[ id1 ].num * sps[ id1 ].ny + sps[ id2 ].num * sps[ id2 ].ny ) / tmp.num;
      tmp.nz = ( sps[ id1 ].num * sps[ id1 ].nz + sps[ id2 ].num * sps[ id2 ].nz ) / tmp.num;
      for( size_t i = 0; i < 25; ++i ){
	tmp.hist_r[ i ] = sps[ id1 ].hist_r[ i ] + sps[ id2 ].hist_r[ i ];
	tmp.hist_g[ i ] = sps[ id1 ].hist_g[ i ] + sps[ id2 ].hist_g[ i ];
	tmp.hist_b[ i ] = sps[ id1 ].hist_b[ i ] + sps[ id2 ].hist_b[ i ];
      }
      sps[ id1 ].sim = - neighbors[0].second;
      sps[ id2 ].sim = - neighbors[0].second;
      sps[ id1 ].label_ID_parent = sps.size();
      sps[ id2 ].label_ID_parent = sps.size();
      
      sps.push_back( tmp );
      label2ID.push_back( sps.size() - 1 );
      labels.push_back( labels[ id1 ] );
      for( size_t i = 0; i < labels[ id2 ].size(); ++i )
	labels[ labels.size()-1 ].push_back( labels[ id2 ][ i ] );
      
      // UPDATE 
      std::vector<bool> updated_flg( label2ID.size() );
      for( size_t i = 0; i < label2ID.size(); ++i ) updated_flg[ i ] = false;
      for( size_t i = 1; i < neighbors.size(); ++i ){
	if( (neighbors[ i ].first.first == neighbors[0].first.first) || (neighbors[ i ].first.first == neighbors[0].first.second) ){
	  if( !updated_flg[ neighbors[ i ].first.second ] ){
	    neighbors[ i ].first.first = tmp.label;
	    neighbors[ i ].second = calcSim( sps[ label2ID[ neighbors[ i ].first.first ] ], sps[ label2ID[ neighbors[ i ].first.second ] ], volume_all );
	    updated_flg[ neighbors[ i ].first.second ] = true;
	  }
	  else{ // REMOVE!
	    neighbors.erase( neighbors.begin() + i );
	    --i;
	  }
	}
	else if( (neighbors[ i ].first.second == neighbors[0].first.first) || (neighbors[ i ].first.second == neighbors[0].first.second) ){
	  if( !updated_flg[ neighbors[ i ].first.first ] ){
	    neighbors[ i ].first.second = tmp.label;
	    neighbors[ i ].second = calcSim( sps[ label2ID[ neighbors[ i ].first.first ] ], sps[ label2ID[ neighbors[ i ].first.second ] ], volume_all );
	    updated_flg[ neighbors[ i ].first.first ] = true;
	  }
	  else{ // REMOVE!
	    neighbors.erase( neighbors.begin() + i );
	    --i;
	  }
	}
      }
      neighbors.erase( neighbors.begin() ); // !!!
    }
    
    //* NMS
    std::vector<uint16_t> bbox( 0 );
    std::vector<float> bbox3D( 0 );
    for( int i = sps.size()-1; i > -1; --i ){
      int j = sps[ i ].label_ID_parent;
      bool skip_flg = false;
      while( j != -1 ){
	if( sps[ i ].num / sps[ j ].num > 0.5 ){
	  skip_flg = true;
	  break;
	}
	j = sps[ j ].label_ID_parent;
      }
      if( !skip_flg ){
	bbox.push_back( sps[ i ].min_im_x );
	bbox.push_back( sps[ i ].max_im_x );
	bbox.push_back( sps[ i ].min_im_y );
	bbox.push_back( sps[ i ].max_im_y );
	bbox3D.push_back( sps[ i ].min_x );
	bbox3D.push_back( sps[ i ].max_x );
	bbox3D.push_back( sps[ i ].min_y );
	bbox3D.push_back( sps[ i ].max_y );
	bbox3D.push_back( sps[ i ].min_z );
	bbox3D.push_back( sps[ i ].max_z );
      }
    }

    //* publish
    msg1.data.resize( bbox.size() );
    for( size_t i = 0; i < bbox.size(); ++i )
      msg1.data[ i ] = bbox[ i ];
    pub1.publish(msg1);
    msg2.data.resize( bbox3D.size() );
    for( size_t i = 0; i < bbox3D.size(); ++i )
      msg2.data[ i ] = bbox3D[ i ];
    pub2.publish(msg2);
    std::cout << "Published " << bbox.size() / 4 << " boxes." << std::endl;
    std::cout << "------------------------------------------" << std::endl;

  }

};

int main( int argc, char** argv ){
  ros::init(argc,argv,"selective_search_3d");
  SelectiveSearch3D ss(argc,argv);
  ros::spin();

  return(0);
}
