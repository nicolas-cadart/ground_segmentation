#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include "ground_segmentation/ground_segmentation.h"

class SegmentationNode
{
  ros::Publisher ground_pub_;
  ros::Publisher obstacle_pub_;
  ros::Subscriber cloud_sub_;

  GroundSegmentationParams params_;

public:
  //----------------------------------------------------------------------------
  SegmentationNode(ros::NodeHandle& nh, const std::string& input_topic,
                   const std::string& ground_topic, const std::string& obstacle_topic,
                   const GroundSegmentationParams& params, const bool& latch = false)
    : params_(params)
  {
    // init ROS subscribers/publishers
    ground_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(ground_topic, 1, latch);
    obstacle_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(obstacle_topic, 1, latch);
    cloud_sub_ = nh.subscribe(input_topic, 1, &SegmentationNode::scanCallback, this);
  }

  //----------------------------------------------------------------------------
  void scanCallback(const pcl::PointCloud<pcl::PointXYZ>& cloud)
  {
    // run ground segmentation
    GroundSegmentation segmenter(params_);
    std::vector<int> labels;
    segmenter.segment(cloud, &labels);

    // fill output pointclouds
    pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
    ground_cloud.header = cloud.header;
    obstacle_cloud.header = cloud.header;
    for (size_t i = 0; i < cloud.size(); ++i)
    {
      if (labels[i] == 1)
        ground_cloud.push_back(cloud[i]);
      else
        obstacle_cloud.push_back(cloud[i]);
    }

    // send pointclouds
    ground_pub_.publish(ground_cloud);
    obstacle_pub_.publish(obstacle_cloud);
  }
};

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_segmentation");
  ros::NodeHandle nh("~");

  // Get ground segmentation parameters
  GroundSegmentationParams params;
  nh.param("visualize", params.visualize, params.visualize);
  nh.param("n_bins", params.n_bins, params.n_bins);
  nh.param("n_segments", params.n_segments, params.n_segments);
  nh.param("max_dist_to_line", params.max_dist_to_line, params.max_dist_to_line);
  nh.param("max_slope", params.max_slope, params.max_slope);
  nh.param("long_threshold", params.long_threshold, params.long_threshold);
  nh.param("max_long_height", params.max_long_height, params.max_long_height);
  nh.param("max_start_height", params.max_start_height, params.max_start_height);
  nh.param("sensor_height", params.sensor_height, params.sensor_height);
  nh.param("line_search_angle", params.line_search_angle, params.line_search_angle);
  nh.param("n_threads", params.n_threads, params.n_threads);
  nh.param("r_min", params.r_min, params.r_min);
  nh.param("r_max", params.r_max, params.r_max);

  // Params that need to be squared
  double max_fit_error;
  if (nh.getParam("max_fit_error", max_fit_error))
    params.maxError_square = max_fit_error * max_fit_error;

  // Get topics names
  std::string ground_topic, obstacle_topic, input_topic;
  bool latch;
  nh.param<std::string>("input_topic", input_topic, "velodyne_points");
  nh.param<std::string>("ground_output_topic", ground_topic, "ground_cloud");
  nh.param<std::string>("obstacle_output_topic", obstacle_topic, "obstacle_cloud");
  nh.param("latch", latch, false);

  // Start node
  SegmentationNode node(nh, input_topic, ground_topic, obstacle_topic, params, latch);
  ros::spin();
}
