#ifndef GROUND_SEGMENTATION_H_
#define GROUND_SEGMENTATION_H_

#include <mutex>

#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "ground_segmentation/segment.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef std::pair<pcl::PointXYZ, pcl::PointXYZ> PointLine;  // used only for debug visualization

//------------------------------------------------------------------------------
struct GroundSegmentationParams
{
  GroundSegmentationParams()
    : visualize(false)
    , r_min(1.)
    , r_max(50.)
    , n_bins(30)
    , n_segments(180)
    , max_dist_to_line(0.15)
    , max_slope(1)
    , n_threads(4)
    , maxError_square(0.01)
    , long_threshold(2.0)
    , max_long_height(0.1)
    , max_start_height(0.2)
    , sensor_height(0.2)
    , line_search_angle(0.2)
  {}

  bool visualize;            ///< Visualize estimated ground.
  double r_min;              ///< Minimum range of segmentation.
  double r_max;              ///< Maximum range of segmentation.
  int n_bins;                ///< Number of radial bins.
  int n_segments;            ///< Number of angular segments.
  double max_dist_to_line;   ///< Maximum distance to a ground line to be classified as ground.
  double max_slope;          ///< Max slope to be considered ground line.
  double maxError_square;    ///< Max error for line fit.
  double long_threshold;     ///< Distance at which points are considered far from each other.
  double max_long_height;    ///< Maximum slope for
  double max_start_height;   ///< Maximum heigh of starting line to be labelled ground.
  double sensor_height;      ///< Height of sensor above ground.
  double line_search_angle;  ///< How far to search for a line in angular direction [rad].
  int n_threads;             ///< Number of threads.
};

//------------------------------------------------------------------------------
struct PolarIndex
{
  PolarIndex()
    : segment(std::numeric_limits<unsigned int>::max())
    , bin(std::numeric_limits<unsigned int>::max())
    {}

  PolarIndex(unsigned int segment_index, unsigned int bin_index)
    : segment(segment_index)
    , bin(bin_index)
    {}

  inline bool is_valid() const { return segment != std::numeric_limits<unsigned int>::max() ||
                                        bin != std::numeric_limits<unsigned int>::max(); }

  unsigned int segment;
  unsigned int bin;
};

//------------------------------------------------------------------------------
class GroundSegmentation
{
  const GroundSegmentationParams params_;
  
  std::vector<Segment> segments_;        ///< Access with segments_[segment][bin]
  std::vector<PolarIndex> polar_index_;  ///< Segment & bin index of every point
  std::vector<PointDZ> coordinates_2d_;  ///< 2D coordinates (d, z) of every point in its respective segment

  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;  ///< Visualizer

  // ----- main internal routines -----

  void insertPoints(const PointCloud& cloud);

  void insertPointsThread(const PointCloud& cloud, const size_t start_index, const size_t end_index);

  void fitGroundLines(std::list<PointLine>* lines);

  void fitGroundLinesThread(const unsigned int start_index, const unsigned int end_index,
                            std::list<PointLine>* lines, std::mutex* lines_mutex);

  void assignCluster(std::vector<int>* segmentation);

  void assignClusterThread(const unsigned int& start_index, const unsigned int& end_index,
                           std::vector<int>* segmentation);

  // ----- visualization routines -----

  pcl::PointXYZ minZPointTo3d(const PointDZ& min_z_point, const double& angle);

  void getMinZPointCloud(PointCloud* cloud);

  void visualizePointCloud(const PointCloud::ConstPtr& cloud,
                           const std::string& id = "point_cloud");

  void visualizeLines(const std::list<PointLine>& lines);

  void visualize(const std::list<PointLine>& lines, const PointCloud::ConstPtr& cloud,
                 const PointCloud::ConstPtr& ground_cloud,
                 const PointCloud::ConstPtr& obstacle_cloud);

public:

  // ----- user interface -----

  GroundSegmentation(const GroundSegmentationParams& params = GroundSegmentationParams());

  void segment(const PointCloud& cloud, std::vector<int>* segmentation);
};

#endif // GROUND_SEGMENTATION_H_
