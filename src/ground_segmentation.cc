#include "ground_segmentation/ground_segmentation.h"

#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <thread>

//------------------------------------------------------------------------------
/*!
 * @brief Constructor.
 * @param[in] params Object parameters.
 */
GroundSegmentation::GroundSegmentation(const GroundSegmentationParams& params)
  : params_(params)
  , segments_(params.n_segments,
              Segment(params.n_bins, params.max_slope, params.maxError_square,
                      params.long_threshold, params.max_long_height, params.max_start_height,
                      params.sensor_height))
{
  if (params.visualize)
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
}

//------------------------------------------------------------------------------
/*!
 * @brief Main function, apply ground segmentation on input pointcloud.
 * @param[in] cloud         Pointcloud to process.
 * @param[out] segmentation Array of cloud size, containing label of each point.
 */
void GroundSegmentation::segment(const PointCloud& cloud, std::vector<int>* segmentation)
{
  // init timer
  std::cout << "Segmenting cloud with " << cloud.size() << " points...\n";
  std::chrono::duration<double, std::milli> fp_ms;
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

  // reset segmentation
  segmentation->clear();
  segmentation->resize(cloud.size(), 0);
  bin_index_.resize(cloud.size());
  segment_coordinates_.resize(cloud.size());

  // process
  insertPoints(cloud);
  fp_ms = std::chrono::high_resolution_clock::now() - start;
  double ellapsed = fp_ms.count();
  std::cout << "Pointcloud binning : " << ellapsed << " ms" << std::endl;

  std::list<PointLine> lines;
  if (params_.visualize)
    getLines(&lines);
  else
    getLines(nullptr);
  fp_ms = std::chrono::high_resolution_clock::now() - start;
  std::cout << "Lines fitting      : " << fp_ms.count() - ellapsed << " ms" << std::endl;
  ellapsed = fp_ms.count();

  assignCluster(segmentation);
  fp_ms = std::chrono::high_resolution_clock::now() - start;
  std::cout << "Cluster assignment : " << fp_ms.count() - ellapsed << " ms" << std::endl;
  ellapsed = fp_ms.count();

  // display segmentation
  if (params_.visualize)
  {
    // Visualize.
    PointCloud::Ptr obstacle_cloud(new PointCloud());
    // Get cloud of ground points.
    PointCloud::Ptr ground_cloud(new PointCloud());
    for (size_t i = 0; i < cloud.size(); ++i)
    {
      if (segmentation->at(i) == 1)
        ground_cloud->push_back(cloud[i]);
      else
        obstacle_cloud->push_back(cloud[i]);
    }
    PointCloud::Ptr min_cloud(new PointCloud());
    getMinZPointCloud(min_cloud.get());
    visualize(lines, min_cloud, ground_cloud, obstacle_cloud);
  }

  // display computation duration info
  fp_ms = std::chrono::high_resolution_clock::now() - start;
  std::cout << "Done! Total time   : " << fp_ms.count() << " ms" << std::endl << std::endl;
}

//------------------------------------------------------------------------------
/*!
 * @brief Launch lines fitting in several threads to compute ground model.
 * @param[out] lines Array where to store computed lines for visualization purpose.
 */
void GroundSegmentation::getLines(std::list<PointLine>* lines = nullptr)
{
  std::mutex line_mutex;
  std::vector<std::thread> threads(params_.n_threads);

  // start lineFit threads
  for (int i = 0; i < params_.n_threads; ++i)
  {
    const size_t start_index = params_.n_segments / params_.n_threads * i;
    const size_t end_index = std::min(params_.n_segments / params_.n_threads * (i + 1), params_.n_segments - 1);
    threads[i] = std::thread(&GroundSegmentation::lineFitThread, this, 
                             start_index, end_index, lines, &line_mutex);
  }

  // wait for threads to finish
  for (auto& thread : threads)
    thread.join();
}

//------------------------------------------------------------------------------
/*!
 * @brief Fit lines to ground model for a subset of segments.
 * @param[in] start_index The index of the first segment to process.
 * @param[in] end_index   The index of the last segment to process.
 * @param[out] lines      Array where to store computed lines for visualization purpose.
 * @param[in] lines_mutex Mutex used to protect access to lines.
 */
void GroundSegmentation::lineFitThread(const unsigned int start_index, const unsigned int end_index,
                                       std::list<PointLine>* lines, std::mutex* lines_mutex)
{
  const bool visualize = lines;
  const double segment_step = 2 * M_PI / params_.n_segments;
  double angle = -M_PI + segment_step * (start_index + 0.5);

  for(Segment& segment : segments_)
  {
    segment.fitSegmentLines();
    
    // Convert lines to 3d if we want to.
    if (visualize)
    {
      std::list<Segment::Line> segment_lines;
      segment.getLines(&segment_lines);
      for (const auto& line : segment_lines)
      {
        const pcl::PointXYZ start = minZPointTo3d(line.first, angle);
        const pcl::PointXYZ end = minZPointTo3d(line.second, angle);
        lines_mutex->lock();
        lines->emplace_back(start, end);
        lines_mutex->unlock();
      }

      angle += segment_step;
    }
  }
}

//------------------------------------------------------------------------------
/*!
 * @brief     
 * @param
 */
void GroundSegmentation::getMinZPointCloud(PointCloud* cloud)
{
  const double seg_step = 2 * M_PI / params_.n_segments;
  double angle = -M_PI + seg_step / 2;
  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter)
  {
    for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter)
    {
      const pcl::PointXYZ min = minZPointTo3d(bin_iter->getMinZPoint(), angle);
      cloud->push_back(min);
    }

    angle += seg_step;
  }
}

//------------------------------------------------------------------------------
/*!
 * @brief     
 * @param
 */
pcl::PointXYZ GroundSegmentation::minZPointTo3d(const PointDZ& min_z_point,
                                                const double& angle)
{
  pcl::PointXYZ point;
  point.x = cos(angle) * min_z_point.d;
  point.y = sin(angle) * min_z_point.d;
  point.z = min_z_point.z;
  return point;
}

//------------------------------------------------------------------------------
/*!
 * @brief      Launch threads that will assign a ground/non-ground label to each point from lines models.
 * @param[out] Array of same size as pointcloud where to store segmentation labels.
 */
void GroundSegmentation::assignCluster(std::vector<int>* segmentation)
{
  std::vector<std::thread> threads(params_.n_threads);
  const size_t cloud_size = segmentation->size();
  for (unsigned int i = 0; i < params_.n_threads; ++i)
  {
    const size_t start_index = cloud_size / params_.n_threads * i;
    const size_t end_index = std::min(cloud_size / params_.n_threads * (i + 1), cloud_size - 1);
    threads[i] = std::thread(&GroundSegmentation::assignClusterThread, this, 
                             start_index, end_index, segmentation);
  }

  // Wait for threads to finish.
  for (auto& thread : threads)
    thread.join();
}

//------------------------------------------------------------------------------
/*!
 * @brief Assign a ground/non-ground label to a subset of points from lines models.
 * @param[in] start_index   The index of the first point of the cloud to process.
 * @param[in] end_index     The index of the last point of the cloud to process.
 * @param[out] segmentation Array of same size as pointcloud where to store segmentation labels.
 */
void GroundSegmentation::assignClusterThread(const unsigned int& start_index,
                                             const unsigned int& end_index,
                                             std::vector<int>* segmentation)
{
  const double segment_step = 2 * M_PI / params_.n_segments;
  for (unsigned int i = start_index; i < end_index; ++i)
  {
    PointDZ& point_2d = segment_coordinates_[i];
    const int segment_index = bin_index_[i].first;

    // check if point is in range (otherwise segment_index = -1)
    if (segment_index >= 0)
    {
      // Search neighboring segments.
      double dist = segments_[segment_index].verticalDistanceToLine(point_2d.d, point_2d.z);
      int steps = 1;
      while (dist == -1 && steps * segment_step < params_.line_search_angle)
      {
        // Fix indices that are out of bounds.
        int index_1 = segment_index + steps;
        while (index_1 >= params_.n_segments)
          index_1 -= params_.n_segments;
        int index_2 = segment_index - steps;
        while (index_2 < 0)
          index_2 += params_.n_segments;
        
        // Get distance to neighboring lines.
        const double dist_1 = segments_[index_1].verticalDistanceToLine(point_2d.d, point_2d.z);
        const double dist_2 = segments_[index_2].verticalDistanceToLine(point_2d.d, point_2d.z);

        // Select larger distance if both segments return a valid distance. CHECK
        if (dist_1 > dist)
          dist = dist_1;
        if (dist_2 > dist)
          dist = dist_2;

        ++steps;
      }

      if (dist < params_.max_dist_to_line && dist != -1)
        segmentation->at(i) = 1;    
    }
  }
}

//------------------------------------------------------------------------------
/*!
 * @brief     
 * @param
 */
void GroundSegmentation::getMinZPoints(PointCloud* out_cloud)
{
  const double seg_step = 2 * M_PI / params_.n_segments;
  const double bin_step = (params_.r_max - params_.r_min) / params_.n_bins;
  double angle = -M_PI + seg_step / 2;
  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter)
  {
    double dist = params_.r_min + bin_step / 2;
    for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter)
    {
      pcl::PointXYZ point;
      if (bin_iter->hasPoint())
      {
        PointDZ min_z_point(bin_iter->getMinZPoint());
        point.x = cos(angle) * min_z_point.d;
        point.y = sin(angle) * min_z_point.d;
        point.z = min_z_point.z;

        out_cloud->push_back(point);
      }
      dist += bin_step;
    }
    angle += seg_step;
  }
}

//------------------------------------------------------------------------------
/*!
 * @brief     Start parallel threads to bin pointcloud into segments/bins.
 * @param[in] cloud The input pointcloud to process.
 */
void GroundSegmentation::insertPoints(const PointCloud& cloud)
{
  std::vector<std::thread> threads(params_.n_threads);
  const size_t points_per_thread = cloud.size() / params_.n_threads;

  // Launch threads.
  for (unsigned int i = 0; i < params_.n_threads; ++i)
  {
    const size_t start_index = i * points_per_thread;
    const size_t end_index = std::min((i + 1) * points_per_thread, cloud.size() - 1);
    threads[i] = std::thread(&GroundSegmentation::insertionThread, this, cloud, start_index, end_index);
  }

  // Wait for threads to finish.
  for (auto& thread : threads)
    thread.join();
}

//------------------------------------------------------------------------------
/*!
 * @brief     Compute each segment/bin index for each point.
 * @param[in] cloud       The input pointcloud to process.
 * @param[in] start_index The index of the first point of the cloud to process.
 * @param[in] end_index   The index of the last point of the cloud to process.
 */
void GroundSegmentation::insertionThread(const PointCloud& cloud, 
                                         const size_t start_index,
                                         const size_t end_index)
{
  // compute other params
  const double segment_step = 2 * M_PI / params_.n_segments;
  const double bin_step = (params_.r_max - params_.r_min) / params_.n_bins;

  // loop over selected points
  for (unsigned int i = start_index; i < end_index; ++i)
  {
    pcl::PointXYZ point(cloud[i]);
    const double range = sqrt(point.x * point.x + point.y * point.y);

    // if point is in range, bin it
    if (params_.r_min < range && range < params_.r_max)
    {
      const double angle = std::atan2(point.y, point.x);
      const unsigned int segment_index = (angle + M_PI) / segment_step;
      const unsigned int bin_index = (range - params_.r_min) / bin_step;
      segments_[segment_index][bin_index].addPoint(range, point.z);
      bin_index_[i] = std::make_pair(segment_index, bin_index);
    }
    // otherwise (too far or too close), ignore it
    else
      bin_index_[i] = std::pair<unsigned int, unsigned int>(-1, -1);

    segment_coordinates_[i] = PointDZ(range, point.z);
  }
}

//------------------------------------------------------------------------------
void GroundSegmentation::visualizePointCloud(const PointCloud::ConstPtr& cloud,
                                             const std::string& id)
{
  viewer_->addPointCloud(cloud, id, 0);
}

void GroundSegmentation::visualizeLines(const std::list<PointLine>& lines)
{
  size_t counter = 0;
  for (auto it = lines.begin(); it != lines.end(); ++it)
  {
    viewer_->addLine<pcl::PointXYZ>(it->first, it->second, std::to_string(counter++));
  }
}

void GroundSegmentation::visualize(const std::list<PointLine>& lines,
                                   const PointCloud::ConstPtr& min_cloud,
                                   const PointCloud::ConstPtr& ground_cloud,
                                   const PointCloud::ConstPtr& obstacle_cloud)
{
  viewer_->setBackgroundColor(0, 0, 0);
  viewer_->addCoordinateSystem(1.0);
  viewer_->initCameraParameters();
  viewer_->setCameraPosition(-2.0, 0, 2.0, 1.0, 0, 0);
  visualizePointCloud(min_cloud, "min_cloud");
  viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f,
                                            0.0f, "min_cloud");
  viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0f,
                                            "min_cloud");
  visualizePointCloud(ground_cloud, "ground_cloud");
  viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f,
                                            0.0f, "ground_cloud");
  visualizePointCloud(obstacle_cloud, "obstacle_cloud");
  visualizeLines(lines);
  while (!viewer_->wasStopped())
  {
    viewer_->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}
