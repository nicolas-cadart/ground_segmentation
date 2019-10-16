#ifndef GROUND_SEGMENTATION_SEGMENT_H_
#define GROUND_SEGMENTATION_SEGMENT_H_

#include <list>
#include <map>

#include "ground_segmentation/bin.h"

class Segment
{
public:
  typedef std::pair<Bin::MinZPoint, Bin::MinZPoint> Line;
  typedef std::pair<double, double> LocalLine;

private:
  // Parameters. Description in GroundSegmentation.
  const double maxSlope_;
  const double maxError_;
  const double longThreshold_;
  const double maxLongHeight_;
  const double maxStartHeight_;
  const double sensorHeight_;

  std::vector<Bin> bins_;
  std::list<Line> lines_;

  LocalLine fitLocalLine(const std::list<Bin::MinZPoint>& points);

  double getMeanError(const std::list<Bin::MinZPoint>& points, const LocalLine& line);

  double getMaxError(const std::list<Bin::MinZPoint>& points, const LocalLine& line);

  Line localLineToLine(const LocalLine& local_line, const std::list<Bin::MinZPoint>& line_points);

public:
  Segment(unsigned int n_bins, double max_slope, double max_error,
          double long_threshold, double max_long_height,
          double max_start_height, double sensor_height);

  double verticalDistanceToLine(double d, double z);

  void fitSegmentLines();

  bool getLines(std::list<Line>* lines);

  inline Bin& operator[](const size_t& index) { return bins_[index]; }
  inline std::vector<Bin>::iterator begin() { return bins_.begin(); }
  inline std::vector<Bin>::iterator end() { return bins_.end(); }
};

#endif /* GROUND_SEGMENTATION_SEGMENT_H_ */
