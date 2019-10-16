#include "ground_segmentation/segment.h"

Segment::Segment(unsigned int n_bins, double max_slope, double max_error,
                 double long_threshold, double max_long_height,
                 double max_start_height, double sensor_height)
  : bins_(n_bins)
  , maxSlope_(max_slope)
  , maxError_(max_error)
  , longThreshold_(long_threshold)
  , maxLongHeight_(max_long_height)
  , maxStartHeight_(max_start_height)
  , sensorHeight_(sensor_height)
{}

//------------------------------------------------------------------------------
/*!
 * @brief Build ground model for this segment by fitting lines.
 */
void Segment::fitSegmentLines()
{
  // Find first non empty bin.
  auto line_start = bins_.begin();
  while (!line_start->hasPoint())
  {
    ++line_start;

    // Stop if all bins are empty.
    if (line_start == bins_.end())
      return;
  }
  
  // Fill lines.
  bool is_long_line = false;
  double cur_ground_height = -sensorHeight_;
  std::list<PointDZ> current_line_points(1, line_start->getMinZPoint());
  LocalLine cur_line;

  for (auto line_iter = line_start + 1; line_iter != bins_.end(); ++line_iter)
  {
    // if non-empty bin
    if (line_iter->hasPoint())
    {
      PointDZ cur_point = line_iter->getMinZPoint();

      // check if current point is far from previous one
      if (cur_point.d - current_line_points.back().d > longThreshold_)
        is_long_line = true;

      // if current line already exists (from at least 2 points)
      if (current_line_points.size() >= 2)
      {
        // Get expected z value to possibly reject far away points.
        double expected_z = std::numeric_limits<double>::max();
        if (is_long_line && current_line_points.size() >= 2)  // CHECK > 2 condition
          expected_z = cur_line.slope * cur_point.d + cur_line.offset;

        // add current point to current line, and fit again line
        current_line_points.push_back(cur_point);
        cur_line = fitLocalLine(current_line_points);
        const double error = getMaxError(current_line_points, cur_line);

        // If not a good line, split current and begin new line
        if (error > maxError_ ||                                                   // too big fit error
            std::fabs(cur_line.slope) > maxSlope_ ||                               // too big slope
            is_long_line && std::fabs(expected_z - cur_point.z) > maxLongHeight_)  // CHECK too big height diff between last 2 points
        {
          // Remove current point
          current_line_points.pop_back();

          // Don't let lines with 2 base points through. CHECK
          if (current_line_points.size() >= 3)
          {
            const LocalLine new_line = fitLocalLine(current_line_points);
            lines_.push_back(localLineToLine(new_line, current_line_points));
            cur_ground_height = new_line.slope * current_line_points.back().d + new_line.offset;
          }

          // Start new line (keep only last point)
          is_long_line = false;
          current_line_points.erase(current_line_points.begin(), --current_line_points.end());
          --line_iter;
        }
      }

      // If current "line" contains only a single point, try to add current point or reset line
      else
      {
        if (cur_point.d - current_line_points.back().d < longThreshold_ &&  // if not too far from previous one
            std::fabs(current_line_points.back().z - cur_ground_height) < maxStartHeight_) // CHECK
        {
          // Add point if valid.
          current_line_points.push_back(cur_point);
        }
        else
        {
          // Start new line.
          current_line_points.clear();
          current_line_points.push_back(cur_point);
        }
      }
    }
  }

  // Add last line.
  if (current_line_points.size() > 2)
  {
    const LocalLine new_line = fitLocalLine(current_line_points);
    lines_.push_back(localLineToLine(new_line, current_line_points));
  }
}

//------------------------------------------------------------------------------
/*!
 * @brief
 */
Segment::Line Segment::localLineToLine(const LocalLine& local_line,
                                       const std::list<PointDZ>& line_points)
{
  Line line;
  const double first_d = line_points.front().d;
  const double second_d = line_points.back().d;
  const double first_z = local_line.slope * first_d + local_line.offset;
  const double second_z = local_line.slope * second_d + local_line.offset;
  line.first.z = first_z;
  line.first.d = first_d;
  line.second.z = second_z;
  line.second.d = second_d;
  return line;
}

//------------------------------------------------------------------------------
/*!
 * @brief
 */
double Segment::verticalDistanceToLine(double d, double z)
{
  const double kMargin = 0.1;  // CHECK use?
  double distance = -1;
  for (const auto& line : lines_)
  {
    if (line.first.d - kMargin < d && line.second.d + kMargin > d)
    {
      const double delta_z = line.second.z - line.first.z;
      const double delta_d = line.second.d - line.first.d;
      const double expected_z = (d - line.first.d) / delta_d * delta_z + line.first.z;
      distance = std::fabs(z - expected_z);
    }
  }
  return distance;
}

//------------------------------------------------------------------------------
/*!
 * @brief
 */
double Segment::getMeanError(const std::list<PointDZ>& points, const LocalLine& line)
{
  double error_sum = 0;
  for (const auto& point : points)
  {
    const double residual = (line.slope * point.d + line.offset) - point.z;
    error_sum += residual * residual;
  }
  return error_sum / points.size();
}

//------------------------------------------------------------------------------
/*!
 * @brief
 */
double Segment::getMaxError(const std::list<PointDZ>& points, const LocalLine& line)
{
  double max_error = 0;
  for (const auto& point : points)
  {
    const double residual = (line.slope * point.d + line.offset) - point.z;
    const double error = residual * residual;
    if (error > max_error)
      max_error = error;
  }
  return max_error;
}

//------------------------------------------------------------------------------
/*!
 * @brief
 */
LocalLine Segment::fitLocalLine(const std::list<PointDZ>& points)
{
  const unsigned int n_points = points.size();
  Eigen::MatrixXd X(n_points, 2);
  Eigen::VectorXd Y(n_points);
  unsigned int counter = 0;
  for (const auto& point : points)
  {
    X(counter, 0) = point.d;
    X(counter, 1) = 1;
    Y(counter) = point.z;
    ++counter;
  }
  Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
  LocalLine line_result;
  line_result.slope = result(0);
  line_result.offset = result(1);
  return line_result;
}

//------------------------------------------------------------------------------
/*!
 * @brief
 */
bool Segment::getLines(std::list<Line>* lines)
{
  if (lines_.empty())
  {
    return false;
  }
  else
  {
    *lines = lines_;
    return true;
  }
}
