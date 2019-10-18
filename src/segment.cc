#include "ground_segmentation/segment.h"

Segment::Segment(unsigned int n_bins, double max_slope, double max_error,
                 double long_threshold, double max_long_height,
                 double max_start_height)
  : bins_(n_bins)
  , maxSlope_(max_slope)
  , maxError_(max_error)
  , longThreshold_(long_threshold)
  , maxLongHeight_(max_long_height)
  , maxStartHeight_(max_start_height)
{}

//------------------------------------------------------------------------------
/*!
 * @brief Build ground model for this segment by fitting lines.
 */
void Segment::fitSegmentLines()
{
  // Find first non empty bin.
  std::vector<Bin>::iterator line_start = bins_.begin();
  while (!line_start->hasPoint())
  {
    ++line_start;

    // Stop if all bins are empty.
    if (line_start == bins_.end())
      return;
  }
  
  // Fill lines.
  bool is_long_line = false;
  double cur_ground_height = 0;  // After transform, we assume that LiDAR is located as position (0, 0, 0).
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
 * @brief Build absolute coordinates line from local line paramters and points range.
 * @param[in] local_line Parameters (slope and intercept) of the line.
 * @param[in] line_points Points used to fit local_line, used here to find range definition.
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
 * @brief Compute vertical distance of a point to the closest line of the segment.
 * @param[in] d Range coordinate of the 2d point.
 * @param[in] z Heigth coordinate of the 2d point.
 * @return Distance of 2d point (d, z) to the segment.
 */
double Segment::verticalDistanceToLine(double d, double z)
{
  const double dTol = 0.2;  // [m] line validity range tolerance
  double distance = -1;  // TODO init to big value
  for (const Line& line : lines_)
  {
    // if point is in validity range of the line
    if (line.first.d - dTol < d && d < line.second.d + dTol)
    {
      const double deltaZ = line.second.z - line.first.z;
      const double deltaD = line.second.d - line.first.d;
      const double expectedZ = (d - line.first.d) / deltaD * deltaZ + line.first.z;
      distance = std::fabs(z - expectedZ);  // TODO update only if distance is smaller
    }
  }
  return distance;
}

//------------------------------------------------------------------------------
/*!
 * @brief Compute mean squared error of points to a line.
 * @param[in] points Array of points used to fit the line.
 * @param[in] line Fitted line to compare to points.
 * @return mean squared error to line.
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
 * @brief Compute max squared error of the worst fitted point to a line.
 * @param[in] points Array of points used to fit the line.
 * @param[in] line Fitted line to compare to points.
 * @return max squared error to line.
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
 * @brief Fit a line (slope & intercept) to a set of 2D points.
 * @param[in] points Points to fit with a line.
 * @return The fitted line.
 */
LocalLine Segment::fitLocalLine(const std::list<PointDZ>& points)
{
  const unsigned int n_points = points.size();
  Eigen::MatrixXd X(n_points, 2);
  Eigen::VectorXd Y(n_points);
  unsigned int counter = 0;
  for (const PointDZ& point : points)
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
 * @brief Return fitted lines if they exist.
 * @param[out] lines Array where to store fitted lines.
 * @return true if success, false otherwise.
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
