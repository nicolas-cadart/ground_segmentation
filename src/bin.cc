#include "ground_segmentation/bin.h"

#include <limits>

Bin::Bin()
  : min_z(std::numeric_limits<double>::max())
  , min_z_range(std::numeric_limits<double>::max())
  , has_point_(false)
{}

Bin::Bin(const Bin& bin)
  : min_z(std::numeric_limits<double>::max())
  , min_z_range(std::numeric_limits<double>::max())
  , has_point_(false)
{}

void Bin::addPoint(const pcl::PointXYZ& point)
{
  const double range = sqrt(point.x * point.x + point.y * point.y);
  this->addPoint(range, point.z);
}

void Bin::addPoint(double d, double z)
{
  has_point_ = true;
  if (z < min_z)
  {
    min_z = z;
    min_z_range = d;
  }
}

PointDZ Bin::getMinZPoint()
{
  PointDZ point; // CHECK set defautlt coordinates to std::numeric_limits<double>::max() ?

  if (has_point_)
  {
    point.z = min_z;
    point.d = min_z_range;
  }

  return point;
}
