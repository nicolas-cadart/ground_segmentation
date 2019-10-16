#ifndef GROUND_SEGMENTATION_BIN_H_
#define GROUND_SEGMENTATION_BIN_H_

#include <atomic>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//------------------------------------------------------------------------------
struct PointDZ
{
  PointDZ()
    : z(0.)
    , d(0.)
  {}

  PointDZ(double distance, double height)
    : z(height)
    , d(distance)
  {}

  bool operator==(const PointDZ& comp) { return z == comp.z && d == comp.d; }

  double z;  ///< Z (height)
  double d;  ///< distance (range)
};

//------------------------------------------------------------------------------
class Bin
{
private:
  std::atomic<bool> has_point_;
  std::atomic<double> min_z;
  std::atomic<double> min_z_range;

public:
  Bin();

  /// \brief Fake copy constructor to allow vector<vector<Bin> > initialization.
  Bin(const Bin& bin);

  void addPoint(const pcl::PointXYZ& point);
  void addPoint(double d, double z);

  PointDZ getMinZPoint();

  inline bool hasPoint() { return has_point_; }
};

#endif /* GROUND_SEGMENTATION_BIN_H_ */
