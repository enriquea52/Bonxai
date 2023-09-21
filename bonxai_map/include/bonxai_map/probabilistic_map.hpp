#pragma once

#include "bonxai/bonxai.hpp"
#include <eigen3/Eigen/Geometry>
#include <unordered_set>

namespace Bonxai
{

template <class Functor>
void RayIterator(const CoordT& key_origin,
                 const CoordT& key_end,
                 const Functor& func);

inline void ComputeRay(const CoordT& key_origin,
                       const CoordT& key_end,
                       std::vector<CoordT>& ray)
{
  ray.clear();
  RayIterator(key_origin, key_end, [&ray](const CoordT& coord)
              {
                ray.push_back(coord);
                return true;
              } );
}

/**
 * @brief The ProbabilisticMap class is meant to behave as much as possible as
 * octomap::Octree, given the same voxel size.
 *
 * Insert a point cloud to update the current probability
 */
class ProbabilisticMap
{
public:
  using Vector3D = Eigen::Vector3d;

  /// Compute the logds, but return the result as an integer,
  /// The real number is represented as a fixed precision
  /// integer (6 decimals after the comma)
  [[nodiscard]] static constexpr int32_t logods(float prob)
  {
    return int32_t(1e6 * std::log(prob / (1.0 - prob)));
  }

  /// Expect the fixed comma value returned by logods()
  [[nodiscard]] static constexpr float prob(int32_t logods_fixed)
  {
    float logods = float(logods_fixed) * 1e-6;
    return (1.0 - 1.0 / (1.0 + std::exp(logods)));
  }

  struct CellT {
    // variable used to check if a cell was already updated in this loop
    int32_t update_id : 4;
    // the probability of the cell to be occupied
    int32_t probability_log : 28;

    CellT(): update_id(0), probability_log(UnknownProbability) {};
  };

  /// These default values are the same as OctoMap
  struct Options {
    int32_t prob_miss_log = logods(0.4f);
    int32_t prob_hit_log = logods(0.7f);

    int32_t clamp_min_log = logods(0.12f);
    int32_t clamp_max_log = logods(0.97f);

    int32_t occupancy_threshold_log = logods(0.5);
  };

  static const int32_t UnknownProbability;

  ProbabilisticMap(double resolution);

  [[nodiscard]] VoxelGrid<CellT>& grid();

  [[nodiscard]] const VoxelGrid<CellT>& grid() const;

  [[nodiscard]] const Options& options() const;

  void setOptions(const Options& options);

  /**
   * @brief insertPointCloud will update the probability map
   * with a new set of detections.
   * The template function can accept points of different types,
   * such as pcl:Point, Eigen::Vector or Bonxai::Point3d
   *
   * Both origin and points must be in word coordinates
   *
   * @param points   a vector of points which represent detected obstacles
   * @param origin   origin of the point cloud
   * @param max_range  max range of the ray, if exceeded, we will use that
   * to compute a free space
   */
  template <typename PointT>
  void insertPointCloud(const std::vector<PointT>& points,
                        const PointT& origin,
                        double max_range);

  template <typename PointT>
  void insertPointCloud(const std::vector<PointT,  Eigen::aligned_allocator<PointT>>  &points,
                        const PointT &origin,
                        double max_range);


  /*
  Compute min x, y, z and max x, y, z
  This is useful for building a color map
  */ 
  template<typename PointT> inline
  void calcMinMax(std::vector<PointT>& occupied_voxels, 
                  double& min_x, double& min_y, double& min_z,
                  double& max_x,  double& max_y,  double& max_z);

  [[nodiscard]] bool isOccupied(const Bonxai::CoordT& coord) const;

  [[nodiscard]] bool isUnknown(const Bonxai::CoordT& coord) const;

  [[nodiscard]] bool isFree(const Bonxai::CoordT& coord) const;

  void getOccupiedVoxels(std::vector<Bonxai::CoordT>& coords);

  void getFreeVoxels(std::vector<Bonxai::CoordT>& coords);

  template <typename PointT>
  void getOccupiedVoxels(std::vector<PointT>& points)
  {
    thread_local std::vector<Bonxai::CoordT> coords;
    coords.clear();
    getOccupiedVoxels(coords);
    for(const auto& coord: coords)
    {
      const auto p = _grid.coordToPos(coord);
      points.emplace_back( p.x, p.y, p.z );
    }
  }

private:
  VoxelGrid<CellT> _grid;
  Options _options;
  uint8_t _update_count = 1;

  std::unordered_set<CoordT> _miss_coords;
  std::vector<CoordT> _hit_coords;

  Bonxai::VoxelGrid<CellT>::Accessor _accessor;

  void addPoint(const Vector3D& origin,
                const Vector3D& point,
                float max_range,
                float max_sqr);

  void updateFreeCells(const Vector3D& origin);
};

//--------------------------------------------------

template <typename PointT>
inline void ProbabilisticMap::insertPointCloud(const std::vector<PointT>& points,
                                               const PointT& origin,
                                               double max_range)
{
  const auto from = ConvertPoint<Vector3D>(origin);
  const double max_range_sqr = max_range * max_range;
  for (const auto& point : points)
  {
    const auto to = ConvertPoint<Vector3D>(point);
    addPoint(from, to, max_range, max_range_sqr);
  }
  updateFreeCells(from);
}

template <class Functor> inline
void RayIterator(const CoordT& key_origin,
                 const CoordT& key_end,
                 const Functor &func)
{
  if (key_origin == key_end)
  {
    return;
  }
  if(!func(key_origin))
  {
    return;
  }

  CoordT error = { 0, 0, 0 };
  CoordT coord = key_origin;
  CoordT delta = (key_end - coord);
  const CoordT step = { delta.x < 0 ? -1 : 1,
                        delta.y < 0 ? -1 : 1,
                        delta.z < 0 ? -1 : 1 };

  delta = { delta.x < 0 ? -delta.x : delta.x,
            delta.y < 0 ? -delta.y : delta.y,
            delta.z < 0 ? -delta.z : delta.z };

  const int max = std::max(std::max(delta.x, delta.y), delta.z);

          // maximum change of any coordinate
  for (int i = 0; i < max - 1; ++i)
  {
    // update errors
    error = error + delta;
    // manual loop unrolling
    if ((error.x << 1) >= max)
    {
      coord.x += step.x;
      error.x -= max;
    }
    if ((error.y << 1) >= max)
    {
      coord.y += step.y;
      error.y -= max;
    }
    if ((error.z << 1) >= max)
    {
      coord.z += step.z;
      error.z -= max;
    }
    if(!func(coord))
    {
      return;
    }
  }
}

template<typename PointT> inline
void ProbabilisticMap::insertPointCloud(const std::vector<PointT,  Eigen::aligned_allocator<PointT>>  &points,
                                        const PointT &origin,
                                        double max_range)
{
  Eigen::Vector3f fromf = origin.getVector3fMap();
  const auto from = fromf.cast<double>();
  const double max_range_sqr = max_range*max_range;
  for(const auto& point: points)
  {
    Eigen::Vector3f tof = point.getVector3fMap();
    const auto to = tof.cast<double>();
    addPoint(from, to, max_range, max_range_sqr);
  }
  updateFreeCells(from);
}

template<typename PointT> inline
void ProbabilisticMap::calcMinMax(std::vector<PointT>& occupied_voxels, 
                                  double& min_x, double& min_y, double& min_z,
                                  double& max_x,  double& max_y,  double& max_z) 
{

  double max_value[3], min_value[3];

  for (unsigned i = 0; i< 3; i++){
    max_value[i] = -std::numeric_limits<double>::max();
    min_value[i] = std::numeric_limits<double>::max();
  }

  for(const auto& node: occupied_voxels)
  {

    double x = node.x();
    double y = node.y();
    double z = node.z();
    if (x < min_value[0]) min_value[0] = x;
    if (y < min_value[1]) min_value[1] = y;
    if (z < min_value[2]) min_value[2] = z;

    if (x > max_value[0]) max_value[0] = x;
    if (y > max_value[1]) max_value[1] = y;
    if (z > max_value[2]) max_value[2] = z;

  }

  min_x = min_value[0]; min_y = min_value[1]; min_z = min_value[2];
  max_x = max_value[0]; max_y = max_value[1]; max_z = max_value[2];
}


}
