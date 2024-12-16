#include "active_3d_planning_octomap/map/octomap.h"

#include "active_3d_planning_core/data/system_constraints.h"

namespace active_3d_planning {
namespace map {

ModuleFactoryRegistry::Registration<OctoMap> OctoMap::registration("OctoMap");

OctoMap::OctoMap(PlannerI& planner)
    : OccupancyMap(planner),
      c_voxel_size_(0.1) {  // Default voxel size is 0.1 meters
  octree_ = std::make_unique<octomap::OcTree>(c_voxel_size_);
}

octomap::Octree& OctoMap::getOctree() { return *octree_; }

void OctoMap::setupFromParamMap(Module::ParamMap* param_map) {
  // request a map from octomap server?
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  esdf_server_.reset(new voxblox::EsdfServer(nh, nh_private));
  esdf_server_->setTraversabilityRadius(
      planner_.getSystemConstraints().collision_radius);

  // cache constants
  c_voxel_size_ = esdf_server_->getEsdfMapPtr()->voxel_size();
}

bool OctoMap::isTraversable(const Eigen::Vector3d& position,
                            const Eigen::Quaterniond& orientation) {
  double distance = 0.0;
  if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(position,
                                                           &distance)) {
    // This means the voxel is observed
    return (distance > planner_.getSystemConstraints().collision_radius);
  }
  return false;
}

bool OctoMap::isObserved(const Eigen::Vector3d& point) {
  return esdf_server_->getEsdfMapPtr()->isObserved(point);
}

// get occupancy
unsigned char OctoMap::getVoxelState(const Eigen::Vector3d& point) {
  double distance = 0.0;
  if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) {
    // This means the voxel is observed
    if (distance < c_voxel_size_) {
      return OctoMap::OCCUPIED;
    } else {
      return OctoMap::FREE;
    }
  } else {
    return OctoMap::UNKNOWN;
  }
}

// get voxel size
double OctoMap::getVoxelSize() { return c_voxel_size_; }

// get the center of a voxel from input point
bool OctoMap::getVoxelCenter(Eigen::Vector3d* center,
                             const Eigen::Vector3d& point) {
  voxblox::BlockIndex block_id = esdf_server_->getEsdfMapPtr()
                                     ->getEsdfLayerPtr()
                                     ->computeBlockIndexFromCoordinates(
                                         point.cast<voxblox::FloatingPoint>());
  *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_)
                .cast<double>();
  voxblox::VoxelIndex voxel_id =
      voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
          (point - *center).cast<voxblox::FloatingPoint>(),
          1.0 / c_voxel_size_);
  *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_)
                 .cast<double>();
  return true;
}

}  // namespace map
}  // namespace active_3d_planning
