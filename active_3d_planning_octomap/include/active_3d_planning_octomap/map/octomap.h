#ifndef ACTIVE_3D_PLANNING_OCTOMAP_MAP_OCTOMAP_H_
#define ACTIVE_3D_PLANNING_OCTOMAP_MAP_OCTOMAP_H_

#include <memory>

#include <active_3d_planning_core/module/module_factory_registry.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>

#include "active_3d_planning_core/map/occupancy_map.h"

namespace active_3d_planning {
namespace map {

// Voxblox as a map representation
class OctoMap : public OccupancyMap {
 public:
  explicit OctoMap(PlannerI& planner);  // NOLINT

  // implement virtual methods
  void setupFromParamMap(Module::ParamMap* param_map) override;

  // check collision for a single pose
  bool isTraversable(const Eigen::Vector3d& position,
                     const Eigen::Quaterniond& orientation) override;

  // check whether point is part of the map
  bool isObserved(const Eigen::Vector3d& point) override;

  // get occupancy
  unsigned char getVoxelState(const Eigen::Vector3d& point) override;

  // get voxel size
  double getVoxelSize() override;

  // get the center of a voxel from input point
  bool getVoxelCenter(Eigen::Vector3d* center,
                      const Eigen::Vector3d& point) override;

  // accessor to the octree
  octomap::OcTree& getOctree();

 protected:
  static ModuleFactoryRegistry::Registration<OctoMap> registration;

  std::unique_ptr<octomap::OcTree&> octree_;

  // cache constants
  double c_voxel_size_;
};

}  // namespace map
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_OCTOMAP_MAP_OCTOMAP_H_
