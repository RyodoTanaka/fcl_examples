#ifndef __FCL_EXAMPLES_UTILS_H__
#define __FCL_EXAMPLES_UTILS_H__

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision_result.h>
#include <fcl/narrowphase/continuous_collision_object.h>
#include <fcl/narrowphase/continuous_collision_request.h>
#include <fcl/narrowphase/continuous_collision_result.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>

using namespace std;

namespace fcl{
namespace example {
// Default Functions
// Collision Function
template <typename S>
bool CollisionFunction(CollisionObject<S>* o1, CollisionObject<S>* o2, void* cdata_);
// Disctance Function
template <typename S>
bool DistanceFunction(CollisionObject<S>* o1, CollisionObject<S>* o2, void* cdata_, S& dist);

template <typename S>
struct CollisionData
{
  CollisionData() { done = false; }

  CollisionRequest<S> request;
  CollisionResult<S> result;

  bool done;
};

template <typename S>
struct DistanceData
{
  DistanceData() { done = false; }

  DistanceRequest<S> request;
  DistanceResult<S> result;
  bool done;
};

void generateEnvironments(vector< CollisionObject<double>* >& env);

void generateQueries(vector< CollisionObject<double>* >& req, const Eigen::Vector3d trans, const Eigen::Vector3d rot);

} // namespace example
} // namespace fcl

Eigen::Matrix3d setRPY(const Eigen::Vector3d rot)
{
  Eigen::Matrix3d ret;
  ret = Eigen::AngleAxisd(rot.x(), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(rot.y(), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rot.z(), Eigen::Vector3d::UnitZ());
  return ret;
}

namespace fe = fcl::example;

template <typename S>
bool fe::CollisionFunction(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2, void* cdata_)
{
  auto* cdata = static_cast<fe::CollisionData<S>*>(cdata_);
  const auto& request = cdata->request;
  auto& result = cdata->result;

  if(cdata->done) return true;

  collide(o1, o2, request, result);

  if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

template <typename S>
bool fe::DistanceFunction(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2, void* cdata_, S& dist)
{
  auto* cdata = static_cast<fe::DistanceData<S>*>(cdata_);
  const fcl::DistanceRequest<S>& request = cdata->request;
  fcl::DistanceResult<S>& result = cdata->result;
    
  if(cdata->done) { dist = result.min_distance; return true; }

  distance(o1, o2, request, result);

  dist = result.min_distance;

  if(dist <= 0) return true; // in collision or in touch

  return cdata->done;
}

#endif
