#include "utils.h"

#include <cmath>

using namespace std;
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

template <typename S>
void fe::generateEnvironments(vector<shared_ptr<fcl::CollisionObject<S>>>& env)
{
  fcl::Vector3d trans;
  fcl::Vector3d rot;

  // Box
  shared_ptr<fcl::CollisionGeometry<S>> box_geometry =
      make_shared<fcl::Box<S>>(10.,2.,2.);
  shared_ptr<fcl::CollisionObject<S>> box =
      make_shared<fcl::CollisionObject<S>>(box_geometry);
  trans.x() = 5.;
  trans.y() = 0.;
  trans.z() = 0.;
  rot.x() = 0.;
  rot.y() = 0.;
  rot.z() = 0.;
  box->setTranslation(trans);
  box->setRotation(setRPY(rot));
  env.push_back(box);
  
  // Sphere
  shared_ptr<fcl::CollisionGeometry<S>> sphere_geometry =
      make_shared<fcl::Sphere<S>>(2.);
  shared_ptr<fcl::CollisionObject<S>> sphere =
      make_shared<fcl::CollisionObject<S>>(sphere_geometry);
  trans.x() = 8.;
  trans.y() = 3.;
  trans.z() = 0.;
  rot.x() = 0.;
  rot.y() = 0.;
  rot.z() = 0.;
  sphere->setTranslation(trans);
  sphere->setRotation(setRPY(rot));
  env.push_back(sphere);
  
  // Cylinder
  shared_ptr<fcl::CollisionGeometry<S>> cylinder_geometry =
      make_shared<fcl::Cylinder<S>>(1.,6.);
  shared_ptr<fcl::CollisionObject<S>> cylinder =
      make_shared<fcl::CollisionObject<S>>(cylinder_geometry);
  trans.x() = 5.;
  trans.y() = 7.;
  trans.z() = 0.;
  rot.x() = 0.;
  rot.y() = 0.;
  rot.z() = -M_PI/4.;
  cylinder->setTranslation(trans);
  cylinder->setRotation(setRPY(rot));
  env.push_back(cylinder);
}

template <typename S>
void fe::generateRequests(vector<shared_ptr<fcl::CollisionObject<S>>>& req, const Eigen::Vector3d trans, const Eigen::Vector3d rot)
{
  fcl::Vector3d orig_trans;
  fcl::Vector3d orig_rot;

  // Box
  shared_ptr<fcl::CollisionGeometry<S>> box_geometry =
      make_shared<fcl::Box<S>>(2.,2.,2.);
  shared_ptr<fcl::CollisionObject<S>> box =
      make_shared<fcl::CollisionObject<S>>(box_geometry);
  box->setTranslation(trans);
  box->setRotation(setRPY(rot));
  req.push_back(box);

  // Cylinder1
  shared_ptr<fcl::CollisionGeometry<S>> cylinder_geometry1 =
      make_shared<fcl::Cylinder<S>>(0.5,4.);
  shared_ptr<fcl::CollisionObject<S>> cylinder1 =
      make_shared<fcl::CollisionObject<S>>(cylinder_geometry1);
  orig_trans.x() = trans.x() + 1.;
  orig_trans.y() = trans.y() - sqrt(3.);
  orig_trans.z() = 0.;
  orig_rot.x() = 0.;
  orig_rot.y() = 0.;
  orig_rot.z() = -M_PI/3.;
  cylinder1->setTranslation(orig_trans);
  cylinder1->setRotation(setRPY(orig_rot));
  req.push_back(cylinder1);

  // Cylinder2
  shared_ptr<fcl::CollisionGeometry<S>> cylinder_geometry2 =
      make_shared<fcl::Cylinder<S>>(0.5,4.);
  shared_ptr<fcl::CollisionObject<S>> cylinder2 =
      make_shared<fcl::CollisionObject<S>>(cylinder_geometry2);
  orig_trans.x() = trans.x() - 1.;
  orig_trans.y() = trans.y() - sqrt(3.);
  orig_trans.z() = 0.;
  orig_rot.x() = 0.;
  orig_rot.y() = 0.;
  orig_rot.z() = M_PI/3.;
  cylinder2->setTranslation(orig_trans);
  cylinder2->setRotation(setRPY(orig_rot));
  req.push_back(cylinder2);
  
  // Sphere1
  shared_ptr<fcl::CollisionGeometry<S>> sphere_geometry1 =
      make_shared<fcl::Sphere<S>>(1.5);
  shared_ptr<fcl::CollisionObject<S>> sphere1 =
      make_shared<fcl::CollisionObject<S>>(sphere_geometry1);
  orig_trans.x() = trans.x() + 2.;
  orig_trans.y() = trans.y() - 2.*sqrt(3.);
  orig_trans.z() = 0.;
  orig_rot.x() = 0.;
  orig_rot.y() = 0.;
  orig_rot.z() = -M_PI/3.;
  sphere1->setTranslation(orig_trans);
  sphere1->setRotation(setRPY(orig_rot));
  req.push_back(sphere1);

  // Sphere2
  shared_ptr<fcl::CollisionGeometry<S>> sphere_geometry2 =
      make_shared<fcl::Sphere<S>>(1.5);
  shared_ptr<fcl::CollisionObject<S>> sphere2 =
      make_shared<fcl::CollisionObject<S>>(sphere_geometry1);
  orig_trans.x() = trans.x() - 2.;
  orig_trans.y() = trans.y() - 2.*sqrt(3.);
  orig_trans.z() = 0.;
  orig_rot.x() = 0.;
  orig_rot.y() = 0.;
  orig_rot.z() = M_PI/3.;
  sphere2->setTranslation(orig_trans);
  sphere2->setRotation(setRPY(orig_rot));
  req.push_back(sphere2);
}
