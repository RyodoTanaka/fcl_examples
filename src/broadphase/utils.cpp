#include "utils.h"

#include <cmath>

using namespace std;

void fe::generateEnvironments(vector< fcl::CollisionObject<double>* >& env)
{
  fcl::Vector3d trans;
  fcl::Vector3d rot;

  // Box
  shared_ptr<fcl::CollisionGeometry<double>> box_geometry =
      make_shared<fcl::Box<double>>(10.,2.,2.);
  shared_ptr<fcl::CollisionObject<double>> box =
      make_shared<fcl::CollisionObject<double>>(box_geometry);
  trans.x() = 5.;
  trans.y() = 0.;
  trans.z() = 0.;
  rot.x() = 0.;
  rot.y() = 0.;
  rot.z() = 0.;
  box->setTranslation(trans);
  box->setRotation(setRPY(rot));
  env.push_back(box.get());
  
  // Sphere
  shared_ptr<fcl::CollisionGeometry<double>> sphere_geometry =
      make_shared<fcl::Sphere<double>>(2.);
  shared_ptr<fcl::CollisionObject<double>> sphere =
      make_shared<fcl::CollisionObject<double>>(sphere_geometry);
  trans.x() = 8.;
  trans.y() = 3.;
  trans.z() = 0.;
  rot.x() = 0.;
  rot.y() = 0.;
  rot.z() = 0.;
  sphere->setTranslation(trans);
  sphere->setRotation(setRPY(rot));
  env.push_back(sphere.get());
  
  // Cylinder
  shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry =
      make_shared<fcl::Cylinder<double>>(1.,6.);
  shared_ptr<fcl::CollisionObject<double>> cylinder =
      make_shared<fcl::CollisionObject<double>>(cylinder_geometry);
  trans.x() = 5.;
  trans.y() = 7.;
  trans.z() = 0.;
  rot.x() = 0.;
  rot.y() = 0.;
  rot.z() = -M_PI/4.;
  cylinder->setTranslation(trans);
  cylinder->setRotation(setRPY(rot));
  env.push_back(cylinder.get());
}

void fe::generateQueries(vector< fcl::CollisionObject<double>* >& req, const Eigen::Vector3d trans, const Eigen::Vector3d rot)
{
  fcl::Vector3d orig_trans;
  fcl::Vector3d orig_rot;

  // Box
  shared_ptr<fcl::CollisionGeometry<double>> box_geometry =
      make_shared<fcl::Box<double>>(2.,2.,2.);
  shared_ptr<fcl::CollisionObject<double>> box =
      make_shared<fcl::CollisionObject<double>>(box_geometry);
  box->setTranslation(trans);
  box->setRotation(setRPY(rot));
  req.push_back(box.get());

  // Cylinder1
  shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry1 =
      make_shared<fcl::Cylinder<double>>(0.5,4.);
  shared_ptr<fcl::CollisionObject<double>> cylinder1 =
      make_shared<fcl::CollisionObject<double>>(cylinder_geometry1);
  orig_trans.x() = trans.x() + 1.;
  orig_trans.y() = trans.y() - sqrt(3.);
  orig_trans.z() = 0.;
  orig_rot.x() = 0.;
  orig_rot.y() = 0.;
  orig_rot.z() = -M_PI/3.;
  cylinder1->setTranslation(orig_trans);
  cylinder1->setRotation(setRPY(orig_rot));
  req.push_back(cylinder1.get());

  // Cylinder2
  shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry2 =
      make_shared<fcl::Cylinder<double>>(0.5,4.);
  shared_ptr<fcl::CollisionObject<double>> cylinder2 =
      make_shared<fcl::CollisionObject<double>>(cylinder_geometry2);
  orig_trans.x() = trans.x() - 1.;
  orig_trans.y() = trans.y() - sqrt(3.);
  orig_trans.z() = 0.;
  orig_rot.x() = 0.;
  orig_rot.y() = 0.;
  orig_rot.z() = M_PI/3.;
  cylinder2->setTranslation(orig_trans);
  cylinder2->setRotation(setRPY(orig_rot));
  req.push_back(cylinder2.get());
  
  // Sphere1
  shared_ptr<fcl::CollisionGeometry<double>> sphere_geometry1 =
      make_shared<fcl::Sphere<double>>(1.5);
  shared_ptr<fcl::CollisionObject<double>> sphere1 =
      make_shared<fcl::CollisionObject<double>>(sphere_geometry1);
  orig_trans.x() = trans.x() + 2.;
  orig_trans.y() = trans.y() - 2.*sqrt(3.);
  orig_trans.z() = 0.;
  orig_rot.x() = 0.;
  orig_rot.y() = 0.;
  orig_rot.z() = -M_PI/3.;
  sphere1->setTranslation(orig_trans);
  sphere1->setRotation(setRPY(orig_rot));
  req.push_back(sphere1.get());

  // Sphere2
  shared_ptr<fcl::CollisionGeometry<double>> sphere_geometry2 =
      make_shared<fcl::Sphere<double>>(1.5);
  shared_ptr<fcl::CollisionObject<double>> sphere2 =
      make_shared<fcl::CollisionObject<double>>(sphere_geometry1);
  orig_trans.x() = trans.x() - 2.;
  orig_trans.y() = trans.y() - 2.*sqrt(3.);
  orig_trans.z() = 0.;
  orig_rot.x() = 0.;
  orig_rot.y() = 0.;
  orig_rot.z() = M_PI/3.;
  sphere2->setTranslation(orig_trans);
  sphere2->setRotation(setRPY(orig_rot));
  req.push_back(sphere2.get());
}
