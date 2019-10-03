#include "utils.h"

#include <fcl/narrowphase/collision_object.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

using namespace std;
namespace fe = fcl::example;

int main(int argc, char* argv[]) {
  // Group1
  // Box
  shared_ptr<fcl::CollisionGeometry<double>> box_geometry1(new fcl::Box<double>(10.,2.,2.));
  shared_ptr<fcl::CollisionGeometry<double>> sphere_geometry1(new fcl::Sphere<double>(2.0));
  fcl::CollisionObject<double> box1(box_geometry1);
  fcl::CollisionObject<double> sphere1(sphere_geometry1);
  fcl::Vector3d trans1(0.,0.,0.);
  fcl::Vector3d rot1(0.,0.,0.); 
  box1.setTranslation(trans1);
  box1.setRotation(setRPY(rot1));
  trans1.x() = 8.0;
  trans1.y() = -2.0;
  sphere1.setTranslation(trans1);
  sphere1.setRotation(setRPY(rot1));

  // Group2
  shared_ptr<fcl::CollisionGeometry<double>> box_geometry2(new fcl::Box<double>(10.,2.,2.));
  shared_ptr<fcl::CollisionGeometry<double>> sphere_geometry2(new fcl::Sphere<double>(2.0));
  fcl::CollisionObject<double> box2(box_geometry2);
  fcl::CollisionObject<double> sphere2(sphere_geometry2);
  fcl::Vector3d trans2(0.,3.,0.);
  fcl::Vector3d rot2(0.,0.,M_PI/36.); 
  box2.setTranslation(trans2);
  box2.setRotation(setRPY(rot2));
  trans2.y() = 4.0;
  sphere2.setTranslation(trans2);
  sphere2.setRotation(setRPY(rot2));

  
  // DynamicAABBTree BroadPhase Managers
  shared_ptr<fcl::BroadPhaseCollisionManager<double>> group1 = make_shared<fcl::DynamicAABBTreeCollisionManager<double>>(); 
  shared_ptr<fcl::BroadPhaseCollisionManager<double>> group2 = make_shared<fcl::DynamicAABBTreeCollisionManager<double>>();

  // Set Objects
  group1->registerObject(&box1);
  group1->registerObject(&sphere1);
  group2->registerObject(&box2);
  group2->registerObject(&sphere2);

  // Data store
  fe::CollisionData<double> collision_data;
  fe::DistanceData<double> distance_data;

  group1->setup();
  group2->setup();

  // 1. Contact Number between env and que
  group1->collide(group2.get(), &collision_data, fe::CollisionFunction);
  int n_contact_num = collision_data.result.numContacts();
  cout << n_contact_num << endl;
  
  // 2. Distance (minimum) between env and que
  group1->distance(group2.get(), &distance_data, fe::DistanceFunction);
  double min_distance = distance_data.result.min_distance;
  cout << min_distance << endl;

  // // 3. Self collision in env
  // env_manager->collide(&collision_data, fe::CollisionFunction);

  // // 4. Self distance in env
  // env_manager->distance(&distance_data, fe::DistanceFunction);

  // // 5. Collision query between one object in env and the entire objects in que
  // que_manager->collide(env[0], &collision_data, fe::CollisionFunction);
  
  // // 6. Distance query between one object in env and the entire objects in que
  // que_manager->distance(env[0], &distance_data, fe::DistanceFunction); 
  
  return 0;
}
