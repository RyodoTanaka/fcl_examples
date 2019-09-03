/** @author Ryodo Tanaka <groadpg@gmail.com> */
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

// Collision, Distance 
#include <fcl/collision.h>
#include <fcl/distance.h>
// What phase will use for checking
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/broadphase/broadphase.h>
// Geometrical shape
#include <fcl/shape/geometric_shapes.h>
// Use pi, phi(golden ratio)
#include <fcl/math/constants.h>
// Distance Request & Result
#include <fcl/collision_data.h>

// If you wonder the fcl::FCL_REAL, fcl::Transform3, and etc...// 
// you should see <fcl/data_types.h> 
// About Transform3f, <fcl/math/transform.h>
// Matrix3f <fcl/math/matrix3f.h>
// Vec3f <fcl/math/vec3f.h

int main(int argc, char* argv[]) {
    // Translate & Rotation 1
    fcl::Vec3f trans1;
    fcl::Matrix3f rot1;
    trans1.setValue(5.,5.,0.);
    rot1.setEulerYPR(0.,0.,0.);
    // Translate & Rotation 2
    fcl::Vec3f trans2;
    fcl::Matrix3f rot2;
    trans2.setValue(1.,0.,0.);
    rot2.setEulerYPR(0.,0.,0.);
    // set Transform
    fcl::Transform3f pose1;
    fcl::Transform3f pose2;
    pose1.setTransform(rot1,trans1);
    pose2.setTransform(rot2,trans2);
    ////////////////
    // set Sphere //
    ////////////////
    double radius1 = 2.0;
    double radius2 = 2.0;
    fcl::Sphere sphere1 = fcl::Sphere(radius1);
    fcl::Sphere sphere2 = fcl::Sphere(radius2);
    // Distance Request and Result 
    fcl::DistanceRequest request;
    fcl::DistanceResult result;

    // Calculate distance
    result.clear();
    fcl::FCL_REAL dist = fcl::distance(&sphere1, pose1, &sphere2, pose2, request, result);

    // Show results
    std::cout << dist << std::endl;
    
    return 0;
}
