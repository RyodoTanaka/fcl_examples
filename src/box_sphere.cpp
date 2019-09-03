/** @author Ryodo Tanaka <groadpg@gmail.com> */
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

// Collision, Distance 
#include <fcl/collision.h>
#include <fcl/distance.h>
// What phase will use for checking
// #include <fcl/narrowphase/narrowphase.h>
// #include <fcl/broadphase/broadphase.h>
// Geometrical shape
#include <fcl/shape/geometric_shapes.h>
// Use pi, phi(golden ratio)
#include <fcl/math/constants.h>
// Distance Request & Result
#include <fcl/collision_data.h>

#include <typeinfo>

// If you wonder the fcl::FCL_REAL, fcl::Transform3, and etc...// 
// you should see <fcl/data_types.h> 
// About Transform3f, <fcl/math/transform.h>
// Matrix3f <fcl/math/matrix3f.h>
// Vec3f <fcl/math/vec3f.h

int main(int argc, char* argv[]) {
    // Translate & Rotation 1
    fcl::Vec3f trans1;
    fcl::Matrix3f rot1;
    trans1.setValue(0.,0.,0.);
    rot1.setEulerYPR(0.,0.,0.);
    // Translate & Rotation 2
    fcl::Vec3f trans2;
    fcl::Matrix3f rot2;
    trans2.setValue(3.0,3.,0.);
    rot2.setEulerYPR(0.,0.,0.);
    // set Transform
    fcl::Transform3f pose1;
    fcl::Transform3f pose2;
    pose1.setTransform(rot1,trans1);
    pose2.setTransform(rot2,trans2);
    //////////////////////
    // set Sphere & Box //
    //////////////////////
    fcl::Box box = fcl::Box(10.0,2.0,2.0);
    double radius = 1.0;
    fcl::Sphere sphere = fcl::Sphere(radius);
    // Distance Request and Result 
    fcl::DistanceRequest request;
    request.enable_nearest_points = true;
    request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    fcl::DistanceResult result;
    result.clear();
    
    // Calculate distance
    fcl::distance(&sphere, pose2, &box, pose1, request, result);

    // Show results
    std::cout << result.min_distance << std::endl;
    std::cout << result.nearest_points[0] << std::endl;
    std::cout << result.nearest_points[1] << std::endl;
    std::cout << pose2.transform(result.nearest_points[0]) << std::endl;
    std::cout << pose1.transform(result.nearest_points[1]) << std::endl;
    return 0;
}
