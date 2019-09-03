/** @author Ryodo Tanaka <groadpg@gmail.com> */
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

// Collision, Distance 
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
// Distance Request & Result
#include <fcl/narrowphase/distance_request.h>
#include <fcl/narrowphase/distance_result.h>

// If you wonder the fcl::FCL_REAL, fcl::Transform3, and etc...// 
// you should see <fcl/data_types.h> 
// About Transform3f, <fcl/math/transform.h>
// Matrix3f <fcl/math/matrix3f.h>
// Vec3f <fcl/math/vec3f.h

Eigen::Matrix3d setRPY(const Eigen::Vector3d rot)
{
    Eigen::Matrix3d ret;
    ret = Eigen::AngleAxisd(rot.x(), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(rot.y(), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rot.z(), Eigen::Vector3d::UnitZ());
    return ret;
}

int main(int argc, char* argv[]) {
    fcl::Vector3d trans1(0.,0.,0.);
    fcl::Vector3d rot1(0.,0.,0.); 
    fcl::Transform3d X_F1;
    X_F1.translation() = trans1;
    // X_F1.rotation() = setRPY(rot1);

    fcl::Vector3d trans2(5.,5.,0.);
    fcl::Vector3d rot2(0.,0.,0.); 
    fcl::Transform3d X_F2;
    X_F2.translation() = trans2;
    // X_F2.rotation() = setRPY(rot2);

    double radius1 = 2.0;
    double radius2 = 2.0;
    std::shared_ptr<fcl::CollisionGeometry<double>> sphere_geometry1(new fcl::Sphere<double>(radius1));
    std::shared_ptr<fcl::CollisionGeometry<double>> sphere_geometry2(new fcl::Sphere<double>(radius2));
    fcl::CollisionObject<double> sphere1(sphere_geometry1, X_F1);
    fcl::CollisionObject<double> sphere2(sphere_geometry2, X_F2);
    
    // Distance Request and Result 
    fcl::DistanceRequest<double> request;
    fcl::DistanceResult<double> result;
    request.enable_nearest_points = true;
    request.enable_signed_distance = true;
    request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    
    // Calculate distance
    result.clear();
    std::cout << "start to calculate distance" << std::endl;
    double dist = fcl::distance(&sphere1, &sphere2, request, result);
    std::cout << "end to calculate distance" << std::endl;

    
    // Show results
    std::cout << dist << std::endl;
    std::cout << result.min_distance << std::endl;
    std::cout << result.nearest_points[0] << std::endl;
    std::cout << result.nearest_points[1] << std::endl;
    // std::cout << pose1.transform(result.nearest_points[0]) << std::endl;
    // std::cout << pose2.transform(result.nearest_points[1]) << std::endl;
    
    return 0;
}
