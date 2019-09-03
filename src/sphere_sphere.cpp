/** @author Ryodo Tanaka <groadpg@gmail.com> */
#include <iostream>
#include <Eigen/Core>

// Collision, Distance 
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
// Distance Request & Result
#include <fcl/narrowphase/distance_request.h>
#include <fcl/narrowphase/distance_result.h>

// If you wonder the fcl::FCL_REAL, fcl::Transform3, and etc...// 
// you should see <fcl/common/types.h> 
// And you will understand it is equal to Eigen functions

Eigen::Matrix3d setRPY(const Eigen::Vector3d rot)
{
    Eigen::Matrix3d ret;
    ret = Eigen::AngleAxisd(rot.x(), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(rot.y(), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rot.z(), Eigen::Vector3d::UnitZ());
    return ret;
}

int main(int argc, char* argv[]) {
    double radius1 = 1.0;
    double radius2 = 1.0;
    std::shared_ptr<fcl::CollisionGeometry<double>> sphere_geometry1(new fcl::Sphere<double>(radius1));
    std::shared_ptr<fcl::CollisionGeometry<double>> sphere_geometry2(new fcl::Sphere<double>(radius2));
    fcl::CollisionObject<double> sphere1(sphere_geometry1);
    fcl::CollisionObject<double> sphere2(sphere_geometry2);
    fcl::Vector3d trans1(0.,0.,0.);
    fcl::Vector3d rot1(0.,0.,0.); 
    fcl::Vector3d trans2(3.,3.,0.);
    fcl::Vector3d rot2(0.,0.,0.); 
    sphere1.setTranslation(trans1);
    sphere1.setRotation(setRPY(rot1));
    sphere2.setTranslation(trans2);
    sphere2.setRotation(setRPY(rot2));
    
    // Distance Request and Result 
    fcl::DistanceRequest<double> request;
    fcl::DistanceResult<double> result;
    request.enable_nearest_points = true;
    request.enable_signed_distance = true;
    request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    // request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    
    // Calculate distance
    result.clear();
    double dist = fcl::distance(&sphere1, &sphere2, request, result);
    
    // Show results
    std::cout << dist << std::endl;
    std::cout << result.min_distance << std::endl;
    std::cout << result.nearest_points[0].transpose() << std::endl;
    std::cout << result.nearest_points[1].transpose() << std::endl;
    
    return 0;
}
