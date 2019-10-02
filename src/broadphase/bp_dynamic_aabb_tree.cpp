#include "utils.h"

using namespace std;
namespace fe = fcl::example;

int main(int argc, char* argv[]) {

  vector<shared_ptr<fcl::CollisionObject<double>>> env;
  fe::generateEnvironments<double>(env);

  // vector<shared_ptr<fcl::CollisionObject<double>>> req;
  // Eigen::Vector3d trans(0.,5.,0.);
  // Eigen::Vector3d rot(0.,0.,0.);
  // fe::generateRequests(req, trans, rot);
  
  return 0;
}
