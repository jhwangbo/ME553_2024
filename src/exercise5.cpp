//
// Created by Jemin Hwangbo on 2022/04/08.
//

#include "raisim/RaisimServer.hpp"
#include "exercise5_STUDENTID.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world

  // anymal
  auto anymal = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/anymal_c/urdf/anymal.urdf");

  // anymal configuration
  Eigen::VectorXd gc(anymal->getGeneralizedCoordinateDim()), gv(anymal->getDOF()), gf(anymal->getDOF());
  gc << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8; /// Jemin: I'll randomize the gc, gv when grading
  gv << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8;
  gf << 0.15, 0.21, 0.36, 0.24, 0.35, 0.46, 0.57, 0.18, 0.29, 1.0, 1.1, 1.5, 1.1, 1.2, 1.3, 1.6, 1.7, 1.8;
  anymal->setState(gc, gv);
  anymal->setGeneralizedForce(gf);

  /// if you are using an old version of Raisim, you need this line
  world.integrate1();
  Eigen::VectorXd nonlinearity(anymal->getDOF());
  Eigen::MatrixXd massMatrix(anymal->getDOF(), anymal->getDOF());
  massMatrix = anymal->getMassMatrix().e();
  nonlinearity = anymal->getNonlinearities({0,0,-9.81}).e();

  if((computeGeneralizedAcceleration(gc, gv, gf) - massMatrix.inverse() * (gf-nonlinearity)).norm() < 1e-8)
    std::cout<<"passed "<<std::endl;
  else
    std::cout<<"failed. The acceleration should be "<< (massMatrix.inverse() * (gf-nonlinearity)).transpose() <<std::endl;

  return 0;
}
