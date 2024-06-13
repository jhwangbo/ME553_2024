//
// Created by Jemin Hwangbo on 2022/04/08.
//

#include "raisim/RaisimServer.hpp"
#include "final_exam_STUDENTID.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world);

  server.launchServer();

  // anymal
  auto cartPole = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/cartPole/doubleCartPole.urdf");

  // anymal configuration
  Eigen::VectorXd gc(cartPole->getGeneralizedCoordinateDim()), gv(cartPole->getDOF()), gf(cartPole->getDOF());
  gc << 0.0, 0.1, 0.2; /// Jemin: I'll randomize the gc, gv when grading
  gv << 0.1, 0.2, 0.3;
  gf << 0.2, 0.3, 0.4;

  cartPole->setState(gc, gv);
  cartPole->setGeneralizedForce(gf);

  /// if you are using an old version of Raisim, you need this line
  world.integrate1();
  Eigen::VectorXd nonlinearity(cartPole->getDOF());
  Eigen::MatrixXd massMatrix(cartPole->getDOF(), cartPole->getDOF());
  massMatrix = cartPole->getMassMatrix().e();
  nonlinearity = cartPole->getNonlinearities({0,0,-9.81}).e();

  if((computeGeneralizedAcceleration(gc, gv, gf) - massMatrix.inverse() * (gf-nonlinearity)).norm() < 1e-8)
    std::cout<<"passed "<<std::endl;
  else
    std::cout<<"failed. The acceleration should be "<< (massMatrix.inverse() * (gf-nonlinearity)).transpose() <<std::endl;

  while (true) {
    RS_TIMED_LOOP(world.getTimeStep()*1e6)
    world.integrate();
  }

  return 0;
}
