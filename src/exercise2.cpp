//
// Created by Jemin Hwangbo on 2022/03/17.
//


#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

#include "raisim/RaisimServer.hpp"
#include "exercise2_STUDENTID.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world); // visualization server
  world.addGround();
  world.setTimeStep(0.001);

  // a1
  // anymal
  auto anymal = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/anymal_c/urdf/anymal.urdf");
  anymal->setName("anymal");
  server.focusOn(anymal);

  // a1 configuration
  Eigen::VectorXd gc(anymal->getGeneralizedCoordinateDim());
  Eigen::VectorXd gv(anymal->getDOF());

  gc << 0, 0, 10.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  gv << 0.1, 0.2, 0.3, 0.1, 0.4, 0.3, 0.1,0.1,0.1, 0.2,0.2,0.2, 0.3,0.3,0.3, 0.4,0.4,0.4;
  anymal->setState(gc, gv);

  // visualization
  server.launchServer();
  raisim::Vec<3> footVel, footAngVel;
  bool answerCorrect = true;
  raisim::USLEEP(10000000);

  for (int i=0; i<2000; i++) {
    RS_TIMED_LOOP(world.getTimeStep()*2e6);

    anymal->getFrameVelocity("LH_shank_fixed_LH_FOOT", footVel);
    anymal->getFrameAngularVelocity("LH_shank_fixed_LH_FOOT", footAngVel);

    if((footVel.e() - getFootLinearVelocity(gc, gv)).norm() < 1e-8) {
      std::cout<<"the linear velocity is correct "<<std::endl;
    } else {
      std::cout<<"the linear velocity is not correct "<<std::endl;
      answerCorrect = false;
    }

    if((footAngVel.e() - getFootAngularVelocity(gc, gv)).norm() < 1e-8) {
      std::cout<<"the angular velocity is correct "<<std::endl;
    } else {
      std::cout<<"the angular velocity is not correct "<<std::endl;
      answerCorrect = false;
    }

    server.integrateWorldThreadSafe();
    anymal->getState(gc, gv);
  }

  server.killServer();

  if(answerCorrect) {
    std::cout<<"The solution is correct "<<std::endl;
  } else {
    std::cout<<"The solution is not correct "<<std::endl;
  }

  return 0;
}
