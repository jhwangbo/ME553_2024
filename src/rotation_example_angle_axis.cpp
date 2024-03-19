#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main() {
  raisim::World world;
  world.addGround();

  raisim::RaisimServer server(&world);

  auto monkey = server.addVisualMesh("monkey", std::string(_MAKE_STR(RESOURCE_DIR)) + "/monkey/monkey.obj");
  monkey->setColor(0,0,1,1);
  monkey->setPosition(2.5,0,1);

  auto monkey_reference = server.addVisualMesh("monkey_reference", std::string(_MAKE_STR(RESOURCE_DIR)) + "/monkey/monkey.obj");
  monkey_reference->setPosition(0,0,1);

  double angle = M_PI_2;
  Eigen::Vector3d axis{0,0,1};
  axis /= axis.norm();

  Eigen::Vector4d ori;
  ori[0] = std::cos(angle/2.);
  ori.tail(3) = std::sin(angle/2.) * axis;

  monkey->setOrientation(ori);

  server.launchServer();

  while(true) {
    raisim::MSLEEP(4);
  }

  server.killServer();

  return 0;
}
