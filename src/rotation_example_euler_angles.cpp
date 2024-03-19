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

  Eigen::Matrix3d xRot, yRot, zRot, rot;
  double thX = 1.0, thY = M_PI/2., thZ = 0.1;

  xRot << 1,        0,         0,
          0, cos(thX), -sin(thX),
          0, sin(thX),  cos(thX);

  yRot << cos(thY), 0, sin(thY),
                 0, 1,        0,
         -sin(thY), 0, cos(thY);

  zRot << cos(thZ), -sin(thZ), 0,
          sin(thZ),  cos(thZ), 0,
                 0,         0, 1;

  rot = xRot * yRot * zRot;

  raisim::Vec<4> ori;
  raisim::rotMatToQuat(rot, ori);
  monkey->setOrientation(ori.e());
  server.launchServer();

  while(true) {
    raisim::MSLEEP(4);
  }

  server.killServer();

  return 0;
}
