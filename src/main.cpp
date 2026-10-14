#include <iostream>

#include "odin2_ros_driver/driver.hpp"
int main(int argc, char** argv) {
  try {
    return odin_driver::runDriver(argc, argv, odin_driver::makeLiveSdk());
  } catch (const std::exception& e) {
    std::cerr << "Odin2 SDK initialization: " << e.what() << std::endl;
    return 1;
  }
}
