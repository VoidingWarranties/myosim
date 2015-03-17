#include <iostream>
#include <myo/myo.hpp>
#include "../src/Hub.h"

class PrintListener : public myo::DeviceListener {
 public:
  void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {
    std::cout << "Detected pose! " << pose << std::endl;
  }
};

int main() {
  try {
    MyoSim::Hub hub("com.voidingwarranties.myo-simulator-example");

    PrintListener print_listener;
    hub.addListener(&print_listener);

    while (true) {
      std::cout << "Enter a pose: ";
      hub.run(0);
    }
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }
  return 0;
}
