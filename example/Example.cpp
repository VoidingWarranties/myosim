#include <iostream>
#include <myo/myo.hpp>
#include "../src/SimulatedHub.h"

class Listener : public myo::DeviceListener {
 public:
  void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {
    std::cout << "Detected pose! " << pose << std::endl;
  }
};

int main() {
  try {
    MyoSimulator::Hub hub("com.voidingwarranties.myo-simulator-example");

    Listener listener;
    hub.addListener(&listener);

    while (true) {
      hub.run(0);
    }
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }
  return 0;
}
