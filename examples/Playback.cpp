#include <iostream>
#include <myo/myo.hpp>
#include "../src/EventRecorder.h"
#include "../src/EventPlayer.h"
#include "../src/Hub.h"

class PrintListener : public myo::DeviceListener {
 public:
  void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {
    std::cout << "Detected pose! " << pose << std::endl;
  }
};

int main() {
  try {
    // An actual myo::Hub; NOT a MyoSim::Hub.
    myo::Hub hub("com.voidingwarranties.myo-simulator-example");
    myo::Myo* myo = hub.waitForMyo(10000);
    if (!myo) {
      throw std::runtime_error("Unable to find a Myo!");
    }
    // Record only pose events.
    MyoSim::EventRecorder recorder(MyoSim::EventRecorder::POSE);
    hub.addListener(&recorder);
    // Record for 5 seconds.
    myo->unlock(myo::Myo::unlockHold);
    hub.run(5000);
    myo->lock();

    std::cout << "Events recorded. Press ENTER to replay events.";
    getchar();

    MyoSim::Hub simulated_hub;
    PrintListener print_listener;
    simulated_hub.addListener(&print_listener);
    MyoSim::EventPlayer player(simulated_hub);
    player.play(recorder.getEventSession());

  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }
  return 0;
}
