#include <iostream>
#include <myo/myo.hpp>

#include "../src/hub.h"

class PrintListener : public myo::DeviceListener {
 public:
  void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {
    std::cout << "Detected pose! " << pose << std::endl;
  }
};

myo::Pose poseFromString(const std::string& pose_str) {
  if (pose_str == "rest") {
    return myo::Pose::rest;
  } else if (pose_str == "fist") {
    return myo::Pose::fist;
  } else if (pose_str == "waveIn") {
    return myo::Pose::waveIn;
  } else if (pose_str == "waveOut") {
    return myo::Pose::waveOut;
  } else if (pose_str == "fingersSpread") {
    return myo::Pose::fingersSpread;
  } else if (pose_str == "doubleTap") {
    return myo::Pose::doubleTap;
  } else {
    return myo::Pose::unknown;
  }
}

int main() {
  try {
    myosim::Hub hub("com.voidingwarranties.myo-simulator-example");

    PrintListener print_listener;
    hub.addListener(&print_listener);

    while (true) {
      std::cout << "Enter a pose: ";
      std::string pose_str;
      std::cin >> pose_str;
      myo::Pose pose = poseFromString(pose_str);
      hub.simulatePose(nullptr, 0, pose);
    }
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }
  return 0;
}
