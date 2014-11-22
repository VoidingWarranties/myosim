#ifndef MYOSIMULATOR_SIMULATEDHUB_H_
#define MYOSIMULATOR_SIMULATEDHUB_H_

#include <iostream>
#include <string>
#include <myo/myo.hpp>

namespace MyoSimulator {
class Hub : public myo::Hub {
 public:
  Hub(const std::string& applicationIdentifier = "")
      : myo::Hub(applicationIdentifier), myo_(nullptr) {}

  myo::Myo* waitForMyo(unsigned int milliseconds = 0) {
    myo_ = myo::Hub::waitForMyo(milliseconds);
    return myo_;
  }
  void addListener(myo::DeviceListener* listener) { listener_ = listener; }
  void removeListener(myo::DeviceListener* listener) { listener_ = nullptr; }
  void run(unsigned int duration_ms) { detectAndTriggerPose(); }
  void runOnce(unsigned int duration_ms) { detectAndTriggerPose(); }

 private:
  void detectAndTriggerPose() {
    std::string pose_str;
    std::cin >> pose_str;
    myo::Pose pose;
    if (pose_str == "rest")
      pose = myo::Pose::rest;
    else if (pose_str == "fist")
      pose = myo::Pose::fist;
    else if (pose_str == "waveIn")
      pose = myo::Pose::waveIn;
    else if (pose_str == "waveOut")
      pose = myo::Pose::waveOut;
    else if (pose_str == "fingersSpread")
      pose = myo::Pose::fingersSpread;
    else if (pose_str == "reserved1")
      pose = myo::Pose::reserved1;
    else if (pose_str == "thumbToPinky")
      pose = myo::Pose::thumbToPinky;
    else if (pose_str == "unknown")
      pose = myo::Pose::unknown;
    else {
      std::cerr << "MyoSimulator: \"" << pose_str
                << "\" is not a valid pose. Valid poses are:\n"
                   "               rest, fist, waveIn, waveOut, fingersSpread, "
                   "reserved1,\n"
                   "               thumbToPinky, unknown." << std::endl;
      return;
    }
    listener_->onPose(myo_, 0, pose);
  }

  myo::DeviceListener* listener_;
  myo::Myo* myo_;
};
}

#endif
