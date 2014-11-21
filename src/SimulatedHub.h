#ifndef MYOSIMULATOR_SIMULATEDHUB_H_
#define MYOSIMULATOR_SIMULATEDHUB_H_

#include <iostream>
#include <string>
#include <myo/myo.hpp>

namespace MyoSimulator {
class Hub {
 public:
  Hub(const std::string& applicationIdentifier = "")
      : application_identifier_(applicationIdentifier), myo_(nullptr) {}

  // waitForMyo returns a null pointer for now. In the future a SimulatedMyo
  // class should be returned instead. However, a pointer to a SimulatedMyo
  // object needs to be convertable to a myo::Myo pointer in order for onPose
  // and all the other DeviceListener methods to work.
  myo::Myo* waitForMyo(unsigned int milliseconds = 0) { return myo_; }
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
                   "               rest, fist, waveIn, waveOut, fingersSpread, reserved1,\n"
                   "               thumbToPinky, unknown." << std::endl;
      return;
    }
    listener_->onPose(myo_, 0, pose);
  }

  std::string application_identifier_;
  myo::DeviceListener* listener_;
  myo::Myo* myo_;
};
}

#endif
