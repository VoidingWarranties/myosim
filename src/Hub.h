/* This class is a drop in replacement for the myo::Hub class. It listens for
 * input and calls the onPose method for each of the listeners when a pose is
 * typed.
 */

#pragma once

#include <myo/myo.hpp>

#include <iostream>
#include <string>
#include <list>

namespace MyoSim {
class Hub : public myo::Hub {
 public:
  Hub(const std::string& applicationIdentifier = "")
      : myo::Hub(applicationIdentifier), myo_(nullptr) {}

  myo::Myo* waitForMyo(unsigned int milliseconds = 0) {
    myo_ = myo::Hub::waitForMyo(milliseconds);
    return myo_;
  }
  void addListener(myo::DeviceListener* listener) {
    listeners_.push_back(listener);
  }
  void removeListener(myo::DeviceListener* listener) {
    // Find the listener in the list.
    auto itr = listeners_.begin();
    for (; itr != listeners_.end(); ++itr) {
      if (*itr == listener) break;
    }
    // Remove the listener.
    if (itr != listeners_.end()) listeners_.erase(itr);
  }

  // duration_ms currently has no effect. Both of these functions will wait for
  // user input before returning. This should be changed to more closely mimic
  // the myo::Hub behavior.
  void run(unsigned int duration_ms) { detectAndTriggerPose(); }
  void runOnce(unsigned int duration_ms) { detectAndTriggerPose(); }

  /////////////////////////////////////////
  // Functions for simulating Myo events //
  /////////////////////////////////////////
  void onPair(myo::Myo* myo, uint64_t timestamp,
              myo::FirmwareVersion firmware_version);
  void onUnpair(myo::Myo* myo, uint64_t timestamp);
  void onConnect(myo::Myo* myo, uint64_t timestamp,
                 myo::FirmwareVersion firmware_version);
  void onDisconnect(myo::Myo* myo, uint64_t timestamp);
  void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm,
                 myo::XDirection x_direction);
  void onArmUnsync(myo::Myo* myo, uint64_t timestamp);
  void onUnlock(myo::Myo* myo, uint64_t timestamp);
  void onLock(myo::Myo* myo, uint64_t timestamp);
  void onPose(myo::Myo* myo, uint64_t timestamp, const myo::Pose& pose);
  void onOrientationData(myo::Myo* myo, uint64_t timestamp,
                         const myo::Quaternion<float>& rotation);
  void onAccelerometerData(myo::Myo* myo, uint64_t timestamp,
                           const myo::Vector3<float>& accel);
  void onGyroscopeData(myo::Myo* myo, uint64_t timestamp,
                       const myo::Vector3<float>& gyro);
  void onRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi);
  void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg);

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
    else if (pose_str == "doubleTap")
      pose = myo::Pose::doubleTap;
    else if (pose_str == "unknown")
      pose = myo::Pose::unknown;
    else {
      std::cerr
          << "MyoSimulator: \"" << pose_str << "\" is not a valid pose. "
          << "Valid poses are:\n"
          << "  rest, fist, waveIn, waveOut, fingersSpread, doubleTap, unknown."
          << std::endl;
      return;
    }
    for (auto itr = listeners_.begin(); itr != listeners_.end(); ++itr) {
      (*itr)->onPose(myo_, 0, pose);
    }
  }

  std::list<myo::DeviceListener*> listeners_;
  myo::Myo* myo_;
};
}
