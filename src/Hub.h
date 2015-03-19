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
  Hub(const std::string& applicationIdentifier = "");

  myo::Myo* waitForMyo(unsigned int milliseconds = 0);
  void addListener(myo::DeviceListener* listener);
  void removeListener(myo::DeviceListener* listener);

  // duration_ms currently has no effect. Both of these functions will wait for
  // user input before returning. This should be changed to more closely mimic
  // the myo::Hub behavior.
  void run(unsigned int duration_ms);
  void runOnce(unsigned int duration_ms);

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
  void detectAndTriggerPose();

  std::list<myo::DeviceListener*> listeners_;
  myo::Myo* myo_;
};
}
