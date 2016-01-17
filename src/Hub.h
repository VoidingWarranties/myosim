/* Hub derives from myo::Hub and provides helper functions for simulating
 * events. Any call to simulateEvent will call the corresponding event on each
 * of the hub's device listener.
 */

#pragma once

#include <myo/myo.hpp>

#include <iostream>
#include <string>
#include <list>

namespace myosim {
class Hub : public myo::Hub {
 public:
  explicit Hub(const std::string& application_identifier = "");

  /////////////////////////////////////////
  // Functions for simulating Myo events //
  /////////////////////////////////////////
  void simulatePair(myo::Myo* myo, uint64_t timestamp,
                    myo::FirmwareVersion firmware_version);
  void simulateUnpair(myo::Myo* myo, uint64_t timestamp);
  void simulateConnect(myo::Myo* myo, uint64_t timestamp,
                       myo::FirmwareVersion firmware_version);
  void simulateDisconnect(myo::Myo* myo, uint64_t timestamp);
  void simulateArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm,
                       myo::XDirection x_direction, float rotation,
                       myo::WarmupState warmup_state);
  void simulateArmUnsync(myo::Myo* myo, uint64_t timestamp);
  void simulateUnlock(myo::Myo* myo, uint64_t timestamp);
  void simulateLock(myo::Myo* myo, uint64_t timestamp);
  void simulatePose(myo::Myo* myo, uint64_t timestamp, const myo::Pose& pose);
  void simulateOrientationData(myo::Myo* myo, uint64_t timestamp,
                               const myo::Quaternion<float>& rotation);
  void simulateAccelerometerData(myo::Myo* myo, uint64_t timestamp,
                                 const myo::Vector3<float>& accel);
  void simulateGyroscopeData(myo::Myo* myo, uint64_t timestamp,
                             const myo::Vector3<float>& gyro);
  void simulateRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi);
  void simulateBatteryLevelReceived(myo::Myo* myo, uint64_t timestamp,
                                    uint8_t level);
  void simulateEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg);
  void simulateWarmupCompleted(myo::Myo* myo, uint64_t timestamp,
                               myo::WarmupResult warmup_result);
};
} // namespace myosim
