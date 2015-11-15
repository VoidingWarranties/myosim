#pragma once

#include <myo/myo.hpp>

#include <iostream>
#include <string>
#include <list>

namespace MyoSim {
class Hub : public myo::Hub {
 public:
  Hub(const std::string& application_identifier = "");

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

 private:
  std::list<myo::DeviceListener*> listeners_;
};
}
