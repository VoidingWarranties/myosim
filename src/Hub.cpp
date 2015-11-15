#include "Hub.h"

#include <algorithm>

using namespace MyoSim;

Hub::Hub(const std::string& application_identifier)
    : myo::Hub(application_identifier) {}

myo::Myo* Hub::waitForMyo(unsigned int milliseconds) {
  return myo::Hub::waitForMyo(milliseconds);
}

void Hub::addListener(myo::DeviceListener* listener) {
  listeners_.push_back(listener);
}

void Hub::removeListener(myo::DeviceListener* listener) {
  auto itr = std::find(listeners_.begin(), listeners_.end(), listener);
  if (itr != listeners_.end()) listeners_.erase(itr);
}

void Hub::run(unsigned int duration_ms) {}
void Hub::runOnce(unsigned int duration_ms) {}

void Hub::simulatePair(myo::Myo* myo, uint64_t timestamp,
                       myo::FirmwareVersion firmware_version) {
  for (auto listener : listeners_) {
    listener->onPair(myo, timestamp, firmware_version);
  }
}

void Hub::simulateUnpair(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : listeners_) {
    listener->onUnpair(myo, timestamp);
  }
}

void Hub::simulateConnect(myo::Myo* myo, uint64_t timestamp,
                          myo::FirmwareVersion firmware_version) {
  for (auto listener : listeners_) {
    listener->onConnect(myo, timestamp, firmware_version);
  }
}

void Hub::simulateDisconnect(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : listeners_) {
    listener->onDisconnect(myo, timestamp);
  }
}

void Hub::simulateArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm,
                          myo::XDirection x_direction, float rotation,
                          myo::WarmupState warmup_state) {
  for (auto listener : listeners_) {
    listener->onArmSync(myo, timestamp, arm, x_direction, rotation,
                        warmup_state);
  }
}

void Hub::simulateArmUnsync(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : listeners_) {
    listener->onArmUnsync(myo, timestamp);
  }
}

void Hub::simulateUnlock(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : listeners_) {
    listener->onUnlock(myo, timestamp);
  }
}

void Hub::simulateLock(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : listeners_) {
    listener->onLock(myo, timestamp);
  }
}

void Hub::simulatePose(myo::Myo* myo, uint64_t timestamp,
                       const myo::Pose& pose) {
  for (auto listener : listeners_) {
    listener->onPose(myo, timestamp, pose);
  }
}

void Hub::simulateOrientationData(myo::Myo* myo, uint64_t timestamp,
                                  const myo::Quaternion<float>& rotation) {
  for (auto listener : listeners_) {
    listener->onOrientationData(myo, timestamp, rotation);
  }
}

void Hub::simulateAccelerometerData(myo::Myo* myo, uint64_t timestamp,
                                    const myo::Vector3<float>& accel) {
  for (auto listener : listeners_) {
    listener->onAccelerometerData(myo, timestamp, accel);
  }
}

void Hub::simulateGyroscopeData(myo::Myo* myo, uint64_t timestamp,
                                const myo::Vector3<float>& gyro) {
  for (auto listener : listeners_) {
    listener->onGyroscopeData(myo, timestamp, gyro);
  }
}

void Hub::simulateRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi) {
  for (auto listener : listeners_) {
    listener->onRssi(myo, timestamp, rssi);
  }
}

void Hub::simulateBatteryLevelReceived(myo::Myo* myo, uint64_t timestamp,
                                       uint8_t level) {
  for (auto listener : listeners_) {
    listener->onBatteryLevelReceived(myo, timestamp, level);
  }
}

void Hub::simulateEmgData(myo::Myo* myo, uint64_t timestamp,
                          const int8_t* emg) {
  for (auto listener : listeners_) {
    listener->onEmgData(myo, timestamp, emg);
  }
}

void Hub::simulateWarmupCompleted(myo::Myo* myo, uint64_t timestamp,
                                  myo::WarmupResult warmup_result) {
  for (auto listener : listeners_) {
    listener->onWarmupCompleted(myo, timestamp, warmup_result);
  }
}
