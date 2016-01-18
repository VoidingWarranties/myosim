#include "hub.h"

#include <algorithm>

namespace myosim {
Hub::Hub(const std::string& application_identifier)
    : myo::Hub(application_identifier) {}

void Hub::simulatePair(myo::Myo* myo, uint64_t timestamp,
                       myo::FirmwareVersion firmware_version) {
  for (auto listener : _listeners) {
    listener->onPair(myo, timestamp, firmware_version);
  }
}

void Hub::simulateUnpair(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : _listeners) {
    listener->onUnpair(myo, timestamp);
  }
}

void Hub::simulateConnect(myo::Myo* myo, uint64_t timestamp,
                          myo::FirmwareVersion firmware_version) {
  for (auto listener : _listeners) {
    listener->onConnect(myo, timestamp, firmware_version);
  }
}

void Hub::simulateDisconnect(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : _listeners) {
    listener->onDisconnect(myo, timestamp);
  }
}

void Hub::simulateArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm,
                          myo::XDirection x_direction, float rotation,
                          myo::WarmupState warmup_state) {
  for (auto listener : _listeners) {
    listener->onArmSync(myo, timestamp, arm, x_direction, rotation,
                        warmup_state);
  }
}

void Hub::simulateArmUnsync(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : _listeners) {
    listener->onArmUnsync(myo, timestamp);
  }
}

void Hub::simulateUnlock(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : _listeners) {
    listener->onUnlock(myo, timestamp);
  }
}

void Hub::simulateLock(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : _listeners) {
    listener->onLock(myo, timestamp);
  }
}

void Hub::simulatePose(myo::Myo* myo, uint64_t timestamp,
                       const myo::Pose& pose) {
  for (auto listener : _listeners) {
    listener->onPose(myo, timestamp, pose);
  }
}

void Hub::simulateOrientationData(myo::Myo* myo, uint64_t timestamp,
                                  const myo::Quaternion<float>& rotation) {
  for (auto listener : _listeners) {
    listener->onOrientationData(myo, timestamp, rotation);
  }
}

void Hub::simulateAccelerometerData(myo::Myo* myo, uint64_t timestamp,
                                    const myo::Vector3<float>& accel) {
  for (auto listener : _listeners) {
    listener->onAccelerometerData(myo, timestamp, accel);
  }
}

void Hub::simulateGyroscopeData(myo::Myo* myo, uint64_t timestamp,
                                const myo::Vector3<float>& gyro) {
  for (auto listener : _listeners) {
    listener->onGyroscopeData(myo, timestamp, gyro);
  }
}

void Hub::simulateRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi) {
  for (auto listener : _listeners) {
    listener->onRssi(myo, timestamp, rssi);
  }
}

void Hub::simulateBatteryLevelReceived(myo::Myo* myo, uint64_t timestamp,
                                       uint8_t level) {
  for (auto listener : _listeners) {
    listener->onBatteryLevelReceived(myo, timestamp, level);
  }
}

void Hub::simulateEmgData(myo::Myo* myo, uint64_t timestamp,
                          const int8_t* emg) {
  for (auto listener : _listeners) {
    listener->onEmgData(myo, timestamp, emg);
  }
}

void Hub::simulateWarmupCompleted(myo::Myo* myo, uint64_t timestamp,
                                  myo::WarmupResult warmup_result) {
  for (auto listener : _listeners) {
    listener->onWarmupCompleted(myo, timestamp, warmup_result);
  }
}
} // namespace myosim
