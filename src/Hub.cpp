#include "Hub.h"

using namespace MyoSim;

Hub::Hub(const std::string& applicationIdentifier)
    : myo::Hub(applicationIdentifier), myo_(nullptr) {}

myo::Myo* Hub::waitForMyo(unsigned int milliseconds) {
  myo_ = myo::Hub::waitForMyo(milliseconds);
  return myo_;
}

void Hub::addListener(myo::DeviceListener* listener) {
  listeners_.push_back(listener);
}

void Hub::removeListener(myo::DeviceListener* listener) {
  // Find the listener in the list.
  auto itr = listeners_.begin();
  for (; itr != listeners_.end(); ++itr) {
    if (*itr == listener) break;
  }
  // Remove the listener.
  if (itr != listeners_.end()) listeners_.erase(itr);
}

void Hub::run(unsigned int duration_ms) { detectAndTriggerPose(); }
void Hub::runOnce(unsigned int duration_ms) { detectAndTriggerPose(); }

void Hub::onPair(myo::Myo* myo, uint64_t timestamp,
                 myo::FirmwareVersion firmware_version) {
  for (auto listener : listeners_) {
    listener->onPair(myo, timestamp, firmware_version);
  }
}

void Hub::onUnpair(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : listeners_) {
    listener->onUnpair(myo, timestamp);
  }
}

void Hub::onConnect(myo::Myo* myo, uint64_t timestamp,
                    myo::FirmwareVersion firmware_version) {
  for (auto listener : listeners_) {
    listener->onConnect(myo, timestamp, firmware_version);
  }
}

void Hub::onDisconnect(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : listeners_) {
    listener->onDisconnect(myo, timestamp);
  }
}

void Hub::onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm,
                    myo::XDirection x_direction) {
  for (auto listener : listeners_) {
    listener->onArmSync(myo, timestamp, arm, x_direction);
  }
}

void Hub::onArmUnsync(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : listeners_) {
    listener->onArmUnsync(myo, timestamp);
  }
}

void Hub::onUnlock(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : listeners_) {
    listener->onUnlock(myo, timestamp);
  }
}

void Hub::onLock(myo::Myo* myo, uint64_t timestamp) {
  for (auto listener : listeners_) {
    listener->onLock(myo, timestamp);
  }
}

void Hub::onPose(myo::Myo* myo, uint64_t timestamp, const myo::Pose& pose) {
  for (auto listener : listeners_) {
    listener->onPose(myo, timestamp, pose);
  }
}

void Hub::onOrientationData(myo::Myo* myo, uint64_t timestamp,
                            const myo::Quaternion<float>& rotation) {
  for (auto listener : listeners_) {
    listener->onOrientationData(myo, timestamp, rotation);
  }
}

void Hub::onAccelerometerData(myo::Myo* myo, uint64_t timestamp,
                              const myo::Vector3<float>& accel) {
  for (auto listener : listeners_) {
    listener->onAccelerometerData(myo, timestamp, accel);
  }
}

void Hub::onGyroscopeData(myo::Myo* myo, uint64_t timestamp,
                          const myo::Vector3<float>& gyro) {
  for (auto listener : listeners_) {
    listener->onGyroscopeData(myo, timestamp, gyro);
  }
}

void Hub::onRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi) {
  for (auto listener : listeners_) {
    listener->onRssi(myo, timestamp, rssi);
  }
}

void Hub::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg) {
  for (auto listener : listeners_) {
    listener->onEmgData(myo, timestamp, emg);
  }
}

void Hub::detectAndTriggerPose() {
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
