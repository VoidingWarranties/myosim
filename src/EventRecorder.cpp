#include "EventRecorder.h"

using namespace MyoSim;

EventRecorder::EventRecorder(EventTypes event_types)
    : event_types_(event_types) {}

void EventRecorder::endEventLoopGroup() {
  events_.push_back(EventLoopGroup());
}

void EventRecorder::onPair(myo::Myo* myo, uint64_t timestamp,
                           myo::FirmwareVersion firmwareVersion) {
  if (!(event_types_ & PAIR)) return;
  addEvent(std::make_shared<onPairEvent>(GetOrAddMyoIndex(myo), timestamp,
                                         firmwareVersion));
}

void EventRecorder::onUnpair(myo::Myo* myo, uint64_t timestamp) {
  if (!(event_types_ & UNPAIR)) return;
  addEvent(std::make_shared<onUnpairEvent>(GetOrAddMyoIndex(myo), timestamp));
}

void EventRecorder::onConnect(myo::Myo* myo, uint64_t timestamp,
                              myo::FirmwareVersion firmwareVersion) {
  if (!(event_types_ & CONNECT)) return;
  addEvent(std::make_shared<onConnectEvent>(GetOrAddMyoIndex(myo), timestamp,
                                            firmwareVersion));
}

void EventRecorder::onDisconnect(myo::Myo* myo, uint64_t timestamp) {
  if (!(event_types_ & DISCONNECT)) return;
  addEvent(
      std::make_shared<onDisconnectEvent>(GetOrAddMyoIndex(myo), timestamp));
}

void EventRecorder::onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm,
                              myo::XDirection xDirection, float rotation,
                              myo::WarmupState warmupState) {
  if (!(event_types_ & ARM_SYNC)) return;
  addEvent(std::make_shared<onArmSyncEvent>(GetOrAddMyoIndex(myo), timestamp,
                                            arm, xDirection, rotation,
                                            warmupState));
}

void EventRecorder::onArmUnsync(myo::Myo* myo, uint64_t timestamp) {
  if (!(event_types_ & ARM_UNSYNC)) return;
  addEvent(
      std::make_shared<onArmUnsyncEvent>(GetOrAddMyoIndex(myo), timestamp));
}

void EventRecorder::onUnlock(myo::Myo* myo, uint64_t timestamp) {
  if (!(event_types_ & UNLOCK)) return;
  addEvent(std::make_shared<onUnlockEvent>(GetOrAddMyoIndex(myo), timestamp));
}

void EventRecorder::onLock(myo::Myo* myo, uint64_t timestamp) {
  if (!(event_types_ & LOCK)) return;
  addEvent(std::make_shared<onLockEvent>(GetOrAddMyoIndex(myo), timestamp));
}

void EventRecorder::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {
  if (!(event_types_ & POSE)) return;
  addEvent(
      std::make_shared<onPoseEvent>(GetOrAddMyoIndex(myo), timestamp, pose));
}

void EventRecorder::onOrientationData(myo::Myo* myo, uint64_t timestamp,
                                      const myo::Quaternion<float>& rotation) {
  if (!(event_types_ & ORIENTATION_DATA)) return;
  addEvent(std::make_shared<onOrientationDataEvent>(GetOrAddMyoIndex(myo),
                                                    timestamp, rotation));
}

void EventRecorder::onAccelerometerData(myo::Myo* myo, uint64_t timestamp,
                                        const myo::Vector3<float>& accel) {
  if (!(event_types_ & ACCELEROMETER_DATA)) return;
  addEvent(std::make_shared<onAccelerometerDataEvent>(GetOrAddMyoIndex(myo),
                                                      timestamp, accel));
}

void EventRecorder::onGyroscopeData(myo::Myo* myo, uint64_t timestamp,
                                    const myo::Vector3<float>& gyro) {
  if (!(event_types_ & GYROSCOPE_DATA)) return;
  addEvent(std::make_shared<onGyroscopeDataEvent>(GetOrAddMyoIndex(myo),
                                                  timestamp, gyro));
}

void EventRecorder::onRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi) {
  if (!(event_types_ & RSSI)) return;
  addEvent(
      std::make_shared<onRssiEvent>(GetOrAddMyoIndex(myo), timestamp, rssi));
}

void EventRecorder::onBatteryLevelReceived(myo::Myo* myo, uint64_t timestamp,
                                           uint8_t level) {
  if (!(event_types_ & BATTERY_LEVEL_RECEIVED)) return;
  addEvent(std::make_shared<onBatteryLevelReceivedEvent>(GetOrAddMyoIndex(myo),
                                                         timestamp, level));
}

void EventRecorder::onEmgData(myo::Myo* myo, uint64_t timestamp,
                              const int8_t* const emg) {
  if (!(event_types_ & EMG)) return;
  addEvent(
      std::make_shared<onEmgDataEvent>(GetOrAddMyoIndex(myo), timestamp, emg));
}

void EventRecorder::onWarmupCompleted(myo::Myo* myo, uint64_t timestamp,
                                      myo::WarmupResult warmupResult) {
  if (!(event_types_ & WARMUP_COMPLETED)) return;
  addEvent(std::make_shared<onWarmupCompletedEvent>(GetOrAddMyoIndex(myo),
                                                    timestamp, warmupResult));
}

void EventRecorder::addEvent(const std::shared_ptr<MyoEvent>& event) {
  if (events_.empty()) {
    endEventLoopGroup();
  }
  events_.back().group.push_back(event);
}

int EventRecorder::GetOrAddMyoIndex(myo::Myo* myo) {
  if (myo_indicies_.count(myo) != 0) return myo_indicies_[myo];
  return myo_indicies_[myo] = myo_indicies_.size();
}
