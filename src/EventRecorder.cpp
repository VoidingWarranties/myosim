#include "EventRecorder.h"

namespace myosim {
EventRecorder::EventTypes operator|(EventRecorder::EventTypes lhs,
                                    EventRecorder::EventTypes rhs) {
  return static_cast<EventRecorder::EventTypes>(static_cast<int>(lhs) |
                                                static_cast<int>(rhs));
}

EventRecorder::EventRecorder(EventTypes event_types)
    : event_types_(event_types) {}

void EventRecorder::onPeriodic() {
  events_.push_back(std::make_shared<PeriodicEvent>());
}

EventQueue EventRecorder::getEventQueue() const { return events_; }

void EventRecorder::onPair(myo::Myo* myo, uint64_t timestamp,
                           myo::FirmwareVersion firmwareVersion) {
  if (!(event_types_ & PAIR)) return;
  events_.push_back(std::make_shared<PairEvent>(GetOrAddMyoIndex(myo),
                                                timestamp, firmwareVersion));
}

void EventRecorder::onUnpair(myo::Myo* myo, uint64_t timestamp) {
  if (!(event_types_ & UNPAIR)) return;
  events_.push_back(std::make_shared<UnpairEvent>(GetOrAddMyoIndex(myo),
                                                  timestamp));
}

void EventRecorder::onConnect(myo::Myo* myo, uint64_t timestamp,
                              myo::FirmwareVersion firmwareVersion) {
  if (!(event_types_ & CONNECT)) return;
  events_.push_back(std::make_shared<ConnectEvent>(GetOrAddMyoIndex(myo),
                                                   timestamp, firmwareVersion));
}

void EventRecorder::onDisconnect(myo::Myo* myo, uint64_t timestamp) {
  if (!(event_types_ & DISCONNECT)) return;
  events_.push_back(std::make_shared<DisconnectEvent>(GetOrAddMyoIndex(myo),
                                                      timestamp));
}

void EventRecorder::onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm,
                              myo::XDirection xDirection, float rotation,
                              myo::WarmupState warmupState) {
  if (!(event_types_ & ARM_SYNC)) return;
  events_.push_back(std::make_shared<ArmSyncEvent>(GetOrAddMyoIndex(myo),
                                                   timestamp, arm, xDirection,
                                                   rotation, warmupState));
}

void EventRecorder::onArmUnsync(myo::Myo* myo, uint64_t timestamp) {
  if (!(event_types_ & ARM_UNSYNC)) return;
  events_.push_back(std::make_shared<ArmUnsyncEvent>(GetOrAddMyoIndex(myo),
                                                     timestamp));
}

void EventRecorder::onUnlock(myo::Myo* myo, uint64_t timestamp) {
  if (!(event_types_ & UNLOCK)) return;
  events_.push_back(std::make_shared<UnlockEvent>(GetOrAddMyoIndex(myo),
                                                  timestamp));
}

void EventRecorder::onLock(myo::Myo* myo, uint64_t timestamp) {
  if (!(event_types_ & LOCK)) return;
  events_.push_back(std::make_shared<LockEvent>(GetOrAddMyoIndex(myo),
                                                timestamp));
}

void EventRecorder::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {
  if (!(event_types_ & POSE)) return;
  events_.push_back(std::make_shared<PoseEvent>(GetOrAddMyoIndex(myo),
                                                timestamp, pose));
}

void EventRecorder::onOrientationData(myo::Myo* myo, uint64_t timestamp,
                                      const myo::Quaternion<float>& rotation) {
  if (!(event_types_ & ORIENTATION_DATA)) return;
  events_.push_back(std::make_shared<OrientationDataEvent>(GetOrAddMyoIndex(myo),
                                                           timestamp, rotation));
}

void EventRecorder::onAccelerometerData(myo::Myo* myo, uint64_t timestamp,
                                        const myo::Vector3<float>& accel) {
  if (!(event_types_ & ACCELEROMETER_DATA)) return;
  events_.push_back(std::make_shared<AccelerometerDataEvent>(GetOrAddMyoIndex(myo),
                                                             timestamp, accel));
}

void EventRecorder::onGyroscopeData(myo::Myo* myo, uint64_t timestamp,
                                    const myo::Vector3<float>& gyro) {
  if (!(event_types_ & GYROSCOPE_DATA)) return;
  events_.push_back(std::make_shared<GyroscopeDataEvent>(GetOrAddMyoIndex(myo),
                                                         timestamp, gyro));
}

void EventRecorder::onRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi) {
  if (!(event_types_ & RSSI)) return;
  events_.push_back(std::make_shared<RssiEvent>(GetOrAddMyoIndex(myo),
                                                timestamp, rssi));
}

void EventRecorder::onBatteryLevelReceived(myo::Myo* myo, uint64_t timestamp,
                                           uint8_t level) {
  if (!(event_types_ & BATTERY_LEVEL_RECEIVED)) return;
  events_.push_back(std::make_shared<BatteryLevelReceivedEvent>(GetOrAddMyoIndex(myo),
                                                                timestamp, level));
}

void EventRecorder::onEmgData(myo::Myo* myo, uint64_t timestamp,
                              const int8_t* const emg) {
  if (!(event_types_ & EMG)) return;
  events_.push_back(std::make_shared<EmgDataEvent>(GetOrAddMyoIndex(myo),
                                                   timestamp, emg));
}

void EventRecorder::onWarmupCompleted(myo::Myo* myo, uint64_t timestamp,
                                      myo::WarmupResult warmupResult) {
  if (!(event_types_ & WARMUP_COMPLETED)) return;
  events_.push_back(std::make_shared<WarmupCompletedEvent>(GetOrAddMyoIndex(myo),
                                                           timestamp,
                                                           warmupResult));
}

int EventRecorder::GetOrAddMyoIndex(myo::Myo* myo) {
  if (myo_indicies_.count(myo) != 0) return myo_indicies_[myo];
  return myo_indicies_[myo] = myo_indicies_.size();
}
} // namespace myosim
