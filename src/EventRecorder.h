#pragma once

#include <myo/myo.hpp>

#include <map>

#include "EventTypes.h"

namespace MyoSim {
class EventRecorder : public myo::DeviceListener {
 public:
  enum EventTypes {
    PAIR                = 1 << 0,
    UNPAIR              = 1 << 1,
    CONNECT             = 1 << 2,
    DISCONNECT          = 1 << 3,
    ARM_SYNC            = 1 << 4,
    ARM_UNSYNC          = 1 << 5,
    UNLOCK              = 1 << 6,
    LOCK                = 1 << 7,
    POSE                = 1 << 8,
    ORIENTATION_DATA    = 1 << 9,
    ACCELEROMETER_DATA  = 1 << 10,
    GYROSCOPE_DATA      = 1 << 11,
    RSSI                = 1 << 12,
    EMG                 = 1 << 13,
    ALL_EVENT_TYPES     = (1 << 14) - 1
  };

  explicit EventRecorder(EventTypes event_types);

  // Call this function after each call to Hub::run or Hub::runOnce.
  void endEventLoopGroup();

  EventSession getEventSession() const { return events_; }

  ///////////////////////////////////////////////////////////////////////
  // Virtual event functions that override functions in DeviceListener //
  ///////////////////////////////////////////////////////////////////////
  virtual void onPair(myo::Myo* myo, uint64_t timestamp,
                      myo::FirmwareVersion firmwareVersion) override;
  virtual void onUnpair(myo::Myo* myo, uint64_t timestamp) override;
  virtual void onConnect(myo::Myo* myo, uint64_t timestamp,
                         myo::FirmwareVersion firmwareVersion) override;
  virtual void onDisconnect(myo::Myo* myo, uint64_t timestamp) override;
  virtual void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm,
                         myo::XDirection xDirection) override;
  virtual void onArmUnsync(myo::Myo* myo, uint64_t timestamp) override;
  virtual void onUnlock(myo::Myo* myo, uint64_t timestamp) override;
  virtual void onLock(myo::Myo* myo, uint64_t timestamp) override;
  virtual void onPose(myo::Myo* myo, uint64_t timestamp,
                      myo::Pose pose) override;
  virtual void onOrientationData(
      myo::Myo* myo, uint64_t timestamp,
      const myo::Quaternion<float>& rotation) override;
  virtual void onAccelerometerData(myo::Myo* myo, uint64_t timestamp,
                                   const myo::Vector3<float>& accel) override;
  virtual void onGyroscopeData(myo::Myo* myo, uint64_t timestamp,
                               const myo::Vector3<float>& gyro) override;
  virtual void onRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi) override;
  virtual void onEmgData(myo::Myo* myo, uint64_t timestamp,
                         const int8_t* emg) override;

 private:
  // Add an event to the current event group.
  void addEvent(const std::shared_ptr<MyoEvent>& event);

  int GetOrAddMyoIndex(myo::Myo* myo);

  EventTypes event_types_;
  EventSession events_;
  std::map<myo::Myo*, int> myo_indicies_;
};

EventRecorder::EventRecorder(EventTypes event_types)
    : event_types_(event_types) {}

void EventRecorder::endEventLoopGroup() {
  events_.events.push_back(EventLoopGroup());
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
                              myo::XDirection xDirection) {
  if (!(event_types_ & ARM_SYNC)) return;
  addEvent(std::make_shared<onArmSyncEvent>(GetOrAddMyoIndex(myo), timestamp,
                                            arm, xDirection));
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

void EventRecorder::onEmgData(myo::Myo* myo, uint64_t timestamp,
                              const int8_t* const emg) {
  if (!(event_types_ & EMG)) return;
  addEvent(
      std::make_shared<onEmgDataEvent>(GetOrAddMyoIndex(myo), timestamp, emg));
}

void EventRecorder::addEvent(const std::shared_ptr<MyoEvent>& event) {
  if (events_.events.empty()) {
    endEventLoopGroup();
  }
  events_.events.back().group.push_back(event);
}

int EventRecorder::GetOrAddMyoIndex(myo::Myo* myo) {
  if (myo_indicies_.count(myo) != 0) return myo_indicies_[myo];
  return myo_indicies_[myo] = myo_indicies_.size();
}
}
