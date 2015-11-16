#pragma once

#include <myo/myo.hpp>
#include <deque>
#include <memory>

namespace MyoSim {
//////////////////////////////////////////////
// Structs for storing groups of Myo events //
//////////////////////////////////////////////
struct Event {
  Event() {}
  virtual ~Event() {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
  }
};
// Base struct to derive from for all myo events. Note that the pointer to the
// myo is not stored, as Thalmic does not provide a way to construct a Myo
// object from outside a Hub.
struct MyoEvent : Event {
  MyoEvent() {}
  MyoEvent(int myo_index, uint64_t timestamp)
      : myo_index(myo_index), timestamp(timestamp) {}
  virtual ~MyoEvent() {}

  int myo_index;
  uint64_t timestamp;
};
// Event that marks the end of a hub::run
struct onPeriodicEvent : Event {
  onPeriodicEvent() {}
};
// Used to group EventLoopGroups together. This represents all of the events
// recorded in one Myo session.
typedef std::deque<std::shared_ptr<Event>> EventQueue;

////////////////////////////////////////
// Structs for storing raw Myo events //
////////////////////////////////////////
// myo::DeviceListener::onPair
struct onPairEvent : MyoEvent {
  onPairEvent() {}
  onPairEvent(int myo_index, uint64_t timestamp,
              const myo::FirmwareVersion& firmware_version)
      : MyoEvent(myo_index, timestamp), firmware_version(firmware_version) {}

  myo::FirmwareVersion firmware_version;
};
// myo::DeviceListener::onUnpair
struct onUnpairEvent : MyoEvent {
  onUnpairEvent() {}
  onUnpairEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}
};
// myo::DeviceListener::onConnect
struct onConnectEvent : MyoEvent {
  onConnectEvent() {}
  onConnectEvent(int myo_index, uint64_t timestamp,
                 const myo::FirmwareVersion& firmware_version)
      : MyoEvent(myo_index, timestamp), firmware_version(firmware_version) {}

  myo::FirmwareVersion firmware_version;
};
// myo::DeviceListener::onDisconnect
struct onDisconnectEvent : MyoEvent {
  onDisconnectEvent() {}
  onDisconnectEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}
};
// myo::DeviceListener::onArmSync
struct onArmSyncEvent : MyoEvent {
  onArmSyncEvent() {}
  onArmSyncEvent(int myo_index, uint64_t timestamp, myo::Arm arm,
                 myo::XDirection x_direction, float rotation,
                 myo::WarmupState warmup_state)
      : MyoEvent(myo_index, timestamp),
        arm(arm),
        x_direction(x_direction),
        rotation(rotation),
        warmup_state(warmup_state) {}

  myo::Arm arm;
  myo::XDirection x_direction;
  float rotation;
  myo::WarmupState warmup_state;
};
// myo::DeviceListener::onArmUnsync
struct onArmUnsyncEvent : MyoEvent {
  onArmUnsyncEvent() {}
  onArmUnsyncEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}
};
// myo::DeviceListener::onUnlock
struct onUnlockEvent : MyoEvent {
  onUnlockEvent() {}
  onUnlockEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}
};
// myo::DeviceListener::onLock
struct onLockEvent : MyoEvent {
  onLockEvent() {}
  onLockEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}
};
// myo::DeviceListener::onPose
struct onPoseEvent : MyoEvent {
  onPoseEvent() {}
  onPoseEvent(int myo_index, uint64_t timestamp, const myo::Pose& pose)
      : MyoEvent(myo_index, timestamp), pose(pose) {}

  myo::Pose pose;
};
// myo::DeviceListener::onOrientationData
struct onOrientationDataEvent : MyoEvent {
  onOrientationDataEvent() {}
  onOrientationDataEvent(int myo_index, uint64_t timestamp,
                         const myo::Quaternion<float>& rotation)
      : MyoEvent(myo_index, timestamp), rotation(rotation) {}

  myo::Quaternion<float> rotation;
};
// myo::DeviceListener::onAccelerometerData
struct onAccelerometerDataEvent : MyoEvent {
  onAccelerometerDataEvent() {}
  onAccelerometerDataEvent(int myo_index, uint64_t timestamp,
                           const myo::Vector3<float>& accel)
      : MyoEvent(myo_index, timestamp), accel(accel) {}

  myo::Vector3<float> accel;
};
// myo::DeviceListener::onGyroscopeData
struct onGyroscopeDataEvent : MyoEvent {
  onGyroscopeDataEvent() {}
  onGyroscopeDataEvent(int myo_index, uint64_t timestamp,
                       const myo::Vector3<float>& gyro)
      : MyoEvent(myo_index, timestamp), gyro(gyro) {}

  myo::Vector3<float> gyro;
};
// myo::DeviceListener::onRssi
struct onRssiEvent : MyoEvent {
  onRssiEvent() {}
  onRssiEvent(int myo_index, uint64_t timestamp, int8_t rssi)
      : MyoEvent(myo_index, timestamp), rssi(rssi) {}

  int8_t rssi;
};
// myo::DeviceListener::onBatteryLevelReceived
struct onBatteryLevelReceivedEvent : MyoEvent {
  onBatteryLevelReceivedEvent() {}
  onBatteryLevelReceivedEvent(int myo_index, uint64_t timestamp, uint8_t level)
      : MyoEvent(myo_index, timestamp), level(level) {}

  uint8_t level;
};
// myo::DeviceListener::onEmgData
struct onEmgDataEvent : MyoEvent {
  onEmgDataEvent() {}
  onEmgDataEvent(int myo_index, uint64_t timestamp, const int8_t* const emg_ptr)
      : MyoEvent(myo_index, timestamp) {
    for (std::size_t i = 0; i < 8; ++i) {
      emg[i] = emg_ptr[i];
    }
  }

  int8_t emg[8];
};
// myo::DeviceListener::onWarmupCompleted
struct onWarmupCompletedEvent : MyoEvent {
  onWarmupCompletedEvent() {}
  onWarmupCompletedEvent(int myo_index, uint64_t timestamp,
                         myo::WarmupResult warmup_result)
      : MyoEvent(myo_index, timestamp), warmup_result(warmup_result) {}

  myo::WarmupResult warmup_result;
};
} // namespace MyoSim
