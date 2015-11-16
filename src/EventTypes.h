#pragma once

#include <myo/myo.hpp>
#include <deque>
#include <memory>

namespace myosim {
//////////////////////////////////////////////
// Structs for storing groups of Myo events //
//////////////////////////////////////////////
struct Event {
  Event() {}
  virtual ~Event() {}

  // This serialize function is needed in order to compile with boost
  // serialization. I tried putting it in SerializableEventTypes.cpp with all
  // the other serialize functions, but clang fails to compile
  // "boost/serialization/access.hpp:116:11: error: no member named 'serialize' in 'myosim::Event'"
  // This isn't an issue because this function doesn't use any boost macros or functions.
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {}
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
struct PeriodicEvent : Event {
  PeriodicEvent() {}
};
// Used to group EventLoopGroups together. This represents all of the events
// recorded in one Myo session.
typedef std::deque<std::shared_ptr<Event>> EventQueue;

////////////////////////////////////////
// Structs for storing raw Myo events //
////////////////////////////////////////
// myo::DeviceListener::onPair
struct PairEvent : MyoEvent {
  PairEvent() {}
  PairEvent(int myo_index, uint64_t timestamp,
            const myo::FirmwareVersion& firmware_version)
      : MyoEvent(myo_index, timestamp), firmware_version(firmware_version) {}

  myo::FirmwareVersion firmware_version;
};
// myo::DeviceListener::onUnpair
struct UnpairEvent : MyoEvent {
  UnpairEvent() {}
  UnpairEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}
};
// myo::DeviceListener::onConnect
struct ConnectEvent : MyoEvent {
  ConnectEvent() {}
  ConnectEvent(int myo_index, uint64_t timestamp,
               const myo::FirmwareVersion& firmware_version)
      : MyoEvent(myo_index, timestamp), firmware_version(firmware_version) {}

  myo::FirmwareVersion firmware_version;
};
// myo::DeviceListener::onDisconnect
struct DisconnectEvent : MyoEvent {
  DisconnectEvent() {}
  DisconnectEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}
};
// myo::DeviceListener::onArmSync
struct ArmSyncEvent : MyoEvent {
  ArmSyncEvent() {}
  ArmSyncEvent(int myo_index, uint64_t timestamp, myo::Arm arm,
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
struct ArmUnsyncEvent : MyoEvent {
  ArmUnsyncEvent() {}
  ArmUnsyncEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}
};
// myo::DeviceListener::onUnlock
struct UnlockEvent : MyoEvent {
  UnlockEvent() {}
  UnlockEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}
};
// myo::DeviceListener::onLock
struct LockEvent : MyoEvent {
  LockEvent() {}
  LockEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}
};
// myo::DeviceListener::onPose
struct PoseEvent : MyoEvent {
  PoseEvent() {}
  PoseEvent(int myo_index, uint64_t timestamp, const myo::Pose& pose)
      : MyoEvent(myo_index, timestamp), pose(pose) {}

  myo::Pose pose;
};
// myo::DeviceListener::onOrientationData
struct OrientationDataEvent : MyoEvent {
  OrientationDataEvent() {}
  OrientationDataEvent(int myo_index, uint64_t timestamp,
                       const myo::Quaternion<float>& rotation)
      : MyoEvent(myo_index, timestamp), rotation(rotation) {}

  myo::Quaternion<float> rotation;
};
// myo::DeviceListener::onAccelerometerData
struct AccelerometerDataEvent : MyoEvent {
  AccelerometerDataEvent() {}
  AccelerometerDataEvent(int myo_index, uint64_t timestamp,
                         const myo::Vector3<float>& accel)
      : MyoEvent(myo_index, timestamp), accel(accel) {}

  myo::Vector3<float> accel;
};
// myo::DeviceListener::onGyroscopeData
struct GyroscopeDataEvent : MyoEvent {
  GyroscopeDataEvent() {}
  GyroscopeDataEvent(int myo_index, uint64_t timestamp,
                     const myo::Vector3<float>& gyro)
      : MyoEvent(myo_index, timestamp), gyro(gyro) {}

  myo::Vector3<float> gyro;
};
// myo::DeviceListener::onRssi
struct RssiEvent : MyoEvent {
  RssiEvent() {}
  RssiEvent(int myo_index, uint64_t timestamp, int8_t rssi)
      : MyoEvent(myo_index, timestamp), rssi(rssi) {}

  int8_t rssi;
};
// myo::DeviceListener::onBatteryLevelReceived
struct BatteryLevelReceivedEvent : MyoEvent {
  BatteryLevelReceivedEvent() {}
  BatteryLevelReceivedEvent(int myo_index, uint64_t timestamp, uint8_t level)
      : MyoEvent(myo_index, timestamp), level(level) {}

  uint8_t level;
};
// myo::DeviceListener::onEmgData
struct EmgDataEvent : MyoEvent {
  EmgDataEvent() {}
  EmgDataEvent(int myo_index, uint64_t timestamp, const int8_t* const emg_ptr)
      : MyoEvent(myo_index, timestamp) {
    for (std::size_t i = 0; i < 8; ++i) {
      emg[i] = emg_ptr[i];
    }
  }

  int8_t emg[8];
};
// myo::DeviceListener::onWarmupCompleted
struct WarmupCompletedEvent : MyoEvent {
  WarmupCompletedEvent() {}
  WarmupCompletedEvent(int myo_index, uint64_t timestamp,
                       myo::WarmupResult warmup_result)
      : MyoEvent(myo_index, timestamp), warmup_result(warmup_result) {}

  myo::WarmupResult warmup_result;
};
} // namespace myosim
