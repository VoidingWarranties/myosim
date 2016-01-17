#pragma once

#include <myo/myo.hpp>

#include <map>

#include "EventTypes.h"

namespace myosim {
class EventRecorder : public myo::DeviceListener {
 public:
  enum EventTypes {
    PAIR                   = 1 << 0,
    UNPAIR                 = 1 << 1,
    CONNECT                = 1 << 2,
    DISCONNECT             = 1 << 3,
    ARM_SYNC               = 1 << 4,
    ARM_UNSYNC             = 1 << 5,
    UNLOCK                 = 1 << 6,
    LOCK                   = 1 << 7,
    POSE                   = 1 << 8,
    ORIENTATION_DATA       = 1 << 9,
    ACCELEROMETER_DATA     = 1 << 10,
    GYROSCOPE_DATA         = 1 << 11,
    RSSI                   = 1 << 12,
    BATTERY_LEVEL_RECEIVED = 1 << 13,
    EMG                    = 1 << 14,
    WARMUP_COMPLETED       = 1 << 15,
    ALL_EVENTS             = (1 << 16) - 1
  };

  explicit EventRecorder(EventTypes event_types);

  // Call this function after each call to Hub::run or Hub::runOnce to mark the
  // end of an event group.
  void onPeriodic();
  EventQueue getEventQueue();

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
                         myo::XDirection xDirection, float rotation,
                         myo::WarmupState warmupState) override;
  virtual void onArmUnsync(myo::Myo* myo, uint64_t timestamp) override;
  virtual void onUnlock(myo::Myo* myo, uint64_t timestamp) override;
  virtual void onLock(myo::Myo* myo, uint64_t timestamp) override;
  virtual void onPose(myo::Myo* myo, uint64_t timestamp,
                      myo::Pose pose) override;
  virtual void onOrientationData(myo::Myo* myo, uint64_t timestamp,
                                 const myo::Quaternion<float>& rotation) override;
  virtual void onAccelerometerData(myo::Myo* myo, uint64_t timestamp,
                                   const myo::Vector3<float>& accel) override;
  virtual void onGyroscopeData(myo::Myo* myo, uint64_t timestamp,
                               const myo::Vector3<float>& gyro) override;
  virtual void onRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi) override;
  virtual void onBatteryLevelReceived(myo::Myo* myo, uint64_t timestamp,
                                      uint8_t level) override;
  virtual void onEmgData(myo::Myo* myo, uint64_t timestamp,
                         const int8_t* emg) override;
  virtual void onWarmupCompleted(myo::Myo* myo, uint64_t timestamp,
                                 myo::WarmupResult warmupResult) override;

 private:
  size_t getMyoIndex(myo::Myo* myo);

  const EventTypes event_types_;
  EventQueue events_;
  std::map<myo::Myo*, size_t> myo_indicies_;
};

EventRecorder::EventTypes operator|(EventRecorder::EventTypes a, EventRecorder::EventTypes b);
} // namespace myosim
