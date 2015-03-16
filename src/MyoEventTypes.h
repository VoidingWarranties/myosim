#pragma once

#include <myo/myo.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <vector>

namespace {
///////////////////////////////////////////////////
// Structs for stealing private member variables //
///////////////////////////////////////////////////
template <typename Tag, typename Tag::type M>
struct Rob {
  friend typename Tag::type get(Tag) { return M; }
};
// Steal private members of Pose.
struct Pose_type {
  typedef myo::Pose::Type myo::Pose::*type;
  friend type get(Pose_type);
};
template struct Rob<Pose_type, &myo::Pose::_type>;
// Steal private members of Quaternion<float>.
template <class T>
struct Quaternion_x {
  typedef T myo::Quaternion<T>::*type;
  friend type get(Quaternion_x);
};
template struct Rob<Quaternion_x<float>, &myo::Quaternion<float>::_x>;
template <class T>
struct Quaternion_y {
  typedef T myo::Quaternion<T>::*type;
  friend type get(Quaternion_y);
};
template struct Rob<Quaternion_y<float>, &myo::Quaternion<float>::_y>;
template <class T>
struct Quaternion_z {
  typedef T myo::Quaternion<T>::*type;
  friend type get(Quaternion_z);
};
template struct Rob<Quaternion_z<float>, &myo::Quaternion<float>::_z>;
template <class T>
struct Quaternion_w {
  typedef T myo::Quaternion<T>::*type;
  friend type get(Quaternion_w);
};
template struct Rob<Quaternion_w<float>, &myo::Quaternion<float>::_w>;
// Steal private members of Vector3<float>.
template <class T>
struct Vector3_data {
  typedef T (myo::Vector3<T>::*type)[3];
  friend type get(Vector3_data);
};
template struct Rob<Vector3_data<float>, &myo::Vector3<float>::_data>;
}

namespace myo {
/////////////////////////////////////////////
// Serialization functions for myo classes //
/////////////////////////////////////////////
// myo::FirmwareVersion
template <class Archive>
void serialize(Archive& ar, myo::FirmwareVersion& fv,
               const unsigned int version) {
  ar& fv.firmwareVersionMajor;
  ar& fv.firmwareVersionMinor;
  ar& fv.firmwareVersionPatch;
  ar& fv.firmwareVersionHardwareRev;
}
// myo::Pose
template <class Archive>
void serialize(Archive& ar, myo::Pose& pose, const unsigned int version) {
  ar& pose.*get(Pose_type());
}
// myo::Quaternion<T>
template <class Archive, class T>
void serialize(Archive& ar, myo::Quaternion<T>& quat,
               const unsigned int version) {
  ar& quat.*get(Quaternion_x<T>());
  ar& quat.*get(Quaternion_y<T>());
  ar& quat.*get(Quaternion_z<T>());
  ar& quat.*get(Quaternion_w<T>());
}
// myo::Vector3<T>
template <class Archive, class T>
void serialize(Archive& ar, myo::Vector3<T>& vec, const unsigned int version) {
  ar & vec.*get(Vector3_data<T>());
}
}

namespace MyoSim {
//////////////////////////////////////////////
// Structs for storing groups of Myo events //
//////////////////////////////////////////////
// Base struct to derive from for all myo events. Note that the pointer to the
// myo is not stored, as Thalmic does not provide a way to construct a Myo
// object from outside a Hub.
struct MyoEvent {
  MyoEvent(uint64_t timestamp) : timestamp(timestamp) {}

  uint64_t timestamp;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & timestamp;
  }
};
// This struct is used to group events that occured in the same Hub::run() loop.
struct EventLoopGroup {
  // TODO: consider changing this to vector<unique_ptr<...>>
  std::vector<std::shared_ptr<MyoEvent>> events;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & events;
  }
};
// Used to group EventLoopGroups together. This represents all of the events
// recorded in one Myo session.
struct EventSession {
  std::vector<EventLoopGroup> events;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & events;
  }
};

////////////////////////////////////////
// Structs for storing raw Myo events //
////////////////////////////////////////
// myo::DeviceListener::onPose
struct onPairEvent : MyoEvent {
  onPairEvent(uint64_t timestamp, const myo::FirmwareVersion& firmware_version)
      : MyoEvent(timestamp), firmware_version(firmware_version) {}

  myo::FirmwareVersion firmware_version;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & firmware_version;
  }
};
// myo::DeviceListener::onUnpair
struct onUnpairEvent : MyoEvent {
  onUnpairEvent(uint64_t timestamp) : MyoEvent(timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
  }
};
// myo::DeviceListener::onConnect
struct onConnectEvent : MyoEvent {
  onConnectEvent(uint64_t timestamp,
                 const myo::FirmwareVersion& firmware_version)
      : MyoEvent(timestamp), firmware_version(firmware_version) {}

  myo::FirmwareVersion firmware_version;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & firmware_version;
  }
};
// myo::DeviceListener::onDisconnect
struct onDisconnectEvent : MyoEvent {
  onDisconnectEvent(uint64_t timestamp) : MyoEvent(timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
  }
};
// myo::DeviceListener::onArmSync
struct onArmSyncEvent : MyoEvent {
  onArmSyncEvent(uint64_t timestamp, myo::Arm arm, myo::XDirection x_direction)
      : MyoEvent(timestamp), arm(arm), x_direction(x_direction) {}

  myo::Arm arm;
  myo::XDirection x_direction;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & arm;
    ar & x_direction;
  }
};
// myo::DeviceListener::onArmUnsync
struct onArmUnsyncEvent : MyoEvent {
  onArmUnsyncEvent(uint64_t timestamp) : MyoEvent(timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
  }
};
// myo::DeviceListener::onUnlock
struct onUnlockEvent : MyoEvent {
  onUnlockEvent(uint64_t timestamp) : MyoEvent(timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
  }
};
// myo::DeviceListener::onLock
struct onLockEvent : MyoEvent {
  onLockEvent(uint64_t timestamp) : MyoEvent(timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
  }
};
// myo::DeviceListener::onPose
struct onPoseEvent : MyoEvent {
  onPoseEvent(uint64_t timestamp, const myo::Pose& pose)
      : MyoEvent(timestamp), pose(pose) {}

  myo::Pose pose;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & pose;
  }
};
// myo::DeviceListener::onOrientationData
struct onOrientationDataEvent : MyoEvent {
  onOrientationDataEvent(uint64_t timestamp,
                         const myo::Quaternion<float>& rotation)
      : MyoEvent(timestamp), rotation(rotation) {}

  myo::Quaternion<float> rotation;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & rotation;
  }
};
// myo::DeviceListener::onAccelerometerData
struct onAccelerometerDataEvent : MyoEvent {
  onAccelerometerDataEvent(uint64_t timestamp, const myo::Vector3<float>& accel)
      : MyoEvent(timestamp), accel(accel) {}

  myo::Vector3<float> accel;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & accel;
  }
};
// myo::DeviceListener::onGyroscopeData
struct onGyroscopeDataEvent : MyoEvent {
  onGyroscopeDataEvent(uint64_t timestamp, const myo::Vector3<float>& gyro)
      : MyoEvent(timestamp), gyro(gyro) {}

  myo::Vector3<float> gyro;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & gyro;
  }
};
// myo::DeviceListener::onRssi
struct onRssiEvent : MyoEvent {
  onRssiEvent(uint64_t timestamp, int8_t rssi)
      : MyoEvent(timestamp), rssi(rssi) {}

  int8_t rssi;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & rssi;
  }
};
// myo::DeviceListener::onEmgData
struct onEmgDataEvent : MyoEvent {
  onEmgDataEvent(uint64_t timestamp, const int8_t* const emg_ptr)
      : MyoEvent(timestamp) {
    for (std::size_t i = 0; i < 8; ++i) {
      emg[i] = emg_ptr[i];
    }
  }

  int8_t emg[8];

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & emg;
  }
};
}
