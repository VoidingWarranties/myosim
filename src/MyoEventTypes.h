#pragma once

#include <myo/myo.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

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
void serialize(Archive& ar, myo::Vector3<T>& vec,
               const unsigned int version) {
  ar & vec.*get(Vector3_data<T>());
}
}

namespace MyoSim {
////////////////////////////////////
// Structs for storing Myo events //
////////////////////////////////////
// Base struct to derive from for all myo events. Note that the pointer to the
// myo is not stored, as Thalmic does not provide a way to construct a Myo
// object from outside a Hub.
struct MyoEvent {
  uint64_t timestamp;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & timestamp;
  }
};
// myo::DeviceListener::onPose
struct onPairEvent : MyoEvent {
  myo::FirmwareVersion firmware_version;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & firmware_version;
  }
};
// myo::DeviceListener::onUnpair
struct onUnpairEvent : MyoEvent {
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
  }
};
// myo::DeviceListener::onConnect
struct onConnectEvent : MyoEvent {
  myo::FirmwareVersion firmware_version;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & firmware_version;
  }
};
// myo::DeviceListener::onDisconnect
struct onDisconnectEvent : MyoEvent {
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
  }
};
// myo::DeviceListener::onArmSync
struct onArmSyncEvent : MyoEvent {
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
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
  }
};
// myo::DeviceListener::onUnlock
struct onUnlockEvent : MyoEvent {
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
  }
};
// myo::DeviceListener::onLock
struct onLockEvent : MyoEvent {
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
  }
};
// myo::DeviceListener::onPose
struct onPoseEvent : MyoEvent {
  myo::Pose pose;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & pose;
  }
};
// myo::DeviceListener::onOrientationData
struct onOrientationDataEvent : MyoEvent {
  myo::Quaternion<float> rotation;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & rotation;
  }
};
// myo::DeviceListener::onAccelerometerData
struct onAccelerometerDataEvent : MyoEvent {
  myo::Vector3<float> accel;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & accel;
  }
};
// myo::DeviceListener::onGyroscopeData
struct onGyroscopeDataEvent : MyoEvent {
  myo::Vector3<float> gyro;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & gyro;
  }
};
// myo::DeviceListener::onRssi
struct onRssiEvent : MyoEvent {
  int8_t rssi;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & rssi;
  }
};
// myo::DeviceListener::onEmgData
struct onEmgDataEvent : MyoEvent {
  int8_t emg[8];

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & boost::serialization::base_object<MyoEvent>(*this);
    ar & emg;
  }
};
}
