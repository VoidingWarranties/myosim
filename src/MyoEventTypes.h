#pragma once

#include <myo/myo.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/export.hpp>
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
  ar & boost::serialization::make_nvp("major", fv.firmwareVersionMajor);
  ar & boost::serialization::make_nvp("minor", fv.firmwareVersionMinor);
  ar & boost::serialization::make_nvp("patch", fv.firmwareVersionPatch);
  ar & boost::serialization::make_nvp("hardwareRev",
                                      fv.firmwareVersionHardwareRev);
}
// myo::Pose
template <class Archive>
void serialize(Archive& ar, myo::Pose& pose, const unsigned int version) {
  ar & boost::serialization::make_nvp("type", pose.*get(Pose_type()));
}
// myo::Quaternion<T>
template <class Archive, class T>
void serialize(Archive& ar, myo::Quaternion<T>& quat,
               const unsigned int version) {
  ar & boost::serialization::make_nvp("x", quat.*get(Quaternion_x<T>()));
  ar & boost::serialization::make_nvp("y", quat.*get(Quaternion_y<T>()));
  ar & boost::serialization::make_nvp("z", quat.*get(Quaternion_z<T>()));
  ar & boost::serialization::make_nvp("w", quat.*get(Quaternion_w<T>()));
}
// myo::Vector3<T>
template <class Archive, class T>
void serialize(Archive& ar, myo::Vector3<T>& vec, const unsigned int version) {
  ar & boost::serialization::make_nvp("data", vec.*get(Vector3_data<T>()));
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
  MyoEvent() {}
  MyoEvent(uint64_t timestamp) : timestamp(timestamp) {}
  virtual ~MyoEvent() {}

  uint64_t timestamp;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(timestamp);
  }
};
BOOST_SERIALIZATION_ASSUME_ABSTRACT(MyoEvent);
// This struct is used to group events that occured in the same Hub::run() loop.
struct EventLoopGroup {
  // TODO: consider changing this to vector<unique_ptr<...>>
  // TODO: change name of events to event_group
  std::vector<std::shared_ptr<MyoEvent>> events;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(events);
  }
};
// Used to group EventLoopGroups together. This represents all of the events
// recorded in one Myo session.
struct EventSession {
  std::vector<EventLoopGroup> events;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(events);
  }
};

////////////////////////////////////////
// Structs for storing raw Myo events //
////////////////////////////////////////
// myo::DeviceListener::onPair
struct onPairEvent : MyoEvent {
  onPairEvent() {}
  onPairEvent(uint64_t timestamp, const myo::FirmwareVersion& firmware_version)
      : MyoEvent(timestamp), firmware_version(firmware_version) {}

  myo::FirmwareVersion firmware_version;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
    ar & BOOST_SERIALIZATION_NVP(firmware_version);
  }
};
// myo::DeviceListener::onUnpair
struct onUnpairEvent : MyoEvent {
  onUnpairEvent() {}
  onUnpairEvent(uint64_t timestamp) : MyoEvent(timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
  }
};
// myo::DeviceListener::onConnect
struct onConnectEvent : MyoEvent {
  onConnectEvent() {}
  onConnectEvent(uint64_t timestamp,
                 const myo::FirmwareVersion& firmware_version)
      : MyoEvent(timestamp), firmware_version(firmware_version) {}

  myo::FirmwareVersion firmware_version;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
    ar & BOOST_SERIALIZATION_NVP(firmware_version);
  }
};
// myo::DeviceListener::onDisconnect
struct onDisconnectEvent : MyoEvent {
  onDisconnectEvent() {}
  onDisconnectEvent(uint64_t timestamp) : MyoEvent(timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
  }
};
// myo::DeviceListener::onArmSync
struct onArmSyncEvent : MyoEvent {
  onArmSyncEvent() {}
  onArmSyncEvent(uint64_t timestamp, myo::Arm arm, myo::XDirection x_direction)
      : MyoEvent(timestamp), arm(arm), x_direction(x_direction) {}

  myo::Arm arm;
  myo::XDirection x_direction;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
    ar & BOOST_SERIALIZATION_NVP(arm);
    ar & BOOST_SERIALIZATION_NVP(x_direction);
  }
};
// myo::DeviceListener::onArmUnsync
struct onArmUnsyncEvent : MyoEvent {
  onArmUnsyncEvent() {}
  onArmUnsyncEvent(uint64_t timestamp) : MyoEvent(timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
  }
};
// myo::DeviceListener::onUnlock
struct onUnlockEvent : MyoEvent {
  onUnlockEvent() {}
  onUnlockEvent(uint64_t timestamp) : MyoEvent(timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
  }
};
// myo::DeviceListener::onLock
struct onLockEvent : MyoEvent {
  onLockEvent() {}
  onLockEvent(uint64_t timestamp) : MyoEvent(timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
  }
};
// myo::DeviceListener::onPose
struct onPoseEvent : MyoEvent {
  onPoseEvent() {}
  onPoseEvent(uint64_t timestamp, const myo::Pose& pose)
      : MyoEvent(timestamp), pose(pose) {}

  myo::Pose pose;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
    ar & BOOST_SERIALIZATION_NVP(pose);
  }
};
// myo::DeviceListener::onOrientationData
struct onOrientationDataEvent : MyoEvent {
  onOrientationDataEvent() {}
  onOrientationDataEvent(uint64_t timestamp,
                         const myo::Quaternion<float>& rotation)
      : MyoEvent(timestamp), rotation(rotation) {}

  myo::Quaternion<float> rotation;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
    ar & BOOST_SERIALIZATION_NVP(rotation);
  }
};
// myo::DeviceListener::onAccelerometerData
struct onAccelerometerDataEvent : MyoEvent {
  onAccelerometerDataEvent() {}
  onAccelerometerDataEvent(uint64_t timestamp, const myo::Vector3<float>& accel)
      : MyoEvent(timestamp), accel(accel) {}

  myo::Vector3<float> accel;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
    ar & BOOST_SERIALIZATION_NVP(accel);
  }
};
// myo::DeviceListener::onGyroscopeData
struct onGyroscopeDataEvent : MyoEvent {
  onGyroscopeDataEvent() {}
  onGyroscopeDataEvent(uint64_t timestamp, const myo::Vector3<float>& gyro)
      : MyoEvent(timestamp), gyro(gyro) {}

  myo::Vector3<float> gyro;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
    ar & BOOST_SERIALIZATION_NVP(gyro);
  }
};
// myo::DeviceListener::onRssi
struct onRssiEvent : MyoEvent {
  onRssiEvent() {}
  onRssiEvent(uint64_t timestamp, int8_t rssi)
      : MyoEvent(timestamp), rssi(rssi) {}

  int8_t rssi;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
    ar & BOOST_SERIALIZATION_NVP(rssi);
  }
};
// myo::DeviceListener::onEmgData
struct onEmgDataEvent : MyoEvent {
  onEmgDataEvent() {}
  onEmgDataEvent(uint64_t timestamp, const int8_t* const emg_ptr)
      : MyoEvent(timestamp) {
    for (std::size_t i = 0; i < 8; ++i) {
      emg[i] = emg_ptr[i];
    }
  }

  int8_t emg[8];

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
    ar & BOOST_SERIALIZATION_NVP(emg);
  }
};
}

// These must always appear after the boost/archive headers.
// Putting the macros here does not gaurantee this.
// TODO: figure out a better place to put these macros.
BOOST_CLASS_EXPORT(MyoSim::onPairEvent);
BOOST_CLASS_EXPORT(MyoSim::onUnpairEvent);
BOOST_CLASS_EXPORT(MyoSim::onConnectEvent);
BOOST_CLASS_EXPORT(MyoSim::onDisconnectEvent);
BOOST_CLASS_EXPORT(MyoSim::onArmSyncEvent);
BOOST_CLASS_EXPORT(MyoSim::onArmUnsyncEvent);
BOOST_CLASS_EXPORT(MyoSim::onUnlockEvent);
BOOST_CLASS_EXPORT(MyoSim::onLockEvent);
BOOST_CLASS_EXPORT(MyoSim::onPoseEvent);
BOOST_CLASS_EXPORT(MyoSim::onOrientationDataEvent);
BOOST_CLASS_EXPORT(MyoSim::onAccelerometerDataEvent);
BOOST_CLASS_EXPORT(MyoSim::onGyroscopeDataEvent);
BOOST_CLASS_EXPORT(MyoSim::onRssiEvent);
BOOST_CLASS_EXPORT(MyoSim::onEmgDataEvent);
