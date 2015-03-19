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
  MyoEvent(int myo_index, uint64_t timestamp)
      : myo_index(myo_index), timestamp(timestamp) {}
  virtual ~MyoEvent() {}

  int myo_index;
  uint64_t timestamp;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(myo_index);
    ar & BOOST_SERIALIZATION_NVP(timestamp);
  }
};
BOOST_SERIALIZATION_ASSUME_ABSTRACT(MyoEvent);
// This struct is used to group events that occured in the same Hub::run() loop.
struct EventLoopGroup {
  // TODO: consider changing this to vector<unique_ptr<...>>
  std::vector<std::shared_ptr<MyoEvent>> group;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(group);
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
  onPairEvent(int myo_index, uint64_t timestamp,
              const myo::FirmwareVersion& firmware_version)
      : MyoEvent(myo_index, timestamp), firmware_version(firmware_version) {}

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
  onUnpairEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
  }
};
// myo::DeviceListener::onConnect
struct onConnectEvent : MyoEvent {
  onConnectEvent() {}
  onConnectEvent(int myo_index, uint64_t timestamp,
                 const myo::FirmwareVersion& firmware_version)
      : MyoEvent(myo_index, timestamp), firmware_version(firmware_version) {}

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
  onDisconnectEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
  }
};
// myo::DeviceListener::onArmSync
struct onArmSyncEvent : MyoEvent {
  onArmSyncEvent() {}
  onArmSyncEvent(int myo_index, uint64_t timestamp, myo::Arm arm,
                 myo::XDirection x_direction)
      : MyoEvent(myo_index, timestamp), arm(arm), x_direction(x_direction) {}

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
  onArmUnsyncEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
  }
};
// myo::DeviceListener::onUnlock
struct onUnlockEvent : MyoEvent {
  onUnlockEvent() {}
  onUnlockEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
  }
};
// myo::DeviceListener::onLock
struct onLockEvent : MyoEvent {
  onLockEvent() {}
  onLockEvent(int myo_index, uint64_t timestamp)
      : MyoEvent(myo_index, timestamp) {}

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MyoEvent);
  }
};
// myo::DeviceListener::onPose
struct onPoseEvent : MyoEvent {
  onPoseEvent() {}
  onPoseEvent(int myo_index, uint64_t timestamp, const myo::Pose& pose)
      : MyoEvent(myo_index, timestamp), pose(pose) {}

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
  onOrientationDataEvent(int myo_index, uint64_t timestamp,
                         const myo::Quaternion<float>& rotation)
      : MyoEvent(myo_index, timestamp), rotation(rotation) {}

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
  onAccelerometerDataEvent(int myo_index, uint64_t timestamp,
                           const myo::Vector3<float>& accel)
      : MyoEvent(myo_index, timestamp), accel(accel) {}

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
  onGyroscopeDataEvent(int myo_index, uint64_t timestamp,
                       const myo::Vector3<float>& gyro)
      : MyoEvent(myo_index, timestamp), gyro(gyro) {}

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
  onRssiEvent(int myo_index, uint64_t timestamp, int8_t rssi)
      : MyoEvent(myo_index, timestamp), rssi(rssi) {}

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
  onEmgDataEvent(int myo_index, uint64_t timestamp, const int8_t* const emg_ptr)
      : MyoEvent(myo_index, timestamp) {
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
