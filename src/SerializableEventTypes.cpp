#include "EventTypes.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

BOOST_SERIALIZATION_ASSUME_ABSTRACT(MyoSim::Event);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(MyoSim::MyoEvent);

BOOST_CLASS_EXPORT(MyoSim::onPeriodicEvent);
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
BOOST_CLASS_EXPORT(MyoSim::onBatteryLevelReceivedEvent);
BOOST_CLASS_EXPORT(MyoSim::onEmgDataEvent);
BOOST_CLASS_EXPORT(MyoSim::onWarmupCompletedEvent);

namespace {
///////////////////////////////////////////////////////////////////////////////
//               Structs for stealing private member variables               //
// http://bloglitb.blogspot.com/2011/12/access-to-private-members-safer.html //
///////////////////////////////////////////////////////////////////////////////
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

namespace boost {
namespace serialization {
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

////////////////////////////////////////////////
// Functions for serializing grouping structs //
////////////////////////////////////////////////
// Event
/*
template <class Archive>
void serialize(Archive& ar, MyoSim::Event& event, const unsigned int version) {
}
*/
// MyoEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::MyoEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "Event", boost::serialization::base_object<MyoSim::Event>(event));
  ar & BOOST_SERIALIZATION_NVP(event.myo_index);
  ar & BOOST_SERIALIZATION_NVP(event.timestamp);
}
// onPeriodicEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onPeriodicEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "Event", boost::serialization::base_object<MyoSim::Event>(event));
}

/////////////////////////////////////////////////
// Functions for serializing myo event structs //
/////////////////////////////////////////////////
// onPairEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onPairEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.firmware_version);
}
// onUnpairEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onUnpairEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
}
// onConnectEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onConnectEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.firmware_version);
}
// onDisconnectEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onDisconnectEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
}
// onArmSyncEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onArmSyncEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.arm);
  ar & BOOST_SERIALIZATION_NVP(event.x_direction);
  ar & BOOST_SERIALIZATION_NVP(event.rotation);
  ar & BOOST_SERIALIZATION_NVP(event.warmup_state);
}
// onArmUnsyncEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onArmUnsyncEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
}
// onUnlockEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onUnlockEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
}
// onLockEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onLockEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
}
// onPoseEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onPoseEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.pose);
}
// onOrientationDataEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onOrientationDataEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.rotation);
}
// onAccelerometerDataEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onAccelerometerDataEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.accel);
}
// onGyroscopeDataEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onGyroscopeDataEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.gyro);
}
// onRssiEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onRssiEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.rssi);
}
// onBatteryLevelReceivedEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onBatteryLevelReceivedEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.level);
}
// onEmgDataEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onEmgDataEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.emg);
}
// onWarmupCompletedEvent
template <class Archive>
void serialize(Archive& ar, MyoSim::onWarmupCompletedEvent& event, const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<MyoSim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.warmup_result);
}

}
}
