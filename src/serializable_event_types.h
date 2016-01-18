/* This header file provides the necessary functions for serializing event types
 * with boost's serialization library. Event serialization requires serializing
 * all the relevant member variables in myo classes. Some of these member
 * variables are private and therefore require a kludge to access them. This is
 * necessary only for myo::Pose, myo::Quaternion, and myo::Vector.
 */

#pragma once

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "event_types.h"

BOOST_SERIALIZATION_ASSUME_ABSTRACT(myosim::Event);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(myosim::MyoEvent);

namespace {
////////////////////////////////////////////////////////////////////////////////
//               Structs for stealing private member variables                //
// http://bloglitb.blogspot.com/2011/12/access-to-private-members-safer.html  //
//                                                                            //
// Some myo classes have private member variables. Namely myo::Pose,          //
// myo::Quaternion, and myo::Vector. These structs provide access to specific //
// member variables of these classes. For example, to access the private      //
// member variable myo::Pose::type:                                           //
//     auto type = pose.*get(Pose_type())                                     //
////////////////////////////////////////////////////////////////////////////////
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
} // namespace

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
  // Steal private member variable pose.type.
  ar & boost::serialization::make_nvp("type", pose.*get(Pose_type()));
}
// myo::Quaternion<T>
template <class Archive, class T>
void serialize(Archive& ar, myo::Quaternion<T>& quat,
               const unsigned int version) {
  // Steal private member variables quat.x/y/z/w.
  ar & boost::serialization::make_nvp("x", quat.*get(Quaternion_x<T>()));
  ar & boost::serialization::make_nvp("y", quat.*get(Quaternion_y<T>()));
  ar & boost::serialization::make_nvp("z", quat.*get(Quaternion_z<T>()));
  ar & boost::serialization::make_nvp("w", quat.*get(Quaternion_w<T>()));
}
// myo::Vector3<T>
template <class Archive, class T>
void serialize(Archive& ar, myo::Vector3<T>& vec, const unsigned int version) {
  // Steal private member variable vec.data.
  ar & boost::serialization::make_nvp("data", vec.*get(Vector3_data<T>()));
}

////////////////////////////////////////////////
// Functions for serializing grouping structs //
////////////////////////////////////////////////
// Event
template <class Archive>
void serialize(Archive& ar, myosim::Event& event, const unsigned int version) {}
// MyoEvent
template <class Archive>
void serialize(Archive& ar, myosim::MyoEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "Event", boost::serialization::base_object<myosim::Event>(event));
  ar & BOOST_SERIALIZATION_NVP(event.myo_index);
  ar & BOOST_SERIALIZATION_NVP(event.timestamp);
}
// PeriodicEvent
template <class Archive>
void serialize(Archive& ar, myosim::PeriodicEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "Event", boost::serialization::base_object<myosim::Event>(event));
}
// EventQueue
template <class Archive>
void serialize(Archive& ar, myosim::EventQueue& event_queue,
               const unsigned int version) {
  ar & BOOST_SERIALIZATION_NVP(event_queue.num_myos);
  ar & BOOST_SERIALIZATION_NVP(event_queue.queue);
}

/////////////////////////////////////////////////
// Functions for serializing myo event structs //
/////////////////////////////////////////////////
// PairEvent
template <class Archive>
void serialize(Archive& ar, myosim::PairEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.firmware_version);
}
// UnpairEvent
template <class Archive>
void serialize(Archive& ar, myosim::UnpairEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
}
// ConnectEvent
template <class Archive>
void serialize(Archive& ar, myosim::ConnectEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.firmware_version);
}
// DisconnectEvent
template <class Archive>
void serialize(Archive& ar, myosim::DisconnectEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
}
// ArmSyncEvent
template <class Archive>
void serialize(Archive& ar, myosim::ArmSyncEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.arm);
  ar & BOOST_SERIALIZATION_NVP(event.x_direction);
  ar & BOOST_SERIALIZATION_NVP(event.rotation);
  ar & BOOST_SERIALIZATION_NVP(event.warmup_state);
}
// ArmUnsyncEvent
template <class Archive>
void serialize(Archive& ar, myosim::ArmUnsyncEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
}
// UnlockEvent
template <class Archive>
void serialize(Archive& ar, myosim::UnlockEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
}
// LockEvent
template <class Archive>
void serialize(Archive& ar, myosim::LockEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
}
// PoseEvent
template <class Archive>
void serialize(Archive& ar, myosim::PoseEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.pose);
}
// OrientationDataEvent
template <class Archive>
void serialize(Archive& ar, myosim::OrientationDataEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.rotation);
}
// AccelerometerDataEvent
template <class Archive>
void serialize(Archive& ar, myosim::AccelerometerDataEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.accel);
}
// GyroscopeDataEvent
template <class Archive>
void serialize(Archive& ar, myosim::GyroscopeDataEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.gyro);
}
// RssiEvent
template <class Archive>
void serialize(Archive& ar, myosim::RssiEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.rssi);
}
// BatteryLevelReceivedEvent
template <class Archive>
void serialize(Archive& ar, myosim::BatteryLevelReceivedEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.level);
}
// EmgDataEvent
template <class Archive>
void serialize(Archive& ar, myosim::EmgDataEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.emg);
}
// WarmupCompletedEvent
template <class Archive>
void serialize(Archive& ar, myosim::WarmupCompletedEvent& event,
               const unsigned int version) {
  ar & boost::serialization::make_nvp(
      "MyoEvent", boost::serialization::base_object<myosim::MyoEvent>(event));
  ar & BOOST_SERIALIZATION_NVP(event.warmup_result);
}
} // namespace serialization
} // namespace boost
