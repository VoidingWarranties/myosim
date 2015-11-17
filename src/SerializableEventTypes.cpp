#include <boost/serialization/export.hpp>
// Include headers for all archive classes that could be used here. This must be
// done because BOOST_CLASS_EXPORT must appear after any archive class headers.
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include "EventTypes.h"

BOOST_CLASS_EXPORT(myosim::PeriodicEvent);
BOOST_CLASS_EXPORT(myosim::EventQueue);
BOOST_CLASS_EXPORT(myosim::PairEvent);
BOOST_CLASS_EXPORT(myosim::UnpairEvent);
BOOST_CLASS_EXPORT(myosim::ConnectEvent);
BOOST_CLASS_EXPORT(myosim::DisconnectEvent);
BOOST_CLASS_EXPORT(myosim::ArmSyncEvent);
BOOST_CLASS_EXPORT(myosim::ArmUnsyncEvent);
BOOST_CLASS_EXPORT(myosim::UnlockEvent);
BOOST_CLASS_EXPORT(myosim::LockEvent);
BOOST_CLASS_EXPORT(myosim::PoseEvent);
BOOST_CLASS_EXPORT(myosim::OrientationDataEvent);
BOOST_CLASS_EXPORT(myosim::AccelerometerDataEvent);
BOOST_CLASS_EXPORT(myosim::GyroscopeDataEvent);
BOOST_CLASS_EXPORT(myosim::RssiEvent);
BOOST_CLASS_EXPORT(myosim::BatteryLevelReceivedEvent);
BOOST_CLASS_EXPORT(myosim::EmgDataEvent);
BOOST_CLASS_EXPORT(myosim::WarmupCompletedEvent);

#include "SerializableEventTypes.h"
