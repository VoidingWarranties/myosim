#include "EventTypes.h"

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

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
