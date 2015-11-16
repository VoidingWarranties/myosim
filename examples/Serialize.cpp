#include <fstream>

#include <boost/serialization/deque.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <myo/myo.hpp>
#include "../src/EventRecorder.h"
#include "../src/EventPlayerHub.h"
#include "../src/Hub.h"

class PrintListener : public myo::DeviceListener {
 public:
  void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {
    std::cout << "Detected pose! " << pose << std::endl;
  }
};

void saveEvents(const MyoSim::EventQueue& events, const std::string& file_path) {
  std::ofstream ofs(file_path);
  boost::archive::xml_oarchive oa(ofs);
  oa << BOOST_SERIALIZATION_NVP(events);
}

MyoSim::EventQueue loadEvents(const std::string& file_path) {
  MyoSim::EventQueue events;
  std::ifstream ifs(file_path);
  boost::archive::xml_iarchive ia(ifs);
  ia >> BOOST_SERIALIZATION_NVP(events);
  return events;
}

int main() {
  try {
    myo::Hub hub("com.voidingwarranties.myo-simulator-example");
    myo::Myo* myo = hub.waitForMyo(10000);
    if (!myo) {
      throw std::runtime_error("Unable to find a Myo!");
    }
    // Record only pose events.
    MyoSim::EventRecorder recorder(MyoSim::EventRecorder::POSE);
    hub.addListener(&recorder);
    // Record for 5 seconds.
    myo->unlock(myo::Myo::unlockHold);
    hub.run(5000);
    myo->lock();

    // Serialize event session.
    saveEvents(recorder.getEventQueue(), "serialized_events");

    std::cout << "Events recorded and serialized. "
              << "Press ENTER to deserialize and replay events.";
    getchar();

    // Deserialize event session.
    auto deserialized_events = loadEvents("serialized_events");

    // Replay event session.
    MyoSim::Hub simulated_hub;
    PrintListener print_listener;
    MyoSim::EventPlayerHub player_hub(deserialized_events);
    player_hub.addListener(&print_listener);
    player_hub.runAll();

  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }
  return 0;
}
