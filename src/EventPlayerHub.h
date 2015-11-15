#pragma once

#include <myo/myo.hpp>

#include <functional>
#include <chrono>
#include <thread>

#include "Hub.h"
#include "EventTypes.h"

namespace MyoSim {
class EventPlayerHub : public Hub {
 public:
  explicit EventPlayerHub(const EventSession& events, float playback_speed = 1,
                          const std::string& application_identifier = "");

  void runAll();
  void run(unsigned int duration_ms);
  void runOnce(unsigned int duration_ms);

 private:
  int64_t findFirstTimestamp(const EventSession& session);
  void simulateEvent(const std::shared_ptr<MyoEvent>& p_event);

  const float playback_speed_;
  EventSession events_;
};
}
