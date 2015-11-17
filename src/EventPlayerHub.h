#pragma once

#include <functional>
#include <vector>
#include <myo/myo.hpp>

#include "Hub.h"
#include "EventTypes.h"

namespace myosim {
class EventPlayerHub : public Hub {
 public:
  explicit EventPlayerHub(const EventQueue& events, float playback_speed = 1,
                          const std::string& application_identifier = "");

  void runAll(const std::function<void(void)>& periodic = [](){});
  void run(unsigned int duration_ms);
  void runOnce(unsigned int duration_ms);

 private:
  void popOnPeriodicEvents();
  void simulateEvent(MyoEvent* p_event);

  const float playback_speed_;
  EventQueue events_;
  uint64_t tmus_previous_run_end_;
  std::vector<myo::Myo*> myos_;
};
} // namespace myosim
