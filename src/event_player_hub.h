/* EventPlayerHub derives from Hub and provides new implementations of run and
 * runOnce that hide myo::Hub's methods. runAll, run, and runOnce will replay
 * the EventQueue given in the constructor.
 */

#pragma once

#include <functional>
#include <vector>
#include <myo/myo.hpp>

#include "hub.h"
#include "event_types.h"

namespace myosim {
class EventPlayerHub : public Hub {
 public:
  explicit EventPlayerHub(const EventQueue& events, float playback_speed = 1,
                          const std::string& application_identifier = "");

  // Simulates all events in the EventQueue, consuming the queue as it does so.
  // periodic is called on every PeriodicEvent.
  void runAll(const std::function<void(void)>& periodic = [](){});
  // Run the event loop simulation for the specified duration (in milliseconds).
  void run(unsigned int duration_ms);
  // Run the event loop simulation until a single event occurs, or the specified
  // duration (in milliseconds) has elapsed.
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
