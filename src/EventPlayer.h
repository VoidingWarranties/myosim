#pragma once

#include <myo/myo.hpp>

#include <functional>
#include <chrono>
#include <thread>

#include "Hub.h"
#include "EventTypes.h"

namespace MyoSim {
class EventPlayer {
 public:
  explicit EventPlayer(MyoSim::Hub& hub);

  void play(const EventSession& session, float playback_speed = 1,
            const std::function<void(void)>& periodic = [](){});

 private:
  int64_t findFirstTimestamp(const EventSession& session);

  MyoSim::Hub& hub_;
  std::function<void(myo::Myo*)> periodic_;
};
}
