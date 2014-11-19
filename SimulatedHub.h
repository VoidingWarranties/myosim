#ifndef SIMULATED_HUB_H_
#define SIMULATED_HUB_H_

#include <string>
#include <myo/myo.hpp>

class SimulatedHub {
 public:
  Hub(const std::string& applicationIdentifier = "") : application_identifier_(applicationIdentifier) {}

  myo::Myo* waitForMyo(unsigned int milliseconds = 0) {}
  void addListener(myo::DeviceListener* listener) { listener_ = listener; }
  void removeListener(myo::DeviceListener* listener) { listener_ = nullptr; }
  void run(unsigned int duration_ms) {
    // Listen for user input here
  }
  void runOnce(unsigned int duration_ms) {
    // Listen for user input once, and then return.
  }

 private:
  std::string application_identifier_;
  myo::DeviceListener* listener_;
};

#endif
