#include "EventPlayerHub.h"

#include <chrono>
#include <thread>

namespace myosim {
EventPlayerHub::EventPlayerHub(const EventQueue& events, float playback_speed,
                               const std::string& application_identifier)
    : events_(events),
      playback_speed_(playback_speed),
      tmus_previous_run_end_(0),
      Hub(application_identifier) {
  popOnPeriodicEvents();
  if (! events_.queue.empty()) {
    tmus_previous_run_end_ = static_cast<MyoEvent*>(events_.queue.front().get())->timestamp;
  }
}

void EventPlayerHub::popOnPeriodicEvents() {
  while (! events_.queue.empty() &&
         dynamic_cast<PeriodicEvent*>(events_.queue.front().get())) {
    events_.queue.pop_front();
  }
}

void EventPlayerHub::runAll(const std::function<void(void)>& periodic) {
  // tm is time in myo time
  // tc is time in std::chrono::stead_clock time
  uint64_t tmus_previous = 0;
  popOnPeriodicEvents();
  if (! events_.queue.empty()) {
    // We can static_cast here because popOnPeriodicEvents removed all the
    // PeriodicEvents and those are the only events that are not MyoEvents.
    tmus_previous = static_cast<MyoEvent*>(events_.queue.front().get())->timestamp;
  }
  while (! events_.queue.empty()) {
    if (auto ptr = dynamic_cast<PeriodicEvent*>(events_.queue.front().get())) {
      periodic();
    } else if (auto ptr = dynamic_cast<MyoEvent*>(events_.queue.front().get())) {
      auto dtcms = std::chrono::microseconds(ptr->timestamp - tmus_previous);
      auto tc_now = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(tc_now + (dtcms / playback_speed_));
      simulateEvent(ptr);
      tmus_previous = ptr->timestamp;
    }
    events_.queue.pop_front();
  }
}

void EventPlayerHub::run(unsigned int duration_ms) {
  // tm is time in myo time
  // tc is time in std::chrono::stead_clock time
  uint64_t tmus_previous = tmus_previous_run_end_;
  uint64_t tmus_end = tmus_previous + (1000 * duration_ms);
  popOnPeriodicEvents();
  while (! events_.queue.empty()) {
    auto ptr_event = static_cast<MyoEvent*>(events_.queue.front().get());
    if (ptr_event->timestamp > tmus_end) {
      tmus_end = ptr_event->timestamp;
      break;
    } else {
      auto dtcus = std::chrono::microseconds(ptr_event->timestamp - tmus_previous);
      auto tc_now = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(tc_now + (dtcus / playback_speed_));
      simulateEvent(ptr_event);
      tmus_previous = ptr_event->timestamp;
      events_.queue.pop_front();
      popOnPeriodicEvents();
    }
  }
  auto dtcus = std::chrono::microseconds(tmus_end - tmus_previous);
  auto tc_now = std::chrono::steady_clock::now();
  std::this_thread::sleep_until(tc_now + (dtcus / playback_speed_));
  tmus_previous_run_end_ = tmus_end;
}

void EventPlayerHub::runOnce(unsigned int duration_ms) {
  // tm is time in myo time
  // tc is time in std::chrono::stead_clock time
  uint64_t tmus_previous = tmus_previous_run_end_;
  uint64_t tmus_end = tmus_previous + (1000 * duration_ms);
  popOnPeriodicEvents();
  if (! events_.queue.empty()) {
    auto ptr_event = static_cast<MyoEvent*>(events_.queue.front().get());
    if (ptr_event->timestamp <= tmus_end) {
      tmus_end = ptr_event->timestamp;
      auto dtcus = std::chrono::microseconds(tmus_end - tmus_previous);
      auto tc_now = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(tc_now + (dtcus / playback_speed_));
      tmus_previous_run_end_ = tmus_end;
      simulateEvent(ptr_event);
      events_.queue.pop_front();
      return;
    }
  }
  auto dtcus = std::chrono::microseconds(tmus_end - tmus_previous);
  auto tc_now = std::chrono::steady_clock::now();
  std::this_thread::sleep_until(tc_now + (dtcus / playback_speed_));
  tmus_previous_run_end_ = tmus_end;
}

void EventPlayerHub::simulateEvent(MyoEvent* p_event) {
  if (auto ptr = dynamic_cast<OrientationDataEvent*>(p_event)) {
    simulateOrientationData(nullptr, ptr->timestamp, ptr->rotation);
  } else if (auto ptr = dynamic_cast<AccelerometerDataEvent*>(p_event)) {
    simulateAccelerometerData(nullptr, ptr->timestamp, ptr->accel);
  } else if (auto ptr = dynamic_cast<GyroscopeDataEvent*>(p_event)) {
    simulateGyroscopeData(nullptr, ptr->timestamp, ptr->gyro);
  } else if (auto ptr = dynamic_cast<EmgDataEvent*>(p_event)) {
    simulateEmgData(nullptr, ptr->timestamp, ptr->emg);
  } else if (auto ptr = dynamic_cast<PoseEvent*>(p_event)) {
    simulatePose(nullptr, ptr->timestamp, ptr->pose);
  } else if (auto ptr = dynamic_cast<UnlockEvent*>(p_event)) {
    simulateUnlock(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<LockEvent*>(p_event)) {
    simulateLock(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<PairEvent*>(p_event)) {
    simulatePair(nullptr, ptr->timestamp, ptr->firmware_version);
  } else if (auto ptr = dynamic_cast<UnpairEvent*>(p_event)) {
    simulateUnpair(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<ConnectEvent*>(p_event)) {
    simulateConnect(nullptr, ptr->timestamp, ptr->firmware_version);
  } else if (auto ptr = dynamic_cast<DisconnectEvent*>(p_event)) {
    simulateDisconnect(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<ArmSyncEvent*>(p_event)) {
    simulateArmSync(nullptr, ptr->timestamp, ptr->arm, ptr->x_direction,
                    ptr->rotation, ptr->warmup_state);
  } else if (auto ptr = dynamic_cast<ArmUnsyncEvent*>(p_event)) {
    simulateArmUnsync(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<RssiEvent*>(p_event)) {
    simulateRssi(nullptr, ptr->timestamp, ptr->rssi);
  } else if (auto ptr = dynamic_cast<BatteryLevelReceivedEvent*>(p_event)) {
    simulateBatteryLevelReceived(nullptr, ptr->timestamp, ptr->level);
  } else if (auto ptr = dynamic_cast<WarmupCompletedEvent*>(p_event)) {
    simulateWarmupCompleted(nullptr, ptr->timestamp, ptr->warmup_result);
  }
}
} // namespace myosim
