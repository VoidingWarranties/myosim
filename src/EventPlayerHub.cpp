#include "EventPlayerHub.h"

#include <chrono>
#include <thread>

using namespace MyoSim;

EventPlayerHub::EventPlayerHub(const EventQueue& events, float playback_speed,
                               const std::string& application_identifier)
    : events_(events),
      playback_speed_(playback_speed),
      tmus_previous_run_end_(0),
      Hub(application_identifier) {
  popOnPeriodicEvents();
  if (! events_.empty()) {
    tmus_previous_run_end_ = static_cast<MyoEvent*>(events_.front().get())->timestamp;
  }
}

void EventPlayerHub::popOnPeriodicEvents() {
  while (! events_.empty() &&
         dynamic_cast<onPeriodicEvent*>(events_.front().get())) {
    events_.pop_front();
  }
}

void EventPlayerHub::runAll(const std::function<void(void)>& periodic) {
  // tm is time in myo time
  // tc is time in std::chrono::stead_clock time
  uint64_t tmus_previous = 0;
  popOnPeriodicEvents();
  if (! events_.empty()) {
    // We can static_cast here because popOnPeriodicEvents removed all the
    // onPeriodicEvents and those are the only events that are not MyoEvents.
    tmus_previous = static_cast<MyoEvent*>(events_.front().get())->timestamp;
  }
  while (! events_.empty()) {
    if (auto ptr = dynamic_cast<onPeriodicEvent*>(events_.front().get())) {
      periodic();
    } else if (auto ptr = dynamic_cast<MyoEvent*>(events_.front().get())) {
      auto dtcms = std::chrono::microseconds(ptr->timestamp - tmus_previous);
      auto tc_now = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(tc_now + (dtcms / playback_speed_));
      simulateEvent(ptr);
      tmus_previous = ptr->timestamp;
    }
    events_.pop_front();
  }
}

void EventPlayerHub::run(unsigned int duration_ms) {
  // tm is time in myo time
  // tc is time in std::chrono::stead_clock time
  uint64_t tmus_previous = tmus_previous_run_end_;
  uint64_t tmus_end = tmus_previous + (1000 * duration_ms);
  popOnPeriodicEvents();
  while (! events_.empty()) {
    auto ptr_event = static_cast<MyoEvent*>(events_.front().get());
    if (ptr_event->timestamp > tmus_end) {
      tmus_end = ptr_event->timestamp;
      break;
    } else {
      auto dtcus = std::chrono::microseconds(ptr_event->timestamp - tmus_previous);
      auto tc_now = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(tc_now + (dtcus / playback_speed_));
      simulateEvent(ptr_event);
      tmus_previous = ptr_event->timestamp;
      events_.pop_front();
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
  if (! events_.empty()) {
    auto ptr_event = static_cast<MyoEvent*>(events_.front().get());
    if (ptr_event->timestamp <= tmus_end) {
      tmus_end = ptr_event->timestamp;
      auto dtcus = std::chrono::microseconds(tmus_end - tmus_previous);
      auto tc_now = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(tc_now + (dtcus / playback_speed_));
      tmus_previous_run_end_ = tmus_end;
      simulateEvent(ptr_event);
      events_.pop_front();
      return;
    }
  }
  auto dtcus = std::chrono::microseconds(tmus_end - tmus_previous);
  auto tc_now = std::chrono::steady_clock::now();
  std::this_thread::sleep_until(tc_now + (dtcus / playback_speed_));
  tmus_previous_run_end_ = tmus_end;
}

void EventPlayerHub::simulateEvent(MyoEvent* p_event) {
  // TODO: reorder if else statements in order of highest frequency.
  //       e.g. put on{Accelerometer,Gyroscope,Orientation,Emg}Data first,
  //            onPose second, etc...
  if (auto ptr = dynamic_cast<onPairEvent*>(p_event)) {
    simulatePair(nullptr, ptr->timestamp, ptr->firmware_version);
  } else if (auto ptr = dynamic_cast<onUnpairEvent*>(p_event)) {
    simulateUnpair(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<onConnectEvent*>(p_event)) {
    simulateConnect(nullptr, ptr->timestamp, ptr->firmware_version);
  } else if (auto ptr = dynamic_cast<onDisconnectEvent*>(p_event)) {
    simulateDisconnect(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<onArmSyncEvent*>(p_event)) {
    simulateArmSync(nullptr, ptr->timestamp, ptr->arm, ptr->x_direction,
                    ptr->rotation, ptr->warmup_state);
  } else if (auto ptr = dynamic_cast<onArmUnsyncEvent*>(p_event)) {
    simulateArmUnsync(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<onUnlockEvent*>(p_event)) {
    simulateUnlock(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<onLockEvent*>(p_event)) {
    simulateLock(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<onPoseEvent*>(p_event)) {
    simulatePose(nullptr, ptr->timestamp, ptr->pose);
  } else if (auto ptr = dynamic_cast<onOrientationDataEvent*>(p_event)) {
    simulateOrientationData(nullptr, ptr->timestamp, ptr->rotation);
  } else if (auto ptr = dynamic_cast<onAccelerometerDataEvent*>(p_event)) {
    simulateAccelerometerData(nullptr, ptr->timestamp, ptr->accel);
  } else if (auto ptr = dynamic_cast<onGyroscopeDataEvent*>(p_event)) {
    simulateGyroscopeData(nullptr, ptr->timestamp, ptr->gyro);
  } else if (auto ptr = dynamic_cast<onRssiEvent*>(p_event)) {
    simulateRssi(nullptr, ptr->timestamp, ptr->rssi);
  } else if (auto ptr = dynamic_cast<onBatteryLevelReceivedEvent*>(p_event)) {
    simulateBatteryLevelReceived(nullptr, ptr->timestamp, ptr->level);
  } else if (auto ptr = dynamic_cast<onEmgDataEvent*>(p_event)) {
    simulateEmgData(nullptr, ptr->timestamp, ptr->emg);
  } else if (auto ptr = dynamic_cast<onWarmupCompletedEvent*>(p_event)) {
    simulateWarmupCompleted(nullptr, ptr->timestamp, ptr->warmup_result);
  }
}
