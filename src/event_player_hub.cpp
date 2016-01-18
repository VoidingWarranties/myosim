#include "event_player_hub.h"

#include <chrono>
#include <thread>

namespace myosim {
EventPlayerHub::EventPlayerHub(const EventQueue& events, float playback_speed,
                               const std::string& application_identifier)
    : events_(events),
      playback_speed_(playback_speed),
      tmus_previous_run_end_(0),
      myos_(events.num_myos, nullptr),
      Hub(application_identifier) {
  // Get the timestamp of the first myo event.
  popOnPeriodicEvents();
  if (! events_.queue.empty()) {
    // We can static_cast here because popOnPeriodicEvents removed all the
    // PeriodicEvents and those are the only events that are not MyoEvents.
    tmus_previous_run_end_ = static_cast<MyoEvent*>(events_.queue.front().get())->timestamp;
  }
  // Get the same number of myos as used in the event queue.
  for (size_t i_myo = 0; i_myo < myos_.size(); ++i_myo) {
    myos_[i_myo] = myo::Hub::waitForMyo(10);
    if (myos_[i_myo] == nullptr) {
      for (size_t j_myo = 0; j_myo < i_myo; ++j_myo) {
        myos_[j_myo] = nullptr;
      }
      break;
    }
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
      std::this_thread::sleep_for(dtcms / playback_speed_);
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
    // We can static_cast here because popOnPeriodicEvents removed all the
    // PeriodicEvents and those are the only events that are not MyoEvents.
    auto ptr_event = static_cast<MyoEvent*>(events_.queue.front().get());
    if (ptr_event->timestamp > tmus_end) {
      // Event doesn't occur in this call to run. Wait until tmus_end.
      auto dtcus = std::chrono::microseconds(tmus_end - tmus_previous);
      std::this_thread::sleep_for(dtcus / playback_speed_);
      // Setup tmus_previous_run_end_ for next call to run or runOnce.
      tmus_previous_run_end_ = tmus_end;
      return;
    } else {
      // Wait until the event's timestamp.
      auto dtcus = std::chrono::microseconds(ptr_event->timestamp - tmus_previous);
      std::this_thread::sleep_for(dtcus / playback_speed_);
      // Simulate event.
      simulateEvent(ptr_event);
      // Setup for next iteration of while loop.
      tmus_previous = ptr_event->timestamp;
      events_.queue.pop_front();
      popOnPeriodicEvents();
    }
  }
}

void EventPlayerHub::runOnce(unsigned int duration_ms) {
  // tm is time in myo time
  // tc is time in std::chrono::stead_clock time
  uint64_t tmus_end = tmus_previous_run_end_ + (1000 * duration_ms);

  popOnPeriodicEvents();
  if (! events_.queue.empty()) {
    // We can static_cast here because popOnPeriodicEvents removed all the
    // PeriodicEvents and those are the only events that are not MyoEvents.
    auto ptr_event = static_cast<MyoEvent*>(events_.queue.front().get());
    if (ptr_event->timestamp > tmus_end) {
      // Event doesn't occur in this call to runOnce. Wait until tmus_end.
      auto dtcus = std::chrono::microseconds(tmus_end - tmus_previous_run_end_);
      std::this_thread::sleep_for(dtcus / playback_speed_);
      // Setup tmus_previous_run_end_ for next call to run or runOnce.
      tmus_previous_run_end_ = tmus_end;
      return;
    } else {
      // Wait until the event's timestamp.
      auto dtcus = std::chrono::microseconds(ptr_event->timestamp - tmus_previous_run_end_);
      std::this_thread::sleep_for(dtcus / playback_speed_);
      // Simulate event.
      simulateEvent(ptr_event);
      // Setup for next call to run or runOnce.
      tmus_previous_run_end_ = ptr_event->timestamp;
      events_.queue.pop_front();
    }
  }
}

void EventPlayerHub::simulateEvent(MyoEvent* p_event) {
  if (auto ptr = dynamic_cast<OrientationDataEvent*>(p_event)) {
    simulateOrientationData(myos_[ptr->myo_index], ptr->timestamp, ptr->rotation);
  } else if (auto ptr = dynamic_cast<AccelerometerDataEvent*>(p_event)) {
    simulateAccelerometerData(myos_[ptr->myo_index], ptr->timestamp, ptr->accel);
  } else if (auto ptr = dynamic_cast<GyroscopeDataEvent*>(p_event)) {
    simulateGyroscopeData(myos_[ptr->myo_index], ptr->timestamp, ptr->gyro);
  } else if (auto ptr = dynamic_cast<EmgDataEvent*>(p_event)) {
    simulateEmgData(myos_[ptr->myo_index], ptr->timestamp, ptr->emg);
  } else if (auto ptr = dynamic_cast<PoseEvent*>(p_event)) {
    simulatePose(myos_[ptr->myo_index], ptr->timestamp, ptr->pose);
  } else if (auto ptr = dynamic_cast<UnlockEvent*>(p_event)) {
    simulateUnlock(myos_[ptr->myo_index], ptr->timestamp);
  } else if (auto ptr = dynamic_cast<LockEvent*>(p_event)) {
    simulateLock(myos_[ptr->myo_index], ptr->timestamp);
  } else if (auto ptr = dynamic_cast<PairEvent*>(p_event)) {
    simulatePair(myos_[ptr->myo_index], ptr->timestamp, ptr->firmware_version);
  } else if (auto ptr = dynamic_cast<UnpairEvent*>(p_event)) {
    simulateUnpair(myos_[ptr->myo_index], ptr->timestamp);
  } else if (auto ptr = dynamic_cast<ConnectEvent*>(p_event)) {
    simulateConnect(myos_[ptr->myo_index], ptr->timestamp, ptr->firmware_version);
  } else if (auto ptr = dynamic_cast<DisconnectEvent*>(p_event)) {
    simulateDisconnect(myos_[ptr->myo_index], ptr->timestamp);
  } else if (auto ptr = dynamic_cast<ArmSyncEvent*>(p_event)) {
    simulateArmSync(myos_[ptr->myo_index], ptr->timestamp, ptr->arm,
                    ptr->x_direction, ptr->rotation, ptr->warmup_state);
  } else if (auto ptr = dynamic_cast<ArmUnsyncEvent*>(p_event)) {
    simulateArmUnsync(myos_[ptr->myo_index], ptr->timestamp);
  } else if (auto ptr = dynamic_cast<RssiEvent*>(p_event)) {
    simulateRssi(myos_[ptr->myo_index], ptr->timestamp, ptr->rssi);
  } else if (auto ptr = dynamic_cast<BatteryLevelReceivedEvent*>(p_event)) {
    simulateBatteryLevelReceived(myos_[ptr->myo_index], ptr->timestamp, ptr->level);
  } else if (auto ptr = dynamic_cast<WarmupCompletedEvent*>(p_event)) {
    simulateWarmupCompleted(myos_[ptr->myo_index], ptr->timestamp, ptr->warmup_result);
  }
}
} // namespace myosim
