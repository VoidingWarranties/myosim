#include "EventPlayerHub.h"

using namespace MyoSim;

EventPlayerHub::EventPlayerHub(const EventSession& events, float playback_speed,
                               const std::string& application_identifier)
    : events_(events),
      playback_speed_(playback_speed),
      Hub(application_identifier) {}

void EventPlayerHub::runAll() {
  int64_t previous_timestamp = findFirstTimestamp(events_);
  for (const auto& group : events_) {
    for (const auto& p_event : group.group) {
      auto microsoecond_diff =
          std::chrono::microseconds(p_event->timestamp - previous_timestamp);
      auto now = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(now + (microsoecond_diff / playback_speed_));
      simulateEvent(p_event);
      previous_timestamp = p_event->timestamp;
    }
  }
}

void EventPlayerHub::run(unsigned int duration_ms) {
  // TODO: implement this.
}

void EventPlayerHub::runOnce(unsigned int duration_ms) {
  // TODO: implement this.
}

int64_t EventPlayerHub::findFirstTimestamp(const EventSession& session) {
  for (const auto& group : session) {
    for (const auto& p_event : group.group) {
      return p_event->timestamp;
    }
  }
  return -1;
}

void EventPlayerHub::simulateEvent(const std::shared_ptr<MyoEvent>& p_event) {
  if (auto ptr = dynamic_cast<onPairEvent*>(p_event.get())) {
    simulatePair(nullptr, ptr->timestamp, ptr->firmware_version);
  } else if (auto ptr = dynamic_cast<onUnpairEvent*>(p_event.get())) {
    simulateUnpair(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<onConnectEvent*>(p_event.get())) {
    simulateConnect(nullptr, ptr->timestamp, ptr->firmware_version);
  } else if (auto ptr = dynamic_cast<onDisconnectEvent*>(p_event.get())) {
    simulateDisconnect(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<onArmSyncEvent*>(p_event.get())) {
    simulateArmSync(nullptr, ptr->timestamp, ptr->arm, ptr->x_direction,
                    ptr->rotation, ptr->warmup_state);
  } else if (auto ptr = dynamic_cast<onArmUnsyncEvent*>(p_event.get())) {
    simulateArmUnsync(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<onUnlockEvent*>(p_event.get())) {
    simulateUnlock(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<onLockEvent*>(p_event.get())) {
    simulateLock(nullptr, ptr->timestamp);
  } else if (auto ptr = dynamic_cast<onPoseEvent*>(p_event.get())) {
    simulatePose(nullptr, ptr->timestamp, ptr->pose);
  } else if (auto ptr = dynamic_cast<onOrientationDataEvent*>(p_event.get())) {
    simulateOrientationData(nullptr, ptr->timestamp, ptr->rotation);
  } else if (auto ptr = dynamic_cast<onAccelerometerDataEvent*>(p_event.get())) {
    simulateAccelerometerData(nullptr, ptr->timestamp, ptr->accel);
  } else if (auto ptr = dynamic_cast<onGyroscopeDataEvent*>(p_event.get())) {
    simulateGyroscopeData(nullptr, ptr->timestamp, ptr->gyro);
  } else if (auto ptr = dynamic_cast<onRssiEvent*>(p_event.get())) {
    simulateRssi(nullptr, ptr->timestamp, ptr->rssi);
  } else if (auto ptr = dynamic_cast<onBatteryLevelReceivedEvent*>(p_event.get())) {
    simulateBatteryLevelReceived(nullptr, ptr->timestamp, ptr->level);
  } else if (auto ptr = dynamic_cast<onEmgDataEvent*>(p_event.get())) {
    simulateEmgData(nullptr, ptr->timestamp, ptr->emg);
  } else if (auto ptr = dynamic_cast<onWarmupCompletedEvent*>(p_event.get())) {
    simulateWarmupCompleted(nullptr, ptr->timestamp, ptr->warmup_result);
  }
}
