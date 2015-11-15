#include "EventPlayer.h"

using namespace MyoSim;

EventPlayer::EventPlayer(MyoSim::Hub& hub) : hub_(hub) {}

void EventPlayer::play(const EventSession& session, float playback_speed,
                       const std::function<void(void)>& periodic) {
  int64_t previous_timestamp = findFirstTimestamp(session);
  for (const auto& group : session) {
    for (const auto& base_ptr : group.group) {
      auto microsoecond_diff =
          std::chrono::microseconds(base_ptr->timestamp - previous_timestamp);
      auto now = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(now + (microsoecond_diff / playback_speed));

      if (auto ptr = dynamic_cast<onPairEvent*>(base_ptr.get())) {
        hub_.simulatePair(nullptr, ptr->timestamp, ptr->firmware_version);
      } else if (auto ptr = dynamic_cast<onUnpairEvent*>(base_ptr.get())) {
        hub_.simulateUnpair(nullptr, ptr->timestamp);
      } else if (auto ptr = dynamic_cast<onConnectEvent*>(base_ptr.get())) {
        hub_.simulateConnect(nullptr, ptr->timestamp, ptr->firmware_version);
      } else if (auto ptr = dynamic_cast<onDisconnectEvent*>(base_ptr.get())) {
        hub_.simulateDisconnect(nullptr, ptr->timestamp);
      } else if (auto ptr = dynamic_cast<onArmSyncEvent*>(base_ptr.get())) {
        hub_.simulateArmSync(nullptr, ptr->timestamp, ptr->arm,
                             ptr->x_direction, ptr->rotation,
                             ptr->warmup_state);
      } else if (auto ptr = dynamic_cast<onArmUnsyncEvent*>(base_ptr.get())) {
        hub_.simulateArmUnsync(nullptr, ptr->timestamp);
      } else if (auto ptr = dynamic_cast<onUnlockEvent*>(base_ptr.get())) {
        hub_.simulateUnlock(nullptr, ptr->timestamp);
      } else if (auto ptr = dynamic_cast<onLockEvent*>(base_ptr.get())) {
        hub_.simulateLock(nullptr, ptr->timestamp);
      } else if (auto ptr = dynamic_cast<onPoseEvent*>(base_ptr.get())) {
        hub_.simulatePose(nullptr, ptr->timestamp, ptr->pose);
      } else if (auto ptr = dynamic_cast<onOrientationDataEvent*>(base_ptr.get())) {
        hub_.simulateOrientationData(nullptr, ptr->timestamp, ptr->rotation);
      } else if (auto ptr = dynamic_cast<onAccelerometerDataEvent*>(base_ptr.get())) {
        hub_.simulateAccelerometerData(nullptr, ptr->timestamp, ptr->accel);
      } else if (auto ptr = dynamic_cast<onGyroscopeDataEvent*>(base_ptr.get())) {
        hub_.simulateGyroscopeData(nullptr, ptr->timestamp, ptr->gyro);
      } else if (auto ptr = dynamic_cast<onRssiEvent*>(base_ptr.get())) {
        hub_.simulateRssi(nullptr, ptr->timestamp, ptr->rssi);
      } else if (auto ptr = dynamic_cast<onBatteryLevelReceivedEvent*>(base_ptr.get())) {
        hub_.simulateBatteryLevelReceived(nullptr, ptr->timestamp, ptr->level);
      } else if (auto ptr = dynamic_cast<onEmgDataEvent*>(base_ptr.get())) {
        hub_.simulateEmgData(nullptr, ptr->timestamp, ptr->emg);
      } else if (auto ptr = dynamic_cast<onWarmupCompletedEvent*>(base_ptr.get())) {
        hub_.simulateWarmupCompleted(nullptr, ptr->timestamp,
                                     ptr->warmup_result);
      }
      previous_timestamp = base_ptr->timestamp;
    }
    // TODO: call periodic for each Myo, event though each Myo* will be nullptr.
    periodic();
  }
}

int64_t EventPlayer::findFirstTimestamp(const EventSession& session) {
  for (const auto& group : session) {
    for (const auto& base_ptr : group.group) {
      return base_ptr->timestamp;
    }
  }
  return -1;
}
