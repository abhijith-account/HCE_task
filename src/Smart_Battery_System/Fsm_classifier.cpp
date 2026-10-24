#include "Fsm_classifier.h"

namespace {
  float g_shunt_ema=0.0f;
  bool g_ema_initialized=false;
}

namespace FsmClassifier {
  void reset(){
    g_shunt_ema=0.0f;
    g_ema_initialized=false;
  }

  float update(int32_t shunt_uv){
    if (!g_ema_initialized){
        g_shunt_ema=static_cast<float>(shunt_uv);
        g_ema_initialized=true;
    }
    else{
        g_shunt_ema=(EMA_ALPHA*static_cast<float>(shunt_uv))+((1.0f-EMA_ALPHA)*g_shunt_ema);
    }
  
    return g_shunt_ema;
  }

  BatteryFSM classify(BatteryFSM current, float ema_uv,
                      int32_t idle_threshold_uv) {
      const float pos_threshold = static_cast<float>(idle_threshold_uv);
      const float neg_threshold = -pos_threshold;
      const float hysteresis = static_cast<float>(HYSTERESIS_UV);

      if (current == BatteryFSM::CHARGING) {
          if (ema_uv < neg_threshold) {
              return BatteryFSM::CHARGING;
          } else if (ema_uv > (pos_threshold + hysteresis)) {
              return BatteryFSM::DISCHARGING;
          } else {
              return BatteryFSM::IDLE;
          }
      } else if (current == BatteryFSM::DISCHARGING) {
          if (ema_uv > pos_threshold) {
              return BatteryFSM::DISCHARGING;
          } else if (ema_uv < (neg_threshold - hysteresis)) {
              return BatteryFSM::CHARGING;
          } else {
              return BatteryFSM::IDLE;
          }
      }

      // Current state is IDLE.
      if (ema_uv < neg_threshold) {
          return BatteryFSM::CHARGING;
      } else if (ema_uv > pos_threshold) {
          return BatteryFSM::DISCHARGING;
      }

      return BatteryFSM::IDLE;
  }

}
