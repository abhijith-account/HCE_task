#pragma once

#include <cstdint>
#include "Battery_Types.h"

namespace FsmClassifier {
  constexpr int32_t HYSTERESIS_UV=1500;
  constexpr float EMA_ALPHA=0.05f;
  
  void reset();
  
  float update(int32_t shunt_uv);
  
  BatteryFSM classify(BatteryFSM current, float ema_uv,int32_t idle_threshold_uv);
}
