#pragma once

namespace KalmanFilter{
  
  void predict(float& soc_pct,float& p_covariance,float current_ma,float dt_hours,float capacity_mah,float process_noise);
  
  void update(float& soc_pct,float& p_covariance,float measured_soc_pct,float measurement_noise);
}
