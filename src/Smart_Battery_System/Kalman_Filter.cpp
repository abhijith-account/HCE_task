#include "Kalman_Filter.h"
#include <algorithm>
using namespace std;

namespace KalmanFilter{

  void predict(float& soc_pct,float& p_covariance,float current_ma,float dt_hours,float capacity_mah,float process_noise){
    const float soc_change=(current_ma*dt_hours/capacity_mah)*100.0f;
    soc_pct+=soc_change;
    p_covariance+=process_noise;
  }
  
  void update(float& soc_pct,float& p_covariance,float measured_soc_pct,float measurement_noise){
    const float kalman_gain=p_covariance/(p_covariance+measurement_noise);
    soc_pct=soc_pct+kalman_gain*(measured_soc_pct-soc_pct);
    p_covariance=(1.0f-kalman_gain)*p_covariance;
    soc_pct=clamp(soc_pct,0.0f,100.0f);
  }

}
