#include "Kalman_Filter.h"
#include <algorithm>

namespace KalmanFilter{

    void predict(float& soc_pct,float& p_covariance,float current_ma,float dt_hours,float capacity_mah,float process_noise){
        if (capacity_mah <= 0.0f) {
            return;
        }
        const float soc_change=(current_ma*dt_hours/capacity_mah)*100.0f;
        soc_pct+=soc_change;
        p_covariance+=process_noise;
    }

    void update(float& soc_pct,float& p_covariance,float measured_soc_pct,float measurement_noise){
        const float denom = p_covariance + measurement_noise;
        if (denom <= 0.0f) {
            return;
        }
        const float kalman_gain = p_covariance / denom;
        soc_pct = soc_pct + kalman_gain * (measured_soc_pct - soc_pct);
        const float one_minus_k = 1.0f - kalman_gain;
        p_covariance = (one_minus_k * one_minus_k * p_covariance) +
                       (kalman_gain * kalman_gain * measurement_noise);
        soc_pct = std::clamp(soc_pct, 0.0f, 100.0f);
    }

}
