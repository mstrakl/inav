#ifndef ADUM_NAV_DLZ_UTILS_HPP
#define ADUM_NAV_DLZ_UTILS_HPP

#include <cmath>

extern "C" {
#include "common/maths.h"
}

namespace AdumDlzUtils {
    class RateLimiter {

    public:
        // Constructor
        RateLimiter(const float max_rate_change_per_sec)
            : max_rate(max_rate_change_per_sec)
            , first_call(true) {}

        // Update function with dt argument
        float update(float target_value, float dt) {
            
            if (first_call) {
                current_value = target_value;
                first_call = false;
                
            } else {
                
                float max_change = max_rate * dt;  

                if (std::fabs(target_value - current_value) > max_change) {
                    if (target_value > current_value) {
                        current_value += max_change;
                    } else {
                        current_value -= max_change;
                    }
                } else {
                    current_value = target_value;
                }
                
            }

            return current_value;
        }
        
        void reset() { first_call = true; };


    private:
        float max_rate{0.0};     // Maximum change per second
        float current_value{0.0};
        bool first_call{true};    // Flag to check if update() is called for the first time

    };
}
#endif