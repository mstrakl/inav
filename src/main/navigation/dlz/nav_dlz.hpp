
#ifndef ADUM_NAV_DLZ_HPP
#define ADUM_NAV_DLZ_HPP

#include "drivers/time.h"
//#include "nav_dlz_utils.hpp"

namespace AdumDlz {

    typedef struct __attribute__((packed)) {
        int16_t  cmdPitch;      // degress * 100
        int16_t  cmdRoll;       // degrees * 100
        int16_t  cmdAltitudeMm; // mm
        int16_t  confidence; // 0 - 1000
    } mspSensorSkyvis_t;

    class Navigation {

    public:
        Navigation();

        void reset();

        void readSkyvisData(const uint8_t* bufferPtr, 
                            unsigned int dataSize);

        void update(const float& centimeterPosX, 
                    const float& centimeterPosY,
                    const float& centimeterVelX, 
                    const float& centimeterVelY,
                    const float& navPitchCmd, 
                    const float& navRollCmd);

        float getPitchCmd() const;

        float getRollCmd() const;


    private:

        timeMs_t    m_lastMspRxTime{0};
        timeMs_t    m_lastUpdateTime{0};
        mspSensorSkyvis_t m_skyvisData{0, 0, 0, 0};

        float m_cmdPitch{0.0f};   // radians
        float m_cmdRoll{0.0f};   // radians
        
        //AdumDlzUtils::RateLimiter m_cmdPitchRL{0.1f}; // radians per second
        //AdumDlzUtils::RateLimiter m_cmdRollRL{0.1f};


    };

} // namespace AdumDlz

#endif // ADUM_NAV_DLZ_HPP