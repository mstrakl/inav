
#ifndef ADUM_NAV_DLZ_HPP
#define ADUM_NAV_DLZ_HPP

#include "drivers/time.h"
//#include "nav_dlz_utils.hpp"

static const int LOGIC_COND_GUIDANCE = 50;

namespace AdumDlz {

    typedef struct __attribute__((packed)) {
        int32_t nedPosX;  // cm
        int32_t nedPosY;  // cm
        int16_t nedVelX;  // cm/s
        int16_t nedVelY;  // cm/s
        int16_t confidence; // 0 - 1000
    } mspSensorSkyvis_t;

    class Navigation {

    public:
        Navigation();

        void reset();

        void readSkyvisData(const uint8_t* bufferPtr, 
                            unsigned int dataSize);

        void update();


        const float getNedPosX() const;
        const float getNedPosY() const;
        const float getWeighedNedPosX() const;
        const float getWeighedNedPosY() const;
        const float getNedVelX() const;
        const float getNedVelY() const;
        const float getFade() const;

    private:

        timeMs_t    m_lastMspRxTime{0};
        timeMs_t    m_lastUpdateTime{0};
        mspSensorSkyvis_t m_skyvisData{0, 0, 0, 0, 0};


        float m_nedPosX{0.0f};
        float m_nedPosY{0.0f};
        float m_nedVelX{0.0f};
        float m_nedVelY{0.0f};

        float m_fade{0.0f};
        
        float m_weighedNedPosX{0.0f};
        float m_weighedNedPosY{0.0f};

    };

} // namespace AdumDlz


#endif // ADUM_NAV_DLZ_HPP