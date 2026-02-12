#include "nav_dlz.hpp"
#include "drivers/time.h"
#include <cstring>

using namespace AdumDlz;

static Navigation *g_nav = nullptr;

// Driver buffer + flag (driver/ISR writes here, main loop performs handoff)
static mspSensorSkyvis_t g_skyvisDrv{};
static volatile bool g_skyvisNewData = false;

extern "C" {

    void adum_dlz_init(void) {
        if (!g_nav) g_nav = new Navigation();
    }


    void adum_dlz_reset(void) {
        if (!g_nav) return;
        g_nav->reset();
    }


    void adum_dlz_readskyvisdata(const uint8_t* bufferPtr, 
                                unsigned int dataSize)
    {
        // Fast driver-side copy into driver buffer; main-loop will perform the authoritative handoff.
        if (dataSize != sizeof(g_skyvisDrv) || bufferPtr == nullptr) {
            return;
        }
        memcpy(&g_skyvisDrv, bufferPtr, sizeof(g_skyvisDrv));
        g_skyvisNewData = true;
    }


    void adum_dlz_update()  {
        if (!g_nav) adum_dlz_init();

        // If driver supplied new data, copy it into the Navigation object from main context.
        if (g_skyvisNewData) {
            g_nav->readSkyvisData(reinterpret_cast<const uint8_t*>(&g_skyvisDrv), sizeof(g_skyvisDrv));
            g_skyvisNewData = false;
        }

        g_nav->update();
    }


    const float adum_dlz_get_ned_pos_x(void) {
        if (!g_nav) return 0.0f;
        return g_nav->getNedPosX();
    }
    const float adum_dlz_get_ned_pos_y(void) {
        if (!g_nav) return 0.0f;
        return g_nav->getNedPosY();
    }
    const float adum_dlz_get_weighed_ned_pos_x(void) {
        if (!g_nav) return 0.0f;
        return g_nav->getWeighedNedPosX();
    }
    const float adum_dlz_get_weighed_ned_pos_y(void) {
        if (!g_nav) return 0.0f;
        return g_nav->getWeighedNedPosY();
    }
    const float adum_dlz_get_ned_vel_x(void) {
        if (!g_nav) return 0.0f;
        return g_nav->getNedVelX();
    }
    const float adum_dlz_get_ned_vel_y(void) {
        if (!g_nav) return 0.0f;
        return g_nav->getNedVelY();
    }
    const float adum_dlz_get_fade(void) {
        if (!g_nav) return 0.0f;
        return g_nav->getFade();
    }


}