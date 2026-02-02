#include "nav_dlz.hpp"
#include "drivers/time.h"

using namespace AdumDlz;

static Navigation *g_nav = nullptr;

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
        if (!g_nav) return;
        g_nav->readSkyvisData(bufferPtr, dataSize);
    } // extern "C"


    void adum_dlz_update()  {
        if (!g_nav) adum_dlz_init();
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