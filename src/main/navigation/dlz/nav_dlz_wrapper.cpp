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


void adum_dlz_update(const float centimeterPosX, 
                     const float centimeterPosY,
                     const float centimeterVelX, 
                     const float centimeterVelY,
                     const float navPitchCmd, 
                     const float navRollCmd)  {
    if (!g_nav) adum_dlz_init();
    g_nav->update(centimeterPosX, 
                  centimeterPosY, 
                  centimeterVelX, 
                  centimeterVelY, 
                  navPitchCmd, 
                  navRollCmd);
}


const float adum_dlz_getpitchcmd() {
    if (!g_nav) return 0.0f;
    return g_nav->getPitchCmd();
}

const float adum_dlz_getrollcmd() {
    if (!g_nav) return 0.0f;
    return g_nav->getRollCmd();
}


}