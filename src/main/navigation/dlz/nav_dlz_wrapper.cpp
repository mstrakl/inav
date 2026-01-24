#include "nav_dlz.hpp"
#include "drivers/time.h"

using namespace AdumDlz;

static Navigation *g_nav = nullptr;

extern "C" {

void adum_dlz_init(void) {
    if (!g_nav) g_nav = new Navigation();
}

void adum_dlz_update(timeMs_t currentTime) {
    if (!g_nav) adum_dlz_init();
    g_nav->update(currentTime);
}

void adum_dlz_reset(void) {
    if (!g_nav) return;
    g_nav->reset();
}

} // extern "C"
