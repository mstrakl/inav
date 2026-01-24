
#include "nav_dlz.hpp"

using namespace AdumDlz;

void Navigation::update(const timeMs_t currentTime) {
    systime = currentTime;
}

void Navigation::reset() {
    systime = 0;
    posX = 0.0f;
}