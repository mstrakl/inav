
#ifndef ADUM_NAV_DLZ_HPP
#define ADUM_NAV_DLZ_HPP

#include "drivers/time.h"

namespace AdumDlz {

    class Navigation {

    public:
        Navigation() = default;
        ~Navigation() = default;

        void reset();

        void update(timeMs_t currentTime);

    private:

        timeMs_t    systime{0};
        float       posX{0.0f};

    };

} // namespace AdumDlz

#endif // ADUM_NAV_DLZ_HPP