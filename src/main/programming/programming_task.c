/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include "platform.h"

#include "programming/logic_condition.h"
#include "programming/global_variables.h"
#include "programming/pid.h"
#include "flight/mixer_profile.h"


#include "drivers/time.h"


#define MIXER_LC_ID 60

static bool old_state = false;
static uint32_t t_change = 0;

static void updateTransitionAdum(void);

void programmingFrameworkUpdateTask(timeUs_t currentTimeUs)
{
    programmingPidUpdateTask(currentTimeUs);
    outputProfileUpdateTask(currentTimeUs);
    logicConditionUpdateTask(currentTimeUs);

    updateTransitionAdum();
}

static void updateTransitionAdum(void)
{

    const bool newState = logicConditionGetValue(MIXER_LC_ID) != 0;


    unsigned int t_fw_to_mc = gvGet(1);
    unsigned int t_mc_to_fw = gvGet(2);


    // Overload with default if stupid values
    if (t_fw_to_mc < 1000 || t_fw_to_mc > 10000) {
        t_fw_to_mc = 1000;
    }

    if (t_mc_to_fw < 1000 || t_mc_to_fw > 10000) {
        t_mc_to_fw = 5000;
    }

    if (newState != old_state) {
        t_change = millis();
    }

    if (newState) {
            
        if (millis() - t_change < t_fw_to_mc) {
            gvSet(0, 20);
        } else {
            gvSet(0, 30);
        }

    } else {

        if (millis() - t_change < t_mc_to_fw) {
            gvSet(0, 20);
        } else {
            gvSet(0, 10);
        }

    }
    
    old_state = newState;

}