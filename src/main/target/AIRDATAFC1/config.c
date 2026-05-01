/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, or (at your option) any
 * later version.
 */

#include <stdint.h>

#include "platform.h"

#include "fc/fc_msp_box.h"
#include "io/serial.h"

void targetConfiguration(void)
{
    const int portIndex = findSerialPortIndexByIdentifier(SERIAL_PORT_USART1);

    if (portIndex >= 0) {
        serialConfigMutable()->portConfigs[portIndex].functionMask = FUNCTION_MSP;
        serialConfigMutable()->portConfigs[portIndex].msp_baudrateIndex = BAUD_115200;
    }
}

void validateAndFixTargetConfig(void)
{
    const int portIndex = findSerialPortIndexByIdentifier(SERIAL_PORT_USART1);

    if (portIndex >= 0) {
        serialConfigMutable()->portConfigs[portIndex].functionMask = FUNCTION_MSP;
        serialConfigMutable()->portConfigs[portIndex].msp_baudrateIndex = BAUD_115200;
    }
}
