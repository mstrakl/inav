#include "navigation_dlz.h"
#include "common/log.h"

#define UPDATE_TIMEOUT_MS 1000  // if no update in this time, DLZ is considered lost
#define MAX_POS_CMD_METERS 20.0f


void mspSkyvisReceiveNewData(const uint8_t * bufferPtr, unsigned int dataSize)
{

    if(dataSize != sizeof(mspSensorSkyvis_t)) {
        LOG_ERROR(SYSTEM, "mspSkyvisReceiveNewData: invalid data size %d", dataSize);
        return;
    }

    const mspSensorSkyvis_t *pkt = (const mspSensorSkyvis_t *)bufferPtr;

    NavDlzData.lastUpdateTime = millis();
    NavDlzData.posX = pkt->posX;
    NavDlzData.posY = pkt->posY;
    NavDlzData.velZ = pkt->velZ;
    NavDlzData.gpsFade = pkt->gpsFade;

    //printf("DLZ.Msp.Rx: Time=%d X=%d Y=%d velZ=%d fade=%d\n", NavDlzData.lastUpdateTime, pkt->posX, pkt->posY, pkt->velZ, pkt->gpsFade);

}
