
#include "OSD.h"


bool OSD_EN = 1; //默认开启OSD，但若AT7456初始化失败则关闭


    /*OSD 初始化相关*/
    if(at7456_init() == 0) OSD_init();
    else OSD_EN = 0; // OSD使能取消


        //50HZ任务
        if(loop50HzFlag==1){
            if(OSD_EN == 1) OSD_update(FLY_MODE, batV, DBM_STA, RC_throt, euler[0], -euler[1], coreT, SYS_LOCK);
            //OSD_update(uint8_t mode, float batV, uint8_t RSSI, uint8_t throt, float pitch, float roll, float coreTemp, bool lock);

        }
