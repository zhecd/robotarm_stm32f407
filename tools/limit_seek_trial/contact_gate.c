#include "contact_gate.h"
bool SeekContact_OnEdge(SeekContact *g, bool active, uint8_t axis,
                       uint8_t limits, uint8_t irq_latch, uint32_t now)
{
    if(!g || !g->seek || !active || irq_latch || axis>=3U ||
       !(limits&(1U<<axis)) || (g->masked_mask&(1U<<axis)))return false;
    Seek s=*g->seek;
    if(!Seek_BeginContact(&s,now,axis))return false;
    *g->seek=s;
    /* 顺序是安全合同：必须先停止真实输出，再屏蔽单路重复唤醒。 */
    SeekContactPort_StopOutputs();
    SeekContactPort_MaskAxis(axis);
    g->masked_mask|=(uint8_t)(1U<<axis);
    return true;
}
