/** 上电/等待阶段的已压住触点确认。输出锁定下先停再屏蔽，仅接管实际低电平轴。
 * 10ms连续低、100ms确认期限；确认后高采样撤销START资格并重新确认。
 * 重新确认期间不得动作，高电平持续/不稳定到100ms则失败，不清过载。
 * 与运行中CONTACT_IRQ_MASK分开记录，避免把启动触点当作运动后的接触成果。 */
#ifndef SEEK_BOOT_CONTACT_H
#define SEEK_BOOT_CONTACT_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    uint8_t masked,stable,low_seen;
    bool failed;
    uint32_t began[3],low_since[3];
} SeekBootContact;
bool SeekBoot_OnEdge(SeekBootContact *b,bool waiting,uint8_t axis,uint8_t input,uint8_t latch,uint32_t now);
void SeekBoot_Tick(SeekBootContact *b,uint8_t input,uint32_t now);
bool SeekBoot_Ready(const SeekBootContact *b,uint8_t input);
bool SeekBoot_ReleasedBeforeContact(const SeekBootContact *b,uint8_t completed,uint8_t input);
#endif
