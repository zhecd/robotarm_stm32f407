#ifndef SEEK_RESULT_H
#define SEEK_RESULT_H
#include "seek_logic.h"
#include "trial_logic.h"
/** 主循环报告已锁存的首个试验结果，不把完成/过载统称为主机停止。 */
void Seek_ReportTerminal(TrialReason trial_reason, SeekReason seek_reason);
#endif
