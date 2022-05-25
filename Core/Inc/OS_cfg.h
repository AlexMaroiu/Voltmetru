#ifndef OS_CFG_H
#define OS_CFG_H

#include "OS.h"

#define NUMBER_OF_SCHEDULE_POINTS (13)
#define OS_TIMER_RELOAD_VALUE (1000ul)

typedef void (*pfunc_t)(void);
typedef struct
{
    uint32_t u32SchedulePoint;
    pfunc_t  pfPfunc;
} OS_tTaskLine;

extern void Task_1msTask(void);
extern void Task_5msTask(void);
extern void Task_10msTask(void);

extern OS_tTaskLine OS_astTasksTable[NUMBER_OF_SCHEDULE_POINTS];

#endif /* OS_CFG_H */
