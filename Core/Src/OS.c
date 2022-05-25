
#include "main.h"
#include "OS_cfg.h"

void OS_Init (void)
{
  //this is the value of OS counter roll over
  SysTick_Config(OS_TIMER_RELOAD_VALUE);;
}

//https://www.youtube.com/watch?v=VfbW6nfG4kw

void OS_Main(void)
{
  /* Holds the OS counter value */
  uint32_t u32CurrentSysCount = 0u;

  /* Correction factor for the diff computing when counter rolls over */
  uint32_t u32ReloadValue = 0u;

  /* Diff value used to execute the task of the current selected schedule point */
  int32_t s32Diff = 0u;

  /* Selected task to be executed from scheduling table */
  uint8_t u8TaskToExecute = 0u;

  /* Synchronize with the OS timer: give a little more time after counter roll over */
  //while (SysTickValueGet() < (OS_TIMER_RELOAD_VALUE - 100u));

  while (1)
  {
    /* Capture the OS counter */
    u32CurrentSysCount = SysTick->VAL;

    /* Detect the sys_counter roll over, but exclude the last point from reloading calculation */
    if ( (u32CurrentSysCount >= OS_astTasksTable[0].u32SchedulePoint) && (u8TaskToExecute != 0))
    {
      u32ReloadValue = OS_TIMER_RELOAD_VALUE;
    }
    else
    {
      u32ReloadValue = 0;
    }

    /* Calculate remaining ticks until scheduling point */
    s32Diff = u32ReloadValue + OS_astTasksTable[u8TaskToExecute].u32SchedulePoint - u32CurrentSysCount;

    if (s32Diff >= 0)
    {
      /* Execute the task */
      (OS_astTasksTable[u8TaskToExecute].pfPfunc)();

      /* make the schedule table parsing circular */
      if (u8TaskToExecute < (NUMBER_OF_SCHEDULE_POINTS - 1))
      {
        u8TaskToExecute += 1;
      }
      else
      {
        u8TaskToExecute = 0;
        SysTick->VAL = 0;
      }
    }
  }
}






