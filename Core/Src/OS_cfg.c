
#include "OS_cfg.h"
#include "main.h"
#include <stdio.h>

#define US_TO_TICKS(x) (x*100)   /* 1/48MHz */
//UART_HandleTypeDef huart2;

/* Scheduling points should be configured in microseconds */
OS_tTaskLine OS_astTasksTable[NUMBER_OF_SCHEDULE_POINTS]=
  {
	/* Point_in_microseconds,  name_of_task */
	{ US_TO_TICKS(9000UL),   Task_1msTask },
	{ US_TO_TICKS(8000UL),   Task_1msTask },
	{ US_TO_TICKS(7000UL),   Task_1msTask },
	{ US_TO_TICKS(6000UL),   Task_1msTask },
	{ US_TO_TICKS(5500UL),   Task_5msTask },
	{ US_TO_TICKS(5000UL),   Task_1msTask },
	{ US_TO_TICKS(4000UL),   Task_1msTask },
	{ US_TO_TICKS(3000UL),   Task_1msTask },
	{ US_TO_TICKS(2500UL),   Task_10msTask},
	{ US_TO_TICKS(2000UL),   Task_1msTask },
	{ US_TO_TICKS(1000UL),   Task_1msTask },
	{ US_TO_TICKS(500UL ),   Task_5msTask },
	{ US_TO_TICKS(0UL   ),   Task_1msTask }
  };

void Task_1msTask(void)
{
	//char uart_buf[50];
	//int uart_buf_len;
	//uart_buf_len = sprintf(uart_buf, " 1 ms task\r\n");
	//HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf,uart_buf_len, 100);
}

void Task_5msTask(void)
{
	//HAL_GPIO_WritePin(GPIOA, backlight_Pin, GPIO_PIN_RESET);
	//char uart_buf[50];
	//int uart_buf_len;
	//uart_buf_len = sprintf(uart_buf, " 5 ms task\r\n");
	//HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf,uart_buf_len, 100);
}

void Task_10msTask(void)
{
    //HAL_GPIO_TogglePin(GPIOA, backlight_Pin);
    asm("NOP");
    //char uart_buf[50];
    //int uart_buf_len;
   // uart_buf_len = sprintf(uart_buf, "10 ms task\r\n");
   // HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf,uart_buf_len, 100);
}






