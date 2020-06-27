/**
 * @file		sys_time.c
 * @author		Andrew Loebs
 * @brief		Implementation file of the sys time module
 * 
 */

#include "sys_time.h"

#include "../st/stm32l4xx.h"
#include "../st/ll/stm32l4xx_ll_utils.h"

#define SYSTICK_PREEMPT_PRIORITY        0
#define SYSTICK_SUB_PRIORITY            0

// private variables
uint32_t sys_time_ms = 0;

// public function definitions
void sys_time_init(void)
{
    sys_time_ms = 0;

    NVIC_SetPriority(SysTick_IRQn, 
            NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 
            SYSTICK_PREEMPT_PRIORITY, SYSTICK_SUB_PRIORITY));
    SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
}

void _sys_time_increment(void)
{
    sys_time_ms++;
}

uint32_t sys_time_get_ms(void)
{
    return sys_time_ms;
}

bool sys_time_is_elapsed(uint32_t start, uint32_t duration_ms)
{
    uint32_t current = sys_time_ms; // snapshot the current time
    uint32_t end = start + duration_ms;

    // if current has overflowed
    if (current < start)
    {
        if (end >= start)
        {
            // end didn't overflow, so the time must have elapsed
            return true;
        }
    }
    // if current has not overflowed
    else
    {
        if (end < start)
        {
            // end overflowed, so the time cannot have elapsed
            return false;
        }
    }

    // they either both overflowed, or both did not overflow, so just compare
    return (current >= end);
}

void sys_time_delay(uint32_t duration_ms)
{
    uint32_t start = sys_time_ms;
    while (!sys_time_is_elapsed(start, duration_ms));
}