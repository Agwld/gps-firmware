/**
 * @file    FreeRTOSConfig.h
 * @brief   FreeRTOS kernel configuration for the SUFST GPS node.
 *
 * Fully static allocation: no heap, no software timers. Regenerating with
 * CubeMX must NOT re-introduce cmsis_os2.c / heap_4.c — see the
 * "Regenerating with CubeMX" checklist in README.md.
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(HOST_TEST_BUILD)
#include <stdint.h>
extern uint32_t SystemCoreClock;
#include "fusion/timebase.h"
#endif

#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    (8)
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)
#define configMAX_TASK_NAME_LEN                 (12)
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  1
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0

/* Memory: static only */
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        0
#define configTOTAL_HEAP_SIZE                   ((size_t)0)
#define configAPPLICATION_ALLOCATED_HEAP        0

/* Hooks */
#define configUSE_IDLE_HOOK                     1
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* Run time and task stats.
 * Reuses the microsecond tick already free-running on TIM3 for GPS PPS
 * timing (fusion/timebase.c) as the run-time-stats clock: no dedicated
 * peripheral needed, and its ~71 min software-extended wrap period
 * comfortably outlasts a race session. Host test builds never link
 * FreeRTOS, so this is a no-op there. */
#if defined(HOST_TEST_BUILD)
#define configGENERATE_RUN_TIME_STATS           0
#else
#define configGENERATE_RUN_TIME_STATS           1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() ((void) 0)
#define portGET_RUN_TIME_COUNTER_VALUE()        timebase_get_tick()
#endif
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* Co-routines and software timers: unused */
#define configUSE_CO_ROUTINES                   0
#define configUSE_TIMERS                        0

/* API functions included */
#define INCLUDE_vTaskPrioritySet                0
#define INCLUDE_uxTaskPriorityGet               0
#define INCLUDE_vTaskDelete                     0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          0
#define INCLUDE_eTaskGetState                   0
#define INCLUDE_xTimerPendFunctionCall          0
#define INCLUDE_xTaskAbortDelay                 0
#define INCLUDE_xTaskGetHandle                  0
#define INCLUDE_xTaskResumeFromISR              0

/* Cortex-M4F interrupt priority configuration.
 * The G4 implements 4 priority bits. ISRs calling FromISR APIs must run at
 * numeric priority >= 5 (i.e. logically lower than MAX_SYSCALL). */
#define configPRIO_BITS                         4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Map FreeRTOS port handlers onto the CMSIS vector names */
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
/* SysTick_Handler is provided in stm32g4xx_it.c and chains HAL_IncTick +
 * xPortSysTickHandler, so it is NOT mapped here. */

#define configASSERT(x)                                                        \
    if ((x) == 0) {                                                            \
        taskDISABLE_INTERRUPTS();                                              \
        for (;;)                                                               \
            ;                                                                  \
    }

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_CONFIG_H */
