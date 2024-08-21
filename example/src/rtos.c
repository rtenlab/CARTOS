//*****************************************************************************
//
//! @file rtos.c
//!
//! @brief Essential functions to make the RTOS run correctly.
//!
//! These functions are required by the RTOS for ticking, sleeping, and basic
//! error checking.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2022, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_4_3_0-0ca7d78a2b of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"
#include "portable.h"
#include "adc_measure.h"
#include "fourier.h"
#include "sniptype.h"
#include "crc.h"
#include "snipmath.h"
#include <limits.h>
#include "sha.h"
#include "search.h"
#include <stdio.h>

#include <time.h>
#include "ddcmath.h"
void busy_delay_ms(uint32_t delay_v);
void busy_delay_ms(uint32_t delay_v)
{
	uint32_t Current_tick;
	for (int i = 0; i<delay_v; i++)
	{
		Current_tick = xTaskGetTickCount();
		while(Current_tick == xTaskGetTickCount());
	}
}
extern int FirstTimeExec;
void voltageTest_task(void *pvParameters);
void fft_task(void  *pvParameters);
void basicmath_task(void *pvParameters);
void sha_task(void *pvParameters);
void crc_task(void *pvParameters);
void stringSearch_task(void *pvParameters);

void atomic1_task(void *pvParameters);
void atomic2_task(void *pvParameters);

void mram_task(void *pvParameters);

#define crcPeriod 5000
#define atomic1Period 6000
#define shaPeriod 8000
#define fftPeriod 10000
#define stringSearchPeriod 15000
#define atomic2Period 60000
#define BasicMathPeriod 120000

#define mramPeriod 4000

#define atomic1Exec 300
#define atomic2Exec 4000


//#define shaPriority 9
//#define BasicMathPriority 5
//#define fftPriority 8
//#define stringSearchPriority 7
//#define crcPriority 11
//#define atomic1Priority 13 //10
//#define atomic2Priority 12 //6


#define shaPriority 9
#define BasicMathPriority 5
#define fftPriority 8
#define stringSearchPriority 7
#define crcPriority 11
#define atomic1Priority 10
#define atomic2Priority 6
// #define atomic1Priority 13
// #define atomic2Priority 12

#define mramPriority 7


#define VERBOSE 1

static StaticTask_t fftTaskTCBBuffer;
static StackType_t fftStack[256];


static StaticTask_t basicMathTaskTCBBuffer;
static StackType_t basicMathStack[256];


static StaticTask_t shaTaskTCBBuffer;
static StackType_t shaStack[256];

static StaticTask_t crcTaskTCBBuffer;
static StackType_t crcStack[256];

static StaticTask_t stringSearchTaskTCBBuffer;
static StackType_t stringSearchStack[256];

static StaticTask_t atomic1TaskTCBBuffer;
static StackType_t atomic1Stack[256];

static StaticTask_t atomic2TaskTCBBuffer;
static StackType_t atomic2Stack[256];


static StaticTask_t voltageTestTaskTCBBuffer;
static StackType_t voltageTestStack[256];

static StaticTask_t mramTaskTCBBuffer;
static StackType_t mramStack[256];

int measure_voltage(void);
void setSystemParams ( int (*f)(void), uint32_t cap, uint32_t ma);
extern uint8_t uart_init_done;


#if (configUSE_PERIODIC_TASKS == 1)
void setTaskPeriod(uint32_t period, uint32_t offset);
void waitForNextPeriod(void);
#endif
/* <RTEN-Lab> */

#if ( ( configUSE_ENERGY_MON == 1 ) && (configUSE_TRACE_FACILITY == 1) )
    extern UBaseType_t nvmCheckpointAvailable;
#else
    UBaseType_t nvmCheckpointAvailable = 0;
#endif

extern void *phUART0;


/* </RTEN-Lab> */



//*****************************************************************************
//
// Task handle for the initial setup task.
//
//*****************************************************************************
TaskHandle_t xSetupTask;

//*****************************************************************************
//
// Interrupt handler for the CTIMER module.
//
//*****************************************************************************
void
am_timer_isr(void)
{
    uint32_t ui32Status;

    am_hal_timer_interrupt_status_get(false, &ui32Status);
    am_hal_timer_interrupt_clear(ui32Status);

    // we don't have this function in new hal
    //  am_hal_ctimer_int_service(ui32Status);
}

//*****************************************************************************
//
// Sleep function called from FreeRTOS IDLE task.
// Do necessary application specific Power down operations here
// Return 0 if this function also incorporates the WFI, else return value same
// as idleTime
//
//*****************************************************************************
uint32_t am_freertos_sleep(uint32_t idleTime)
{
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    return 0;
}

//*****************************************************************************
//
// Recovery function called from FreeRTOS IDLE task, after waking up from Sleep
// Do necessary 'wakeup' operations here, e.g. to power up/enable peripherals etc.
//
//*****************************************************************************
void am_freertos_wakeup(uint32_t idleTime)
{
    return;
}

//*****************************************************************************
//
// FreeRTOS debugging functions.
//
//*****************************************************************************
void
vApplicationMallocFailedHook(void)
{
    //
    // Called if a call to pvPortMalloc() fails because there is insufficient
    // free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    // internally by FreeRTOS API functions that create tasks, queues, software
    // timers, and semaphores.  The size of the FreeRTOS heap is set by the
    // configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
    //
    while (1);
}

void
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pcTaskName;
    (void) pxTask;

    //
    // Run time stack overflow checking is performed if
    // configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    // function is called if a stack overflow is detected.
    //
    while (1)
    {
        __asm("BKPT #0\n") ; // Break into the debugger
    }
}

//*****************************************************************************
//
// High priority task to run immediately after the scheduler starts.
//
// This task is used for any global initialization that must occur after the
// scheduler starts, but before any functional tasks are running. This can be
// useful for enabling events, semaphores, and other global, RTOS-specific
// features.
//
//*****************************************************************************
void
setup_task(void *pvParameters)
{

    //
    // Run setup functions.
    //
        //
    // Initialize the am_util_stdio_printf interface for ITM output
    //
//    am_bsp_itm_printf_enable();

    //
    // Clear the terminal screen, and print a quick message to show that we're
    // alive.
    //
//    am_util_stdio_terminal_clear();
    //
    // Create the functional tasks
    //
//    xTaskCreate(RadioTask, "RadioTask", 256, 0, 3, 1, 1000, &radio_task_handle);
//		xTaskCreate(fft_task, "fft_task", 256, 0, 3, 1, 1000, &fft_task_handl);

    //
    // The setup operations are complete, so suspend the setup task now.
    //
    vTaskSuspend(NULL);

    while (1);
}

//*****************************************************************************
//
// Initializes all tasks
//
//*****************************************************************************
void run_tasks(void)
{
    //
    // Set some interrupt priorities before we create tasks or start the scheduler.
    //
    // Note: Timer priority is handled by the FreeRTOS kernel, so we won't
    // touch it here.
    //
        adc_init();

    //
    // Create essential tasks.
    //
        #if (configUSE_ENERGY_MON == 1)
            uint32_t ma = 15;
            uint32_t cap = 100;
            setSystemParams(measure_voltage, cap, ma);
        #endif
        am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
    for (int ix = 0; ix < AM_BSP_NUM_LEDS; ix++)
    {
        am_devices_led_off(am_bsp_psLEDs, ix);
    }
    am_hal_gpio_pinconfig(3, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_clear(3);

//    xTaskCreate(setup_task, "Setup", 512, 0, 3, 1, 1000, &xSetupTask);
		// xTaskCreate(fft_task, "fft_task", 4096, 0, 3, 1, 1000, &fft_task_handle);
    // #define voltageTaskPriority 15
    // xTaskCreateStatic(voltageTest_task, "voltageTestTask", 256, 0, voltageTaskPriority, 0, 1000, &voltageTestStack[0], &voltageTestTaskTCBBuffer);



    xTaskCreateStatic(crc_task, "crc_task", 256, 0, crcPriority, 1, 3000, &crcStack[0], &crcTaskTCBBuffer);
    xTaskCreateStatic(atomic1_task, "atomic1_task", 256, 0, atomic1Priority, 0, 3042, &atomic1Stack[0], &atomic1TaskTCBBuffer);
    xTaskCreateStatic(sha_task, "sha_task", 256, 0, shaPriority, 1, 3000, &shaStack[0], &shaTaskTCBBuffer);
    xTaskCreateStatic(fft_task, "fft_task", 256, 0, fftPriority, 1, 3000, &fftStack[0], &fftTaskTCBBuffer);
    xTaskCreateStatic(stringSearch_task, "strSearch_task", 256, 0, stringSearchPriority, 1, 3000, &stringSearchStack[0], &stringSearchTaskTCBBuffer);
    xTaskCreateStatic(atomic2_task, "atomic2_task", 256, 0, atomic2Priority, 0, 3913, &atomic2Stack[0], &atomic2TaskTCBBuffer);
    xTaskCreateStatic(basicmath_task, "basicmath_task", 256, 0, BasicMathPriority, 1, 3000, &basicMathStack[0], &basicMathTaskTCBBuffer);
		
		
    // xTaskCreateStatic(mram_task, "mram_task", 256, 0, mramPriority, 0, 1000, &mramStack[0], &mramTaskTCBBuffer);

//			xCheckTaskHandle = xTaskCreateStatic(	prvJITCheckTask,
//													configCHECK_TASK_NAME,
//													xCheckTaskStackSize,
//													( void * ) NULL, /*lint !e961.  The cast is not redundant for all compilers. */
//													xcheckTaskPriority,
//													check_task_preemption,
//													check_task_voltage,
//													&xCheckStack[0],
//													&xCheckTaskTCBBuffer );
        //pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, preemption_enable voltage_required, TaskHandle_t )
        
    //
    // Start the scheduler.
    //
    vTaskStartScheduler();
        while(1);
}


void atomic1_task(void *pvParameters)
{
    #if (configUSE_PERIODIC_TASKS == 1)
    const int offset = 0;
    setTaskPeriod(atomic1Period, offset); // Set the task's period
    if (FirstTimeExec == 1 && offset !=0)
            vTaskDelay(offset);
    #endif
    #define RES_100_OHM_PIN 55
    am_hal_gpio_pincfg_t output_pin_config = am_hal_gpio_pincfg_output;
    output_pin_config.GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_1P0X;
    am_hal_gpio_pinconfig(RES_100_OHM_PIN, output_pin_config);
    am_hal_gpio_output_clear(RES_100_OHM_PIN);

    while(1)
    {
        am_hal_gpio_output_set(RES_100_OHM_PIN);
        long start = xTaskGetTickCount();
        #ifdef VERBOSE 
        am_util_stdio_printf("#%d ATOMIC1 task started, Absolute Time: %lu\n", atomic1Priority, start);
        am_hal_uart_tx_flush(phUART0);
        #endif
        busy_delay_ms(atomic1Exec);
        long end = xTaskGetTickCount();
        long time_spent = end - start;

        #ifdef VERBOSE 
        am_util_stdio_printf("#%d ATOMIC1 Time: %ld, Period: %d, Absolute time: %lu\n",atomic1Priority,time_spent, atomic1Period, end);
        am_hal_uart_tx_flush(phUART0);
        #endif
        am_hal_gpio_output_clear(RES_100_OHM_PIN);

        #if (configUSE_PERIODIC_TASKS == 1)
        waitForNextPeriod(); // suspends the task until next period is reached
        #endif

    }
}


void atomic2_task(void *pvParameters)
{
    #if (configUSE_PERIODIC_TASKS == 1)
    const int offset = 0;
    setTaskPeriod(atomic2Period, offset); // Set the task's period
    if (FirstTimeExec == 1 && offset !=0)
            vTaskDelay(offset);
    #endif
    #define RES_47_OHM_PIN 51
    am_hal_gpio_pincfg_t output_pin_config = am_hal_gpio_pincfg_output;
    output_pin_config.GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_1P0X;
    am_hal_gpio_pinconfig(RES_47_OHM_PIN, output_pin_config);
    am_hal_gpio_output_clear(RES_47_OHM_PIN);
    // GPIO_PINCFG50_DS50
    while(1)
    {
        am_hal_gpio_output_set(RES_47_OHM_PIN);
        long start = xTaskGetTickCount();
        #ifdef VERBOSE 
        am_util_stdio_printf("#%d ATOMIC2 task started, Absolute Time: %lu\n", atomic2Priority,start);
        am_hal_uart_tx_flush(phUART0);
        #endif

        busy_delay_ms(atomic2Exec);
        long end = xTaskGetTickCount();
        long time_spent = end - start;

        #ifdef VERBOSE 
        am_util_stdio_printf("#%d ATOMIC2 Time: %ld, Period: %d, Absolute time: %lu\n",atomic2Priority,time_spent, atomic2Period,end);
        am_hal_uart_tx_flush(phUART0);
        #endif
        am_hal_gpio_output_clear(RES_47_OHM_PIN);
        #if (configUSE_PERIODIC_TASKS == 1)
        waitForNextPeriod(); // suspends the task until next period is reached
        #endif

    }
}



void
voltageTest_task(void *pvParameters)
{
    // #define INPUT_GPIO_PIN 12
    // am_hal_gpio_pinconfig(INPUT_GPIO_PIN, am_hal_gpio_pincfg_input);


    // long cntr = 0;
    // if (cntr>100 || cntr <-100)
    //     cntr = 0;
    // int input_state = 0;
    // long start = 0;
    // long end = 0;
    while(1)
    {
//    am_util_stdio_terminal_clear();
        int adc_voltage = measure_voltage();

        #ifdef VERBOSE
         
        am_util_stdio_printf("Measured voltage: %d, Aboslute Time: %lu!\r\n",adc_voltage, xTaskGetTickCount());
        am_hal_uart_tx_flush(phUART0);
        #endif

        busy_delay_ms(50);
        /*
        cntr++;
        int print_once = 0;
        int hit_cntr = 0;
        for (long i=0; i<5000000; i++)
        {
            int input_value = am_hal_gpio_input_read(INPUT_GPIO_PIN);
            if (print_once ==0 && input_value!= input_state)
            {
                hit_cntr++;
                if (hit_cntr >10)
                {
                    end = xTaskGetTickCount();
                    adc_voltage = measure_voltage();
                    long time_spent = end - start;

                    am_util_stdio_printf("Input Value: %d, State: %d, Voltage: %d, Absolute Time: %lu,  Transition Time: %ld\n\n",input_value, input_state, adc_voltage,end, time_spent);
                    start = xTaskGetTickCount();

                    input_state = (input_state ^ 1)&0x01;
                    print_once = 1;
                    hit_cntr = 0;
                }
            }
            else
            {
                hit_cntr = 0;
            }
        }
			
			
        
        for (int ix = 0; ix < AM_BSP_NUM_LEDS; ix++)
    {
                am_devices_led_toggle(am_bsp_psLEDs, ix);
            
    }
    */

    // long end = xTaskGetTickCount();
    //     long time_spent = end - start;

    //     #ifdef VERBOSE 
    //     am_util_stdio_printf("Time: %ld\n\n",time_spent);
    //     #endif
//  clock_t end = clock();
//  double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
//	am_util_stdio_printf("Time: %.9f\n\n",time_spent);
    }
        
}

#define rw_size 1280/4 //number of 32 bit values (x4 to get number of bytes)
uint32_t sram_rd[rw_size];
uint32_t sram_wr[rw_size];

void mram_task(void *pvParameters)
{
	  uint32_t nvmRdPtrStart = AM_HAL_MRAM_INSTANCE_SIZE + 128*1024;
//		uint32_t *nvmRdPtr = (uint32_t *) nvmRdPtrStart;
		uint32_t nvmWrPtrStart = nvmRdPtrStart + rw_size;
		uint32_t *nvmWrPtr = (uint32_t *) nvmWrPtrStart;	
	
//    #define nvmTCBListSize ((sizeof(TCB_t)*tskNVM_TCB_NUM)+ (16 - (sizeof(TCB_t)*tskNVM_TCB_NUM)%16))/4 // size in words (4 byts) and aligned to 16 bytes for MRAM
//    #define nvmTCBListEnd (nvmTCBListStart + nvmTCBListSize*4)
//    uint32_t *nvmTCBListPtr = (uint32_t *) nvmTCBListStart;

		#if (configUSE_PERIODIC_TASKS == 1)
		const int offset = 0;
		setTaskPeriod(mramPeriod, offset); // Set the task's period
		if (FirstTimeExec == 1 && offset !=0)
				vTaskDelay(offset);
		#endif
//		int output =0;
		#define RUNS 1024
		#define TEST_GPIO_PIN 3
    am_hal_gpio_pinconfig(TEST_GPIO_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_set(TEST_GPIO_PIN);
    // am_hal_gpio_output_clear(3);
    // am_util_stdio_printf("TCB size: %d\n", (sizeof(StaticTask_t)));
    // am_util_stdio_printf("TCB size: %d\n", (sizeof(StaticTask_t)));
    
    while(1)
    {
        am_hal_gpio_output_toggle(TEST_GPIO_PIN);
//        #ifdef VERBOSE 
//        am_util_stdio_printf("#%d MRAM task started, Absolute Time: %lu\n", mramPriority);
//        #endif
//        long start = xTaskGetTickCount();
				register uint16_t i;
//				register uint16_t j;
//				int a = 0;
				for (i = 0; i<RUNS; i++)
				{
					// memcpy(sram_wr, nvmRdPtr, rw_size*sizeof(uint32_t));
//					for (j=0; j<rw_size; j++)
//					{
////						sram_wr[j] = sram_rd[j];
//						sram_wr[j] = *(nvmRdPtr+j);
//					}
            
					am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY,
												sram_rd,
												nvmWrPtr,
												rw_size);

				}
//				
//				
//				
//				
//        long end = xTaskGetTickCount();
//        long time_spent = end - start;

//        #ifdef VERBOSE 
//        am_util_stdio_printf("#%d MRAM Time: %ld, Output: %d, Period: %d\n\n",mramPriority,time_spent, output, mramPeriod);
//        #endif
//        #if (configUSE_PERIODIC_TASKS == 1)
////        waitForNextPeriod(); // suspends the task until next period is reached
//        #endif

    }
}

const unsigned MAXSIZE = 256;
const unsigned MAXWAVES = 256;

float RealIn[MAXSIZE];
float ImagIn[MAXSIZE];
float RealOut[MAXSIZE];
float ImagOut[MAXSIZE];
float coeff[MAXWAVES];
float amp[MAXWAVES];


void fft_task(void *pvParameters)
{

    #if (configUSE_PERIODIC_TASKS == 1)
    const int offset = 0;
    setTaskPeriod(fftPeriod, offset); // Set the task's period
    if (FirstTimeExec == 1 && offset !=0)
        vTaskDelay(offset);
    #endif
    while(1)
    {

        long start = xTaskGetTickCount();
        #ifdef VERBOSE 
        am_util_stdio_printf("#%d FFT task started, Absolute Time: %lu\n", fftPriority, start);
        am_hal_uart_tx_flush(phUART0);
        #endif
        unsigned i,j;
        // float *RealIn;
        // float *ImagIn;
        // float *RealOut;
        // float *ImagOut;
        // float *coeff;
        // float *amp;
        int invfft=0; // 0 for FFT and 1 for iFFT
            
        srand(1);
    
    //  RealIn=(float*)pvPortMalloc(sizeof(float)*MAXSIZE);
    //  ImagIn=(float*)pvPortMalloc(sizeof(float)*MAXSIZE);
    //  RealOut=(float*)pvPortMalloc(sizeof(float)*MAXSIZE);
    //  ImagOut=(float*)pvPortMalloc(sizeof(float)*MAXSIZE);
    //  coeff=(float*)pvPortMalloc(sizeof(float)*MAXWAVES);
    //  amp=(float*)pvPortMalloc(sizeof(float)*MAXWAVES);

     /* Makes MAXWAVES waves of random amplitude and period */
            for(i=0;i<MAXWAVES;i++) 
            {
                coeff[i] = rand()%1000;
                amp[i] = rand()%1000;
            }
         for(i=0;i<MAXSIZE;i++) 
         {
             /*   RealIn[i]=rand();*/
             RealIn[i]=0;
             for(j=0;j<MAXWAVES;j++) 
             {
                 /* randomly select sin or cos */
                 if (rand()%2)
                 {
                        RealIn[i]+=coeff[j]*cos(amp[j]*i);
                    }
                 else
                 {
                    RealIn[i]+=coeff[j]*sin(amp[j]*i);
                 }
                 ImagIn[i]=0;
             }
         }

     /* regular*/
            fft_float (MAXSIZE,invfft,RealIn,ImagIn,RealOut,ImagOut);
        
            long end = xTaskGetTickCount();
            long time_spent = end - start;

            #ifdef VERBOSE 
            am_util_stdio_printf("#%d FFT Time: %ld, Period: %d, Absolute time: %lu\n",fftPriority,time_spent,fftPeriod, end);
            am_hal_uart_tx_flush(phUART0);

            #endif
        #if (configUSE_PERIODIC_TASKS == 1)
            waitForNextPeriod(); // suspends the task until next period is reached
        #endif

    }
}

const int length = 512;
char input[512];
void crc_task(void *pvParameters)
{
    #if (configUSE_PERIODIC_TASKS == 1)
    const int offset = 0;
    setTaskPeriod(crcPeriod, offset); // Set the task's period
    if (FirstTimeExec == 1 && offset !=0)
        vTaskDelay(offset);
    #endif

//	input =(char*)pvPortMalloc(sizeof(char)*length);
//	DWORD crc;
    int runs = 1000;
    int cntr = 0;
    while(1)
    {

        long start = xTaskGetTickCount();
        #ifdef VERBOSE 
        am_util_stdio_printf("#%d CRC task started, Absolute Time: %lu\n", crcPriority, start);
        am_hal_uart_tx_flush(phUART0);
        #endif
        unsigned long output;
//	long charcnt;
//	register int errors = 0;
        for (int i =0; i<runs; i++)
        {
            
            output = crc32buf(input, length);
    //	for (int i=0; i<length; i++)
    //	{
    //				errors |= crc32file(input[i], &crc, &charcnt);
    //				am_util_stdio_printf("%08lX %7ld %s\n", crc, charcnt, *argv);
    //	}
    //	return output;
            if (output>0x7FFF)
                cntr++;
            else
                cntr--;
        }
            long end = xTaskGetTickCount();
            long time_spent = end - start;

        #ifdef VERBOSE 
        am_util_stdio_printf("#%d CRC Time: %ld    cntr: %d , Period: %d, Absolute time: %lu\n",crcPriority,time_spent, cntr/1000,crcPeriod, end);
        am_hal_uart_tx_flush(phUART0);

        #endif
        #if (configUSE_PERIODIC_TASKS == 1)
        waitForNextPeriod(); // suspends the task until next period is reached
        #endif
    }
}

void basicmath_task(void *pvParameters)
{
    #if (configUSE_PERIODIC_TASKS == 1)
    const int offset = 0;
    setTaskPeriod(BasicMathPeriod, offset); // Set the task's period
    if (FirstTimeExec == 1 && offset !=0)
        vTaskDelay(offset);
    #endif
  
    while(1)
    {

        long start = xTaskGetTickCount();
        #ifdef VERBOSE 
        am_util_stdio_printf("#%d BASIC MATH task started, Absolute Time: %lu\n", BasicMathPriority, start);
        am_hal_uart_tx_flush(phUART0);
        #endif
        int runs = 1;
        int j;

        for(j = 0; j < runs; j++) {
    //			am_util_stdio_printf("Basic Math run No: %d\n",j);

            double  a1 = 1.0, b1 = -10.5, c1 = 32.0, d1 = -30.0;
            double  x[3];
            double X;
            int     solutions;
            int i;
            unsigned long l = 0x3fed0169L;
            struct int_sqrt q;

            /* solve soem cubic functions */
            //      am_util_stdio_printf("********* CUBIC FUNCTIONS ***********\n");
            /* should get 3 solutions: 2, 6 & 2.5   */
            SolveCubic(a1, b1, c1, d1, &solutions, x);  
            //  am_util_stdio_printf("Solutions:");
            //  for(i=0;i<solutions;i++)
            //  am_util_stdio_printf(" %f",x[i]);
            //  am_util_stdio_printf("\n");

            a1 = 1.0; b1 = -4.5; c1 = 17.0; d1 = -30.0;
            /* should get 1 solution: 2.5           */
            SolveCubic(a1, b1, c1, d1, &solutions, x);  
            //      am_util_stdio_printf("Solutions:");
            //      for(i=0;i<solutions;i++)
            //        am_util_stdio_printf(" %f",x[i]);
            //      am_util_stdio_printf("\n");

            a1 = 1.0; b1 = -3.5; c1 = 22.0; d1 = -31.0;
            SolveCubic(a1, b1, c1, d1, &solutions, x);
//      am_util_stdio_printf("Solutions:");
//      for(i=0;i<solutions;i++)
//        am_util_stdio_printf(" %f",x[i]);
//      am_util_stdio_printf("\n");

            a1 = 1.0; b1 = -13.7; c1 = 1.0; d1 = -35.0;
            SolveCubic(a1, b1, c1, d1, &solutions, x);
//      am_util_stdio_printf("Solutions:");
//      for(i=0;i<solutions;i++)
//        am_util_stdio_printf(" %f",x[i]);
//      am_util_stdio_printf("\n");

            a1 = 3.0; b1 = 12.34; c1 = 5.0; d1 = 12.0;
            SolveCubic(a1, b1, c1, d1, &solutions, x);
//      am_util_stdio_printf("Solutions:");
//      for(i=0;i<solutions;i++)
//        am_util_stdio_printf(" %f",x[i]);
//      am_util_stdio_printf("\n");

            a1 = -8.0; b1 = -67.89; c1 = 6.0; d1 = -23.6;
            SolveCubic(a1, b1, c1, d1, &solutions, x);
//      am_util_stdio_printf("Solutions:");
//      for(i=0;i<solutions;i++)
//        am_util_stdio_printf(" %f",x[i]);
//      am_util_stdio_printf("\n");

            a1 = 45.0; b1 = 8.67; c1 = 7.5; d1 = 34.0;
            SolveCubic(a1, b1, c1, d1, &solutions, x);
//      am_util_stdio_printf("Solutions:");
//      for(i=0;i<solutions;i++)
//        am_util_stdio_printf(" %f",x[i]);
//      am_util_stdio_printf("\n");

        a1 = -12.0; b1 = -1.7; c1 = 5.3; d1 = 16.0;
            SolveCubic(a1, b1, c1, d1, &solutions, x);
    //      am_util_stdio_printf("Solutions:");
    //      for(i=0;i<solutions;i++)
    //        am_util_stdio_printf(" %f",x[i]);
    //      am_util_stdio_printf("\n");

        /* Now solve some random equations */
            for(a1=1;a1<10;a1+=1) {
                for(b1=10;b1>0;b1-=.25) {
                for(c1=5;c1<15;c1+=0.61) {
                for(d1=-1;d1>-5;d1-=.451) {
                    SolveCubic(a1, b1, c1, d1, &solutions, x);  
        //            am_util_stdio_printf("Solutions:");
        //            for(i=0;i<solutions;i++)
        //              am_util_stdio_printf(" %f",x[i]);
        //            am_util_stdio_printf("\n");
                }
                }
                }
            }


    //      am_util_stdio_printf("********* INTEGER SQR ROOTS ***********\n");
        /* perform some integer square roots */
            for (i = 0; i < 100000; i+=2)
                {
                usqrt(i, &q);
                        // remainder differs on some machines
                // am_util_stdio_printf("sqrt(%3d) = %2d, remainder = %2d\n",
        //         am_util_stdio_printf("sqrt(%3d) = %2d\n", i, q.sqrt);
                }
    //      am_util_stdio_printf("\n");
            for (l = 0x3fed0169L; l < 0x3fed4169L; l++)
                {
                usqrt(l, &q);
                //am_util_stdio_printf("\nsqrt(%lX) = %X, remainder = %X\n", l, q.sqrt, q.frac);
        //         am_util_stdio_printf("sqrt(%lX) = %X\n", l, q.sqrt);
                }


        //      am_util_stdio_printf("********* ANGLE CONVERSION ***********\n");
            /* convert some rads to degrees */
            /*   for (X = 0.0; X <= 360.0; X += 1.0) */
            for (X = 0.0; X <= 360.0; X += .001)
                        {
                            double res1 = deg2rad(X);
        //					am_util_stdio_printf("%3.0f degrees = %.12f radians\n", X, deg2rad(X));
                        }
                        
        //      puts("");
            /*   for (X = 0.0; X <= (2 * PI + 1e-6); X += (PI / 180)) */
            for (X = 0.0; X <= (2 * PI + 1e-6); X += (PI / 5760))
                        {
                        double res2 = rad2deg(X);
        //        am_util_stdio_printf("%.12f radians = %3.0f degrees\n", X, rad2deg(X));
                        }
        
        
        }
        long end = xTaskGetTickCount();
        long time_spent = end - start;

        #ifdef VERBOSE 
        am_util_stdio_printf("#%d BASIC MATH Time: %ld, Period: %d, Absolute time: %lu\n",BasicMathPriority,time_spent,BasicMathPeriod, end);
        am_hal_uart_tx_flush(phUART0);

        #endif

        #if (configUSE_PERIODIC_TASKS == 1)
        waitForNextPeriod(); // suspends the task until next period is reached
        #endif  
    }
}





////////////////////////////////////////////////////////
static size_t table[UCHAR_MAX + 1];

void stringSearch_task(void *pvParameters)
{
    #if (configUSE_PERIODIC_TASKS == 1)
    const int offset = 0;
    setTaskPeriod(stringSearchPeriod, offset); // Set the task's period
    if (FirstTimeExec == 1 && offset !=0)
        vTaskDelay(offset);
    #endif

    while(1)
    {

        #ifdef VERBOSE 
        am_util_stdio_printf("#%d String Search started\n", stringSearchPriority);
        #endif
        long start = xTaskGetTickCount();
        int runs = 100;
        int j;
        char *findme;
        size_t len;
        int cntr = 0;
        for(j = 0; j < runs; j++) {

                int i;

                for (i = 0; find_strings[i]; i++)
                {
                    ///////////  init_search
                    size_t k;
                    char* string = find_strings[i];
                    len = strlen(string);
                    for (k = 0; k <= UCHAR_MAX; k++)                      /* rdg 10/93 */
                                table[k] = len;
                    for (k = 0; k < len; k++)
                                table[(unsigned char)string[k]] = len - k - 1;
                    findme = (char *)string;
                    //////////   strsearch
                    string = search_strings[i];
                    register size_t shift;
                    register size_t pos = len - 1;
                    char *here = NULL;
                    size_t limit=strlen(string);
                    int found = 0;
                    while (pos < limit && found == 0)
                    {
                                while( pos < limit &&
                                            (shift = table[(unsigned char)string[pos]]) > 0)
                                {
                                            pos += shift;
                                }
                                if (0 == shift)
                                {
                                            here = (char *)&string[pos-len+1];
                                            if (0 == strncmp(findme, here, len))
                                            {
                                                        found = 1;
                                                        cntr++;
                                            }
                                            else  pos++;
                                }
                    }
                                    
                    // here = strsearch(search_strings[i]);
                    
                    // //////////////
                    // am_util_stdio_printf("\"%s\" is%s in \"%s\"", find_strings[i],
                    //             here ? "" : " not", search_strings[i]);
                    // if (found)
                    //             am_util_stdio_printf(" [\"%s\"]", here);
                    // putchar('\n');
                }
        }
        long end = xTaskGetTickCount();
        long time_spent = end - start;

        #ifdef VERBOSE 
        am_util_stdio_printf("#%d String Search Time: %ld    cntr: %d, Period: %d, Absolute time: %lu\n",stringSearchPriority,time_spent, cntr,stringSearchPeriod, end);
        am_hal_uart_tx_flush(phUART0);

        #endif
        #if (configUSE_PERIODIC_TASKS == 1)
        waitForNextPeriod(); // suspends the task until next period is reached
        #endif
    }
}


///////////////////////////////////////////////////////
const int SHA_DATA_SIZE = 64;
//BLOCK_SIZE
BYTE data[BLOCK_SIZE];
SHA_INFO sha_info;
void sha_task(void *pvParameters)
{
    #if (configUSE_PERIODIC_TASKS == 1)
    const int offset = 0;
    setTaskPeriod(shaPeriod, offset); // Set the task's period
    if (FirstTimeExec == 1 && offset !=0)
        vTaskDelay(offset);
    #endif

    int runs = 100;
    while(1)
    {

        long start = xTaskGetTickCount();
        #ifdef VERBOSE 
        am_util_stdio_printf("#%d SHA task started, Absolute Time: %lu\n", shaPriority, start);
        am_hal_uart_tx_flush(phUART0);
        #endif
        for (int i=0;i<runs;i++)
        {
//			vTaskDelay(10);
            sha_init(&sha_info);
            sha_update(&sha_info, data, BLOCK_SIZE);
            sha_final(&sha_info);
    //		sha_print(&sha_info);
        }
        long end = xTaskGetTickCount();
        long time_spent = end - start;

        #ifdef VERBOSE 
        am_util_stdio_printf("#%d SHA Time: %ld, Period: %d, Absolute time: %lu\n",shaPriority ,time_spent,shaPeriod, end);
        am_hal_uart_tx_flush(phUART0);
        #endif
        #if (configUSE_PERIODIC_TASKS == 1)
        waitForNextPeriod(); // suspends the task until next period is reached
        #endif    
    }

}
//////////////////////////////////////////////////////////


/* <RTEN-Lab> */
int measure_voltage(){
    return adc_read();
//	return 6000;
}
/**
 * @brief Sets the period of the current task when called first.
 * If the period is set before, it suspends the task until next period reaches
 * @param period period of the task to be set
*/
#if (configUSE_PERIODIC_TASKS == 1)
    void setTaskPeriod(uint32_t period, uint32_t offset)
    {
        xSetTaskPeriod(period / portTICK_PERIOD_MS, offset/portTICK_PERIOD_MS);
    }
    void waitForNextPeriod(void)
    {
        xWaitForNextPeriod();
    }
#endif
#if (configUSE_ENERGY_MON == 1)
    void setSystemParams ( int (*f)(void), uint32_t cap, uint32_t ma)
    {
        xSetVoltMeasureFunction (f);
        xSetChargingSettings(cap, ma);
    }
#endif
/* </RTEN-Lab> */
