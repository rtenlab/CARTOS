//*****************************************************************************
//
//! @file ble_freertos_txpower_ctrl.c
//!
//! @brief ARM Cordio BLE - Transmit Power Control Example
//!
//! Purpose: This example demonstrates the control of BLE TX power level based
//! on pressing Button #0 on the Apollo4 EVB.
//!
//! Printing takes place over the ITM at 1M Baud.
//!
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

//*****************************************************************************
//
// This application has a large number of common include files. For
// convenience, we'll collect them all together in a single header and include
// that everywhere.
//
//*****************************************************************************
#include "rtos.h"
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define ARB_ADDRESS AM_HAL_MRAM_INSTANCE_SIZE
// static uint32_t ui32Source[512];

// static uint32_t ui32Info0[5] = {0x5B75A5FA, 0x7B9C8674, 0x869A96FE, 0xAEC90860,  // INFO0_SIGNATURE0-3
//                                0x00055FF }; // INFO0_SECURITY
//*****************************************************************************
//
// Enable printing to the console.
//

//*****************************************************************************
//
// Main Function
//
//*****************************************************************************

//void *phUART1;

//#define CHECK_ERRORS(x)                                                       \
//    if ((x) != AM_HAL_STATUS_SUCCESS)                                         \
//    {                                                                         \
//        error_handler(x);                                                     \
//    }

//volatile uint32_t ui32LastError;

//void
//error_handler(uint32_t ui32ErrorStatus)
//{
//    ui32LastError = ui32ErrorStatus;

//    while (1);
//}

//const am_hal_uart_config_t g_sUartConfig1 =
//{
//    //
//    // Standard UART settings: 115200-8-N-1
//    //
//    .ui32BaudRate = 115200,
//    .eDataBits = AM_HAL_UART_DATA_BITS_8,
//    .eParity = AM_HAL_UART_PARITY_NONE,
//    .eStopBits = AM_HAL_UART_ONE_STOP_BIT,
//    .eFlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

//    //
//    // Set TX and RX FIFOs to interrupt at half-full.
//    //
//    .eTXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16,
//    .eRXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16,
//};

//void
//am_uart_isr(void)
//{
//    //
//    // Service the FIFOs as necessary, and clear the interrupts.
//    //
//    uint32_t ui32Status;
//    am_hal_uart_interrupt_status_get(phUART1, &ui32Status, true);
//    am_hal_uart_interrupt_clear(phUART1, ui32Status);
//    am_hal_uart_interrupt_service(phUART1, ui32Status);
//}



//*****************************************************************************
//
// UART handle.
//
//*****************************************************************************
void *phUART0;

//*****************************************************************************
//
// UART buffers.
//
//*****************************************************************************
uint8_t g_pui8TxBuffer[512];
uint8_t g_pui8RxBuffer[256];

//*****************************************************************************
//
// UART configuration.
//
//*****************************************************************************
const am_hal_uart_config_t g_sUartConfig =
{
    //
    // Standard UART settings: 115200-8-N-1
    //
    .ui32BaudRate = 115200,
    .eDataBits = AM_HAL_UART_DATA_BITS_8,
    .eParity = AM_HAL_UART_PARITY_NONE,
    .eStopBits = AM_HAL_UART_ONE_STOP_BIT,
    .eFlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

    //
    // Set TX and RX FIFOs to interrupt at half-full.
    //
    .eTXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16,
    .eRXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16,
};

//*****************************************************************************
//
// UART0 interrupt handler.
//
//*****************************************************************************
void
am_uart_isr(void)
{
    //
    // Service the FIFOs as necessary, and clear the interrupts.
    //
    uint32_t ui32Status;
    am_hal_uart_interrupt_status_get(phUART0, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART0, ui32Status);
    am_hal_uart_interrupt_service(phUART0, ui32Status);
}

//*****************************************************************************
//
// UART print string
//
//*****************************************************************************

void
uart_print(char *pcStr)
{
    uint32_t ui32StrLen = 0;
    uint32_t ui32BytesWritten = 0;

    //
    // Measure the length of the string.
    //
    while (pcStr[ui32StrLen] != 0)
    {
        ui32StrLen++;
    }

    //
    // Print the string via the UART.
    //
    const am_hal_uart_transfer_t sUartWrite =
    {
        .eType = AM_HAL_UART_BLOCKING_WRITE,
        .pui8Data = (uint8_t *) pcStr,
        .ui32NumBytes = ui32StrLen,
        .pui32BytesTransferred = &ui32BytesWritten,
        .ui32TimeoutMs = 100,
        .pfnCallback = NULL,
        .pvContext = NULL,
        .ui32ErrorStatus = 0
    };

    am_hal_uart_transfer(phUART0, &sUartWrite);

}




int
main(void)
{
    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();
    am_hal_uart_initialize(AM_BSP_UART_PRINT_INST, &phUART0);
    am_hal_uart_power_control(phUART0, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_uart_configure(phUART0, &g_sUartConfig);

    //
    // Enable the UART pins.
    //
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX);

    //
    // Enable interrupts.
    //
    NVIC_SetPriority((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST), AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));
    am_hal_interrupt_master_enable();

    //
    // Set the main print interface to use the UART print function we defined.
    //
    am_util_stdio_printf_init(uart_print);

    am_util_stdio_terminal_clear();
       
    // //
    // // Initialize the printf interface for ITM output
    // //
    // if (am_bsp_debug_printf_enable())
    // {
    //     // Cannot print - so no point proceeding
    //     while(1);
    // }

    // //
    // // Print the banner.
    // //
    // am_util_stdio_terminal_clear();
    am_util_stdio_printf("Testing!\n\n");
    run_tasks();

    //
    // We shouldn't ever get here.
    //
    while (1)
    {
    }

}

