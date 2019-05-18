/* 
 * Copyright (c) 2019 Joseph H. Gorse IV. All rights reserved.
 * 
 */

#ifndef _ARM_UART_CMSIS_H_
#define _ARM_UART_CMSIS_H_

#include "string.h"

#include "stm32f1xx.h"
#include "uart.h"
#include "gpio.h"
#include "util.h"
#include "circ_buf.h"

#include "Driver_USART.h"
#include "DAP_config.h" // for SWO_USART_PORT
#include "IO_Config.h"

// TODO: Support all UARTS of the stm32f103xb
// NOTE: USART 2 is used for DAPLink. See uart.c.

#if (SWO_USART_PORT == 1)
#define SWO_USARTx USART1
extern ARM_DRIVER_USART Driver_USART1;
#else
#error "Unsupported SWO_USART_PORT."
#endif /* UART1 */

#define ARM_UART_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)

/* USART Driver state flags */
#define USART_FLAG_UNINIT (0)
#define USART_FLAG_INIT (1 << 0)
#define USART_FLAG_POWER (1 << 1)
#define USART_FLAG_CONFIGURED (1 << 2)

#endif // _ARM_UART_CMSIS_H_
