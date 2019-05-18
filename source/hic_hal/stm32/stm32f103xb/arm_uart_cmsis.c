/* 
 * Copyright (c) 2019 Joseph H. Gorse IV. All rights reserved.
 * 
 */
 // For reference, see the implemented version for k26f:
 //   fsl_uart_cmsis.c/h

#include "arm_uart_cmsis.h"

typedef void (*uart_isr_t)(USART_TypeDef *base, void *uart_int_xfr_handle);

typedef const struct _cmsis_uart_resource
{
    USART_TypeDef *base;       /*!< uart peripheral base address.      */
    uint32_t (*GetFreq)(void); /*!< Function to get the clock frequency. */
} cmsis_uart_resource_t;

typedef struct _cmsis_uart_interrupt_driver_state
{
    cmsis_uart_resource_t *resource;  /*!< Basic uart resource. */
    void *handle;            /*!< Interupt transfer handle. */ // TODO: fix type for IRQ handler.
    ARM_USART_SignalEvent_t cb_event; /*!< Callback function.     */
    uint8_t flags;                    /*!< Control and state flags. */
} cmsis_uart_interrupt_driver_state_t;

/* Driver Version */
static const ARM_DRIVER_VERSION s_uartDriverVersion = {ARM_USART_API_VERSION, ARM_UART_DRV_VERSION};

static const ARM_USART_CAPABILITIES s_uartDriverCapabilities = {
    1, /* supports uart (Asynchronous) mode */
    0, /* supports Synchronous Master mode */
    0, /* supports Synchronous Slave mode */
    0, /* supports uart Single-wire mode */
    0, /* supports uart IrDA mode */
    0, /* supports uart Smart Card mode */
    0, /* Smart Card Clock generator */
    0, /* RTS Flow Control available */
    0, /* CTS Flow Control available */
    0, /* Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE */
    0, /* Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT */
    0, /* RTS Line: 0=not available, 1=available */
    0, /* CTS Line: 0=not available, 1=available */
    0, /* DTR Line: 0=not available, 1=available */
    0, /* DSR Line: 0=not available, 1=available */
    0, /* DCD Line: 0=not available, 1=available */
    0, /* RI Line: 0=not available, 1=available */
    0, /* Signal CTS change event: \ref ARM_USART_EVENT_CTS */
    0, /* Signal DSR change event: \ref ARM_USART_EVENT_DSR */
    0, /* Signal DCD change event: \ref ARM_USART_EVENT_DCD */
    0, /* Signal RI change event: \ref ARM_USART_EVENT_RI */
};

// For usart
#define SWO_USARTx                  USART1
#define SWO_UART_ENABLE()           __HAL_RCC_USART1_CLK_ENABLE()
#define SWO_UART_DISABLE()          __HAL_RCC_USART1_CLK_DISABLE()
#define SWO_UART_IRQn               USART1_IRQn
#define SWO_UART_IRQn_Handler       USART1_IRQHandler

#define UART_PINS_PORT_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define UART_PINS_PORT_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()

#define UART_TX_PORT                GPIOA
#define UART_TX_PIN                 GPIO_PIN_9

#define UART_RX_PORT                GPIOA
#define UART_RX_PIN                 GPIO_PIN_10

#define UART_CTS_PORT               GPIOA
#define UART_CTS_PIN                GPIO_PIN_11

#define UART_RTS_PORT               GPIOA
#define UART_RTS_PIN                GPIO_PIN_12


#define RX_OVRF_MSG         "<DAPLink:SWO:Overflow>\n"
#define RX_OVRF_MSG_SIZE    (sizeof(RX_OVRF_MSG) - 1)
#define BUFFER_SIZE         (512)

static circ_buf_t write_buffer;
static uint8_t write_buffer_data[BUFFER_SIZE];
static circ_buf_t read_buffer;
static uint8_t read_buffer_data[BUFFER_SIZE];

static UART_Configuration configuration = {
    .Baudrate = 115200,
    .DataBits = UART_DATA_BITS_8,
    .Parity = UART_PARITY_NONE,
    .StopBits = UART_STOP_BITS_1,
    .FlowControl = UART_FLOW_CONTROL_NONE,
};

extern uint32_t SystemCoreClock;

static void clear_buffers(void)
{
    circ_buf_init(&write_buffer, write_buffer_data, sizeof(write_buffer_data));
    circ_buf_init(&read_buffer, read_buffer_data, sizeof(read_buffer_data));
}

int32_t swo_uart_set_configuration(UART_Configuration *config)
{
    UART_HandleTypeDef uart_handle;
    HAL_StatusTypeDef status;

    memset(&uart_handle, 0, sizeof(uart_handle));
    uart_handle.Instance = SWO_USARTx;

    // parity
    configuration.Parity = config->Parity;
    if(config->Parity == UART_PARITY_ODD) {
        uart_handle.Init.Parity = HAL_UART_PARITY_ODD;
    } else if(config->Parity == UART_PARITY_EVEN) {
        uart_handle.Init.Parity = HAL_UART_PARITY_EVEN;
    } else if(config->Parity == UART_PARITY_NONE) {
        uart_handle.Init.Parity = HAL_UART_PARITY_NONE;
    } else {   //Other not support
        uart_handle.Init.Parity = HAL_UART_PARITY_NONE;
        configuration.Parity = UART_PARITY_NONE;
    }

    // stop bits
    configuration.StopBits = config->StopBits;
    if(config->StopBits == UART_STOP_BITS_2) {
        uart_handle.Init.StopBits = UART_STOPBITS_2;
    } else if(config->StopBits == UART_STOP_BITS_1_5) {
        uart_handle.Init.StopBits = UART_STOPBITS_2;
        configuration.StopBits = UART_STOP_BITS_2;
    } else if(config->StopBits == UART_STOP_BITS_1) {
        uart_handle.Init.StopBits = UART_STOPBITS_1;
    } else {
        uart_handle.Init.StopBits = UART_STOPBITS_1;
        configuration.StopBits = UART_STOP_BITS_1;
    }

    //Only 8 bit support
    configuration.DataBits = UART_DATA_BITS_8;
    uart_handle.Init.WordLength = UART_WORDLENGTH_8B;

    // No flow control
    configuration.FlowControl = UART_FLOW_CONTROL_NONE;
    uart_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    
    // Specified baudrate
    configuration.Baudrate = config->Baudrate;
    uart_handle.Init.BaudRate = config->Baudrate;

    // TX and RX
    uart_handle.Init.Mode = UART_MODE_TX_RX;
    
    // Disable uart and tx/rx interrupt
    SWO_USARTx->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);

    clear_buffers();

    status = HAL_UART_DeInit(&uart_handle);
    util_assert(HAL_OK == status);
    status = HAL_UART_Init(&uart_handle);
    util_assert(HAL_OK == status);
    (void)status;

    SWO_USARTx->CR1 |= USART_IT_RXNE;

    return 1;
}


static int32_t uart_get_configuration(UART_Configuration *config)
{
    config->Baudrate = configuration.Baudrate;
    config->DataBits = configuration.DataBits;
    config->Parity   = configuration.Parity;
    config->StopBits = configuration.StopBits;
    config->FlowControl = UART_FLOW_CONTROL_NONE;

    return 1;
}


// TODO: Support other UARTS
// TODO: Use HAL UART_HandleTypeDef instead of local circular buffer?
static int32_t UART1_NonBlockingInitialize(ARM_USART_SignalEvent_t cb_event)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    SWO_USARTx->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
    clear_buffers();

    SWO_UART_ENABLE();
    UART_PINS_PORT_ENABLE();

    //TX pin
    GPIO_InitStructure.Pin = UART_TX_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(UART_TX_PORT, &GPIO_InitStructure);
    //RX pin
    GPIO_InitStructure.Pin = UART_RX_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(UART_RX_PORT, &GPIO_InitStructure);
    //CTS pin, input
    GPIO_InitStructure.Pin = UART_CTS_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(UART_CTS_PORT, &GPIO_InitStructure);
    //RTS pin, output low
    HAL_GPIO_WritePin(UART_RTS_PORT, UART_RTS_PIN, GPIO_PIN_RESET);
    GPIO_InitStructure.Pin = UART_RTS_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(UART_RTS_PORT, &GPIO_InitStructure);

    NVIC_EnableIRQ(SWO_UART_IRQn);
    return 1;
}

static int32_t UART1_NonBlockingUninitialize(void)
{
    SWO_USARTx->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
    clear_buffers();
    return 1;
}

static int32_t UART1_NonBlockingPowerControl(ARM_POWER_STATE state)
{
		ARM_USART_SignalEvent_t cb_event;

    switch (state)
    {
        case ARM_POWER_OFF:
            UART1_NonBlockingUninitialize();
            break;
        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        case ARM_POWER_FULL:
            UART1_NonBlockingInitialize(cb_event);
            break;
        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

static int32_t UART1_NonBlockingSend(const void *data, uint32_t num)
{
  // TODO: Error check
  uint32_t cnt = circ_buf_write(&write_buffer, data, num);
  SWO_USARTx->CR1 |= USART_IT_TXE;

  return ARM_DRIVER_OK;
}

static int32_t UART1_NonBlockingReceive(void *data, uint32_t num)
{
  circ_buf_read(&read_buffer, data, num);
  return ARM_DRIVER_OK;
}

static int32_t UART1_NonBlockingTransfer(const void *data_out, void *data_in, uint32_t num)
{
  return ARM_DRIVER_ERROR;
}

static uint32_t UART1_NonBlockingGetTxCount(void)
{
  // TODO: Check if TX in progress
  return circ_buf_count_used(&write_buffer);
}

static uint32_t UART1_NonBlockingGetRxCount(void)
{
  // TODO: Check if RX in progress
  return circ_buf_count_used(&read_buffer);
}

static int32_t UART1_NonBlockingControl(uint32_t control, uint32_t arg)
{
  UART_Configuration config;
  
  uart_get_configuration(&config);

  switch (control & ARM_USART_CONTROL_Msk)
  {
    case ARM_USART_MODE_ASYNCHRONOUS:
      /* USART Baudrate */
      config.Baudrate = arg;
      break;

    case ARM_USART_MODE_SINGLE_WIRE:
      if (arg) // Baud
      {
        config.Baudrate = arg;
      }
      else 
      {
        config.Baudrate = 115200;
      }
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  // TODO: Handle ARM_USART_CONTROL_TX/RX commands
  
  switch (control & ARM_USART_PARITY_Msk)
  {
      case ARM_USART_PARITY_NONE:
          config.Parity = UART_PARITY_NONE;
          break;
      case ARM_USART_PARITY_EVEN:
          config.Parity = UART_PARITY_EVEN;
          break;
      case ARM_USART_PARITY_ODD:
          config.Parity = UART_PARITY_ODD;
          break;
      default:
          return ARM_USART_ERROR_PARITY;
  }
  switch (control & ARM_USART_STOP_BITS_Msk)
  {
      case ARM_USART_STOP_BITS_1:
          config.StopBits = UART_STOP_BITS_1;
          break;
      case ARM_USART_STOP_BITS_2:
          config.StopBits = UART_STOP_BITS_2;
          break;
      default:
          return ARM_USART_ERROR_STOP_BITS;
  }
  
  swo_uart_set_configuration(&config); // TODO: Maintain state of configuration.

  return ARM_DRIVER_OK;
}

static ARM_USART_STATUS UART1_NonBlockingGetStatus(void)
{
  UART_HandleTypeDef uart_handle;
  HAL_UART_StateTypeDef status;
  struct _ARM_USART_STATUS arm_status;
  
  memset(&uart_handle, 0, sizeof(uart_handle));
  uart_handle.Instance = SWO_USARTx;
  status = uart_handle.State;
  
  memset(&arm_status, 0, sizeof(arm_status));

  switch (status)
  {
    case HAL_UART_STATE_READY:
      break;
    case HAL_UART_STATE_BUSY_TX:
      arm_status.tx_busy = 1;
      break;
    case HAL_UART_STATE_BUSY_RX:
      arm_status.rx_busy = 1;
      break;
    case HAL_UART_STATE_BUSY:
    case HAL_UART_STATE_BUSY_TX_RX:
      arm_status.tx_busy = 1;
      arm_status.rx_busy = 1;
      break;
    case HAL_UART_STATE_TIMEOUT:
    case HAL_UART_STATE_RESET:
    case HAL_UART_STATE_ERROR:
    default:
      arm_status.tx_underflow = 1;
      arm_status.rx_overflow = 1;
      arm_status.rx_break = 1;
      arm_status.rx_framing_error = 1;
      arm_status.rx_parity_error = 1;  
      break;
  }
  return arm_status;
}

static ARM_DRIVER_VERSION UARTx_GetVersion(void)
{
    return s_uartDriverVersion;
}

static ARM_USART_CAPABILITIES UARTx_GetCapabilities(void)
{
    return s_uartDriverCapabilities;
}

static int32_t UARTx_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

void SWO_UART_IRQn_Handler(void)
{
    const uint32_t sr = SWO_USARTx->SR;

    if (sr & USART_SR_RXNE) {
        uint8_t dat = SWO_USARTx->DR;
        uint32_t free = circ_buf_count_free(&read_buffer);
        if (free > RX_OVRF_MSG_SIZE) {
            circ_buf_push(&read_buffer, dat);
        } else if (RX_OVRF_MSG_SIZE == free) {
            circ_buf_write(&read_buffer, (uint8_t*)RX_OVRF_MSG, RX_OVRF_MSG_SIZE);
        } else {
            // Drop character
        }
    }

    if (sr & USART_SR_TXE) {
        if (circ_buf_count_used(&write_buffer) > 0) {
            SWO_USARTx->DR = circ_buf_pop(&write_buffer);
        } else {
            SWO_USARTx->CR1 &= ~USART_IT_TXE;
        }
    }
}
static ARM_USART_MODEM_STATUS UARTx_GetModemStatus(void)
{
    ARM_USART_MODEM_STATUS modem_status;

    modem_status.cts = 0U;
    modem_status.dsr = 0U;
    modem_status.ri = 0U;
    modem_status.dcd = 0U;
    modem_status.reserved = 0U;

    return modem_status;
}

ARM_DRIVER_USART Driver_USART1 = {
    UARTx_GetVersion,      UARTx_GetCapabilities,
  // TODO: Support DMA
    UART1_NonBlockingInitialize,
    UART1_NonBlockingUninitialize,
    UART1_NonBlockingPowerControl,
    UART1_NonBlockingSend,
    UART1_NonBlockingReceive,
    UART1_NonBlockingTransfer,
    UART1_NonBlockingGetTxCount,
    UART1_NonBlockingGetRxCount,
    UART1_NonBlockingControl,
    UART1_NonBlockingGetStatus,
    UARTx_SetModemControl, UARTx_GetModemStatus};
