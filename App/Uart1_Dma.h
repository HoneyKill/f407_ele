#ifndef __UART1_DMA_H
#define __UART1_DMA_H

#include <stdbool.h>
#include <stdint.h>

#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART1_RX_DMA_BUFFER_SIZE      (128U)
#define UART1_RX_RING_BUFFER_SIZE     (512U)
#define UART1_TX_DOUBLE_BUFFER_SIZE   (256U)

void UART1_DmaCommInit(void);
void UART1_DmaIrqHandler(void);

HAL_StatusTypeDef UART1_DmaTxEnqueue(const uint8_t *data, uint16_t len);
uint16_t UART1_DmaRxAvailable(void);
uint16_t UART1_DmaRxRead(uint8_t *dst, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif
