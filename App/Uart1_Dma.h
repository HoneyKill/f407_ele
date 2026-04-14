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

typedef struct {
    uint32_t tx_enqueue_ok;
    uint32_t tx_enqueue_busy;
    uint32_t tx_enqueue_error;
    uint32_t tx_start_ok;
    uint32_t tx_start_fail;
    uint32_t tx_complete;
    uint32_t rx_bytes_pushed;
    uint32_t rx_overwrite;
    uint16_t rx_available;
    uint16_t last_tx_len;
    uint8_t tx_busy;
    uint8_t tx_ready_0;
    uint8_t tx_ready_1;
    uint8_t last_tx_data[8];
    uint8_t last_rx_data[4];
} UART1_DmaDebugInfo_t;

void UART1_DmaCommInit(void);
void UART1_DmaIrqHandler(void);

HAL_StatusTypeDef UART1_DmaTxEnqueue(const uint8_t *data, uint16_t len);
uint16_t UART1_DmaRxAvailable(void);
uint16_t UART1_DmaRxRead(uint8_t *dst, uint16_t max_len);
void UART1_DmaGetDebugInfo(UART1_DmaDebugInfo_t *info);

#ifdef __cplusplus
}
#endif

#endif
