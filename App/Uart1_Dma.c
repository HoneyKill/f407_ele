#include "Uart1_Dma.h"

#include <string.h>

typedef struct {
    uint8_t buf[2][UART1_TX_DOUBLE_BUFFER_SIZE];
    volatile uint16_t len[2];
    volatile uint8_t active;
    volatile uint8_t ready[2];
    volatile uint8_t busy;
} UART1_TxDoubleBuf_t;

static uint8_t s_uart1_rx_dma[UART1_RX_DMA_BUFFER_SIZE];
static uint8_t s_uart1_rx_ring[UART1_RX_RING_BUFFER_SIZE];
static volatile uint16_t s_uart1_rx_head = 0U;
static volatile uint16_t s_uart1_rx_tail = 0U;
static volatile uint16_t s_uart1_rx_dma_last_pos = 0U;

static UART1_TxDoubleBuf_t s_uart1_tx = {0};
static volatile uint32_t s_tx_enqueue_ok = 0U;
static volatile uint32_t s_tx_enqueue_busy = 0U;
static volatile uint32_t s_tx_enqueue_error = 0U;
static volatile uint32_t s_tx_start_ok = 0U;
static volatile uint32_t s_tx_start_fail = 0U;
static volatile uint32_t s_tx_complete = 0U;
static volatile uint32_t s_rx_bytes_pushed = 0U;
static volatile uint32_t s_rx_overwrite = 0U;
static uint8_t s_last_tx_data[8] = {0};
static uint8_t s_last_rx_data[4] = {0};
static volatile uint16_t s_last_tx_len = 0U;

static uint32_t UART1_Lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void UART1_Unlock(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static void UART1_RingPushByte(uint8_t data)
{
    uint16_t next = (uint16_t)((s_uart1_rx_head + 1U) % UART1_RX_RING_BUFFER_SIZE);

    if (next == s_uart1_rx_tail) {
        s_uart1_rx_tail = (uint16_t)((s_uart1_rx_tail + 1U) % UART1_RX_RING_BUFFER_SIZE);
        ++s_rx_overwrite;
    }

    s_uart1_rx_ring[s_uart1_rx_head] = data;
    s_uart1_rx_head = next;

    s_last_rx_data[0] = s_last_rx_data[1];
    s_last_rx_data[1] = s_last_rx_data[2];
    s_last_rx_data[2] = s_last_rx_data[3];
    s_last_rx_data[3] = data;
}

static void UART1_RingPushBlock(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    for (i = 0U; i < len; ++i) {
        UART1_RingPushByte(data[i]);
    }
    s_rx_bytes_pushed += len;
}

static void UART1_RxCheck(void)
{
    uint16_t pos;

    if (huart1.hdmarx == NULL) {
        return;
    }

    pos = (uint16_t)(UART1_RX_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx));

    if (pos == s_uart1_rx_dma_last_pos) {
        return;
    }

    if (pos > s_uart1_rx_dma_last_pos) {
        UART1_RingPushBlock(&s_uart1_rx_dma[s_uart1_rx_dma_last_pos],
                            (uint16_t)(pos - s_uart1_rx_dma_last_pos));
    } else {
        UART1_RingPushBlock(&s_uart1_rx_dma[s_uart1_rx_dma_last_pos],
                            (uint16_t)(UART1_RX_DMA_BUFFER_SIZE - s_uart1_rx_dma_last_pos));
        if (pos > 0U) {
            UART1_RingPushBlock(&s_uart1_rx_dma[0], pos);
        }
    }

    s_uart1_rx_dma_last_pos = pos;
}

static HAL_StatusTypeDef UART1_TxStart(uint8_t idx)
{
    HAL_StatusTypeDef status;

    s_uart1_tx.active = idx;
    status = HAL_UART_Transmit_DMA(&huart1, s_uart1_tx.buf[idx], s_uart1_tx.len[idx]);
    if (status == HAL_OK) {
        s_uart1_tx.busy = 1U;
        ++s_tx_start_ok;
    } else {
        ++s_tx_start_fail;
    }

    return status;
}

void UART1_DmaCommInit(void)
{
    memset(s_uart1_rx_dma, 0, sizeof(s_uart1_rx_dma));
    memset(s_uart1_rx_ring, 0, sizeof(s_uart1_rx_ring));
    memset(&s_uart1_tx, 0, sizeof(s_uart1_tx));
    memset(s_last_tx_data, 0, sizeof(s_last_tx_data));
    memset(s_last_rx_data, 0, sizeof(s_last_rx_data));

    s_uart1_rx_head = 0U;
    s_uart1_rx_tail = 0U;
    s_uart1_rx_dma_last_pos = 0U;
    s_tx_enqueue_ok = 0U;
    s_tx_enqueue_busy = 0U;
    s_tx_enqueue_error = 0U;
    s_tx_start_ok = 0U;
    s_tx_start_fail = 0U;
    s_tx_complete = 0U;
    s_rx_bytes_pushed = 0U;
    s_rx_overwrite = 0U;
    s_last_tx_len = 0U;

    if (HAL_UART_Receive_DMA(&huart1, s_uart1_rx_dma, UART1_RX_DMA_BUFFER_SIZE) != HAL_OK) {
        Error_Handler();
    }

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

void UART1_DmaIrqHandler(void)
{
    if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) &&
        (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE) != RESET)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        UART1_RxCheck();
    }
}

HAL_StatusTypeDef UART1_DmaTxEnqueue(const uint8_t *data, uint16_t len)
{
    uint8_t idx;
    uint32_t primask;
    HAL_StatusTypeDef status = HAL_OK;

    if ((data == NULL) || (len == 0U) || (len > UART1_TX_DOUBLE_BUFFER_SIZE)) {
        ++s_tx_enqueue_error;
        return HAL_ERROR;
    }

    primask = UART1_Lock();

    if (s_uart1_tx.ready[0] == 0U) {
        idx = 0U;
    } else if (s_uart1_tx.ready[1] == 0U) {
        idx = 1U;
    } else {
        ++s_tx_enqueue_busy;
        UART1_Unlock(primask);
        return HAL_BUSY;
    }

    memcpy(s_uart1_tx.buf[idx], data, len);
    s_uart1_tx.len[idx] = len;
    s_uart1_tx.ready[idx] = 1U;
    ++s_tx_enqueue_ok;
    s_last_tx_len = len;
    memset(s_last_tx_data, 0, sizeof(s_last_tx_data));
    memcpy(s_last_tx_data, data, (len > sizeof(s_last_tx_data)) ? sizeof(s_last_tx_data) : len);

    if (s_uart1_tx.busy == 0U) {
        status = UART1_TxStart(idx);
        if (status != HAL_OK) {
            s_uart1_tx.ready[idx] = 0U;
            s_uart1_tx.busy = 0U;
        }
    }

    UART1_Unlock(primask);
    return status;
}

uint16_t UART1_DmaRxAvailable(void)
{
    uint16_t head;
    uint16_t tail;
    uint16_t available;
    uint32_t primask;

    primask = UART1_Lock();
    head = s_uart1_rx_head;
    tail = s_uart1_rx_tail;
    UART1_Unlock(primask);

    if (head >= tail) {
        available = (uint16_t)(head - tail);
    } else {
        available = (uint16_t)(UART1_RX_RING_BUFFER_SIZE + head - tail);
    }

    return available;
}

uint16_t UART1_DmaRxRead(uint8_t *dst, uint16_t max_len)
{
    uint16_t count = 0U;
    uint32_t primask;

    if ((dst == NULL) || (max_len == 0U)) {
        return 0U;
    }

    primask = UART1_Lock();

    while ((count < max_len) && (s_uart1_rx_tail != s_uart1_rx_head)) {
        dst[count++] = s_uart1_rx_ring[s_uart1_rx_tail];
        s_uart1_rx_tail = (uint16_t)((s_uart1_rx_tail + 1U) % UART1_RX_RING_BUFFER_SIZE);
    }

    UART1_Unlock(primask);
    return count;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t next;

    if (huart != &huart1) {
        return;
    }

    ++s_tx_complete;
    s_uart1_tx.ready[s_uart1_tx.active] = 0U;
    next = (uint8_t)(s_uart1_tx.active ^ 1U);

    if (s_uart1_tx.ready[next] != 0U) {
        if (UART1_TxStart(next) != HAL_OK) {
            s_uart1_tx.busy = 0U;
        }
    } else {
        s_uart1_tx.busy = 0U;
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) {
        return;
    }

    UART1_RxCheck();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) {
        return;
    }

    UART1_RxCheck();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) {
        return;
    }

    (void)HAL_UART_AbortReceive(huart);
    if (HAL_UART_Receive_DMA(huart, s_uart1_rx_dma, UART1_RX_DMA_BUFFER_SIZE) != HAL_OK) {
        Error_Handler();
    }

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

void UART1_DmaGetDebugInfo(UART1_DmaDebugInfo_t *info)
{
    uint16_t head;
    uint16_t tail;
    uint32_t primask;

    if (info == NULL) {
        return;
    }

    primask = UART1_Lock();
    head = s_uart1_rx_head;
    tail = s_uart1_rx_tail;

    info->tx_enqueue_ok = s_tx_enqueue_ok;
    info->tx_enqueue_busy = s_tx_enqueue_busy;
    info->tx_enqueue_error = s_tx_enqueue_error;
    info->tx_start_ok = s_tx_start_ok;
    info->tx_start_fail = s_tx_start_fail;
    info->tx_complete = s_tx_complete;
    info->rx_bytes_pushed = s_rx_bytes_pushed;
    info->rx_overwrite = s_rx_overwrite;
    info->last_tx_len = s_last_tx_len;
    info->tx_busy = s_uart1_tx.busy;
    info->tx_ready_0 = s_uart1_tx.ready[0];
    info->tx_ready_1 = s_uart1_tx.ready[1];
    memcpy(info->last_tx_data, s_last_tx_data, sizeof(info->last_tx_data));
    memcpy(info->last_rx_data, s_last_rx_data, sizeof(info->last_rx_data));

    if (head >= tail) {
        info->rx_available = (uint16_t)(head - tail);
    } else {
        info->rx_available = (uint16_t)(UART1_RX_RING_BUFFER_SIZE + head - tail);
    }

    UART1_Unlock(primask);
}
