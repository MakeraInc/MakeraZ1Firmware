/*
 * SerialConsole_DMA.h
 *
 * UART2 DMA功能实现
 * 使用LPC1768的GPDMA控制器
 *
 * DMA配置：
 * - UART2 TX: DMA Channel 0, Connection 12
 * - UART2 RX: DMA Channel 1, Connection 13
 */

#ifndef SERIALCONSOLE_DMA_H
#define SERIALCONSOLE_DMA_H

#include "lpc17xx.h"
#include "lpc_types.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// DMA配置常量
#define UART2_TX_DMA_CH      0                    // TX使用DMA通道0
#define UART2_RX_DMA_CH      1                    // RX使用DMA通道1
#define UART2_TX_CONN        GPDMA_CONN_UART2_Tx  // 12
#define UART2_RX_CONN        GPDMA_CONN_UART2_Rx  // 13



// DMA状态
typedef enum {
    DMA_STATE_IDLE = 0,
    DMA_STATE_BUSY,
    DMA_STATE_ERROR
} DMA_STATE;

// DMA回调函数类型
typedef void (*DMA_Callback_t)(void);

// DMA控制结构
typedef struct {
    // TX相关
    volatile uint32_t tx_head;          // 写入位置
    volatile uint32_t tx_tail;          // DMA读取位置
    volatile DMA_STATE tx_state;
    DMA_Callback_t tx_complete_callback;

    // RX相关
    volatile uint32_t rx_head;          // DMA写入位置
    volatile uint32_t rx_tail;          // 读取位置
    volatile uint32_t last_processed_pos;
    volatile DMA_STATE rx_state;
    
    volatile uint32_t total_received;
    DMA_Callback_t rx_callback;
} UART2_DMA_Control_t;

// 全局DMA控制结构
extern UART2_DMA_Control_t uart2_dma_ctrl;

// 初始化和配置函数
void UART2_DMA_Init(void);
void UART2_DMA_DeInit(void);

// TX函数
uint32_t UART2_DMA_Send(const uint8_t *data, uint32_t length);
uint32_t UART2_DMA_GetTxFreeSpace(void);
Bool UART2_DMA_IsTxBusy(void);
void UART2_DMA_WaitTxComplete(void);

// RX函数
uint32_t UART2_DMA_Receive(uint8_t *data, uint32_t max_length);
uint32_t get_dma_rx_current_position(void);
void check_dma_progress_periodically(void);

// 回调设置
void UART2_DMA_SetTxCallback(DMA_Callback_t callback);
void UART2_DMA_SetRxCallback(DMA_Callback_t callback);

// 中断处理函数
void DMA_IRQHandler(void);

uint32_t GetRxAvailable(void);
uint32_t CopyDataToRingBuffer(uint8_t *src_data, uint32_t src_len);
uint32_t UART2_ReadData(uint8_t *data, uint32_t max_len);

#ifdef __cplusplus
}
#endif

#endif // SERIALCONSOLE_DMA_H

