/*
 * SerialConsole_DMA.cpp
 *
 * UART2 DMA功能实现
 *
 * 特点：
 * 1. 循环缓冲区实现
 * 2. 双缓冲DMA接收，持续接收不丢失数据
 * 3. DMA发送，减少CPU占用
 * 4. 中断驱动，实时响应
 */

#include "SerialConsole_DMA.h"
#include <string.h>
#include <stdio.h>

#define BUFFER_SIZE      384
#define RX_BUFFER_SIZE   544
#define TX_BUFFER_SIZE   16

__attribute__((section("AHBSRAM1"), aligned(4))) static uint8_t dma_tx_buffer[TX_BUFFER_SIZE];
__attribute__((section("AHBSRAM1"), aligned(4))) static uint8_t dma_rx_buffer[RX_BUFFER_SIZE];
__attribute__((section("AHBSRAM1"), aligned(4))) static uint8_t dma_buffer[BUFFER_SIZE];

// 全局DMA控制结构
UART2_DMA_Control_t uart2_dma_ctrl;

// 内部辅助函数声明
static void UART2_DMA_StartTx(void);
static void UART2_DMA_SetupRx(void);

/******************************************************************************
 * 初始化UART2 DMA
 *****************************************************************************/
void UART2_DMA_Init(void)
{
    UART_FIFO_CFG_Type FIFOCfg;

    // ⚠️ 关键：清零所有缓冲区，避免垃圾数据
    memset(&uart2_dma_ctrl, 0, sizeof(uart2_dma_ctrl));
    memset(&dma_tx_buffer, 0, sizeof(dma_tx_buffer));
    memset(&dma_rx_buffer, 0, sizeof(dma_rx_buffer));
    memset(&dma_buffer, 0, sizeof(dma_buffer));
    uart2_dma_ctrl.tx_state = DMA_STATE_IDLE;
    uart2_dma_ctrl.rx_state = DMA_STATE_IDLE;

    // 1. 初始化GPDMA控制器
    GPDMA_Init();

    // 验证GPDMA是否启用
    if (!(LPC_GPDMA->DMACConfig & 0x01)) {
        // 如果GPDMA没启用，手动启用
        LPC_GPDMA->DMACConfig = 0x01;  // bit0=1: Enable DMA
    }

    // 2. 配置UART2 FIFO为DMA模式
    UART_FIFOConfigStructInit(&FIFOCfg);
    FIFOCfg.FIFO_DMAMode = ENABLE;           // 使能DMA模式
    FIFOCfg.FIFO_Level = UART_FIFO_TRGLEV0;  // FIFO触发级别：1字节（更灵敏）
    FIFOCfg.FIFO_ResetRxBuf = ENABLE;
    FIFOCfg.FIFO_ResetTxBuf = ENABLE;
    UART_FIFOConfig(LPC_UART2, &FIFOCfg);

    // ⚠️ 关键：禁用UART2的RX中断，避免与DMA冲突
    // DMA模式下，UART不应该产生RX中断
    LPC_UART2->IER &= ~(UART_IER_RBRINT_EN);  // 禁用RBR中断

    // 3. 配置RX DMA通道（持续循环接收）
    UART2_DMA_SetupRx();
    // NVIC_SetVector(DMA_IRQn, (uint32_t) &DMA_IRQHandler); // 不需要手动设置，向量表已定义

    // 4. 使能DMA中断
    NVIC_SetPriority(DMA_IRQn, 5);  // 中等优先级
    NVIC_EnableIRQ(DMA_IRQn);

    // 5. 启动RX DMA - 直接设置enable位
    LPC_GPDMACH1->DMACCConfig |= 0x01;  // bit0=1: Enable channel
    uart2_dma_ctrl.rx_state = DMA_STATE_BUSY;
}

/******************************************************************************
 * 反初始化UART2 DMA
 *****************************************************************************/
void UART2_DMA_DeInit(void)
{
    // 禁止DMA通道
    GPDMA_ChannelCmd(UART2_TX_DMA_CH, DISABLE);
    GPDMA_ChannelCmd(UART2_RX_DMA_CH, DISABLE);

    // 清除中断
    LPC_GPDMA->DMACIntTCClear = (1 << UART2_TX_DMA_CH) | (1 << UART2_RX_DMA_CH);
    LPC_GPDMA->DMACIntErrClr = (1 << UART2_TX_DMA_CH) | (1 << UART2_RX_DMA_CH);

    // 禁用DMA中断
    NVIC_DisableIRQ(DMA_IRQn);

    uart2_dma_ctrl.tx_state = DMA_STATE_IDLE;
    uart2_dma_ctrl.rx_state = DMA_STATE_IDLE;
}

/******************************************************************************
 * 配置RX DMA（循环接收模式）- 直接操作寄存器，确保配置正确
 *****************************************************************************/
static void UART2_DMA_SetupRx(void)
{
    // 直接操作DMA Channel 1寄存器
    LPC_GPDMACH_TypeDef *pDMA = LPC_GPDMACH1;  // Channel 1

    // 1. 禁用通道
    pDMA->DMACCConfig = 0;

    // 2. 源地址：UART2 RBR寄存器 (0x40098000 + 0x00)
    pDMA->DMACCSrcAddr = (uint32_t)(&LPC_UART2->RBR);

    // 3. 目标地址：RX缓冲区
    pDMA->DMACCDestAddr = (uint32_t)dma_buffer;

    // 4. 链表指针（不使用）
    pDMA->DMACCLLI = 0;

    // 5. 控制寄存器
    // bit[11:0] = Transfer size (字节数)
    // bit[14:12] = Source burst size (000=1)
    // bit[17:15] = Destination burst size (000=1)
    // bit[20:18] = Source transfer width (000=byte)
    // bit[23:21] = Destination transfer width (000=byte)
    // bit[26] = Source increment (0=不递增，外设地址固定)
    // bit[27] = Destination increment (1=递增)
    // bit[31] = Terminal count interrupt enable
    pDMA->DMACCControl =
        (BUFFER_SIZE & 0xFFF) |  		   	   // Transfer size
        (0 << 12) |                            // Source burst = 1
        (0 << 15) |                            // Dest burst = 1
        (0 << 18) |                            // Source width = byte
        (0 << 21) |                            // Dest width = byte
        (0 << 26) |                            // Source不递增（外设固定地址）
        (1 << 27) |                            // Dest递增
        (1 << 31);                             // Terminal count interrupt

    // 6. 配置寄存器
    // bit[0] = Enable (0=先不启用，等外部启用)
    // bit[5:1] = Source peripheral (UART2_RX = 13)
    // bit[10:6] = Dest peripheral (内存，忽略)
    // bit[13:11] = Transfer type (010=P2M，外设到内存)
    // bit[14] = Interrupt error mask
    // bit[15] = Terminal count interrupt mask
    pDMA->DMACCConfig =
        (0 << 0) |                             // Enable=0（暂不启用）
        (UART2_RX_CONN << 1) |                 // Source=13 (UART2_RX)
        (0 << 6) |                             // Dest（内存）
        (GPDMA_TRANSFERTYPE_P2M << 11) |       // Type=P2M
        (1 << 14) |                            // Error interrupt
        (1 << 15);                             // TC interrupt
}

/******************************************************************************
 * 启动TX DMA传输
 *****************************************************************************/
static void UART2_DMA_StartTx(void)
{
    if (uart2_dma_ctrl.tx_state == DMA_STATE_BUSY) {
        return;  // 已经在发送中
    }

    uint32_t tx_head = uart2_dma_ctrl.tx_head;
    uint32_t tx_tail = uart2_dma_ctrl.tx_tail;

    if (tx_head == tx_tail) {
        return;  // 没有数据要发送
    }

    // 计算要发送的数据量
    uint32_t tx_length;
    if (tx_head > tx_tail) {
        tx_length = tx_head - tx_tail;
    } else {
        // 循环缓冲区，先发送到缓冲区末尾
        tx_length = TX_BUFFER_SIZE - tx_tail;
    }

    // 配置TX DMA
    GPDMA_Channel_CFG_Type DMA_Cfg;
    DMA_Cfg.ChannelNum = UART2_TX_DMA_CH;
    DMA_Cfg.TransferSize = tx_length;
    DMA_Cfg.TransferWidth = GPDMA_WIDTH_BYTE;
    DMA_Cfg.SrcMemAddr = (uint32_t)(&dma_tx_buffer[tx_tail]);
    DMA_Cfg.DstMemAddr = 0;  // 目标为外设，不使用
    DMA_Cfg.TransferType = GPDMA_TRANSFERTYPE_M2P;  // 内存到外设
    DMA_Cfg.SrcConn = 0;  // 源是内存，不使用
    DMA_Cfg.DstConn = UART2_TX_CONN;  // UART2 TX (12)
    DMA_Cfg.DMALLI = 0;

    GPDMA_Setup(&DMA_Cfg);

    // 配置通道寄存器细节
    LPC_GPDMACH_TypeDef *pDMA = (LPC_GPDMACH_TypeDef *)LPC_GPDMACH0;  // Channel 0

    // 源地址：TX缓冲区
    pDMA->DMACCSrcAddr = (uint32_t)(&dma_tx_buffer[tx_tail]);

    // 目标地址：UART2数据寄存器
    pDMA->DMACCDestAddr = (uint32_t)&(LPC_UART2->THR);

    // 控制寄存器
    pDMA->DMACCControl =
        (tx_length & 0xFFF) |                  // 传输大小
        (GPDMA_BSIZE_1 << 12) |                // 源突发大小：1
        (GPDMA_BSIZE_1 << 15) |                // 目标突发大小：1
        (GPDMA_WIDTH_BYTE << 18) |             // 源宽度：字节
        (GPDMA_WIDTH_BYTE << 21) |             // 目标宽度：字节
        (1 << 26) |                            // 源地址递增
        (0 << 27) |                            // 目标地址不递增（外设固定地址）
        (1 << 31);                             // 使能终端计数中断

    // 配置寄存器
    pDMA->DMACCConfig =
        (1 << 0) |                             // 使能通道
        (0 << 1) |                             // 源外设（内存，不使用）
        (UART2_TX_CONN << 6) |                 // 目标外设：UART2 TX
        (GPDMA_TRANSFERTYPE_M2P << 11) |       // 传输类型：内存到外设
        (1 << 14) |                            // 错误中断使能
        (1 << 15);                             // 终端计数中断使能

    uart2_dma_ctrl.tx_state = DMA_STATE_BUSY;
}

/******************************************************************************
 * 发送数据（写入缓冲区并启动DMA）
 *****************************************************************************/
uint32_t UART2_DMA_Send(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return 0;
    }

    uint32_t sent = 0;
    uint32_t free_space = UART2_DMA_GetTxFreeSpace();

    if (length > free_space) {
        length = free_space;  // 只发送缓冲区能容纳的数据
    }

    // 写入循环缓冲区
    uint32_t tx_head = uart2_dma_ctrl.tx_head;
    while (sent < length) {
        dma_tx_buffer[tx_head] = data[sent];
        sent++;
        tx_head = (tx_head + 1) % TX_BUFFER_SIZE;
    }
    uart2_dma_ctrl.tx_head = tx_head;

    // 如果DMA空闲，启动传输
    if (uart2_dma_ctrl.tx_state == DMA_STATE_IDLE) {
        UART2_DMA_StartTx();
    }

    return sent;
}

/******************************************************************************
 * 获取TX缓冲区剩余空间
 *****************************************************************************/
uint32_t UART2_DMA_GetTxFreeSpace(void)
{
    uint32_t tx_head = uart2_dma_ctrl.tx_head;
    uint32_t tx_tail = uart2_dma_ctrl.tx_tail;

    if (tx_head >= tx_tail) {
        return TX_BUFFER_SIZE - (tx_head - tx_tail) - 1;
    } else {
        return tx_tail - tx_head - 1;
    }
}

/******************************************************************************
 * 检查TX是否忙
 *****************************************************************************/
Bool UART2_DMA_IsTxBusy(void)
{
    return (uart2_dma_ctrl.tx_state == DMA_STATE_BUSY) ? TRUE : FALSE;
}

/******************************************************************************
 * 等待TX完成
 *****************************************************************************/
void UART2_DMA_WaitTxComplete(void)
{
    while (uart2_dma_ctrl.tx_state == DMA_STATE_BUSY) {
        // 等待DMA完成
    }
}



/******************************************************************************
 * 设置TX完成回调
 *****************************************************************************/
void UART2_DMA_SetTxCallback(DMA_Callback_t callback)
{
    uart2_dma_ctrl.tx_complete_callback = callback;
}

/******************************************************************************
 * 设置RX回调
 *****************************************************************************/
void UART2_DMA_SetRxCallback(DMA_Callback_t callback)
{
    uart2_dma_ctrl.rx_callback = callback;
}

/******************************************************************************
 * DMA中断处理函数
 *****************************************************************************/
extern "C" void DMA_IRQHandler(void)
{
	uint32_t current_pos = 0;
	uint32_t last_pos = 0;
    // 检查终端计数中断
    uint32_t tc_status = LPC_GPDMA->DMACIntTCStat;
    uint32_t err_status = LPC_GPDMA->DMACIntErrStat;

    // TX DMA完成
    if (tc_status & (1 << UART2_TX_DMA_CH)) {
        // 清除中断标志
        LPC_GPDMA->DMACIntTCClear = (1 << UART2_TX_DMA_CH);

        // 更新tail指针
        uint32_t tx_tail = uart2_dma_ctrl.tx_tail;
        LPC_GPDMACH_TypeDef *pDMA = (LPC_GPDMACH_TypeDef *)LPC_GPDMACH0;
        uint32_t transferred = pDMA->DMACCControl & 0xFFF;

        tx_tail = (tx_tail + transferred) % TX_BUFFER_SIZE;
        uart2_dma_ctrl.tx_tail = tx_tail;

        // 标记为空闲
        uart2_dma_ctrl.tx_state = DMA_STATE_IDLE;

        // 如果还有数据，继续发送
        if (uart2_dma_ctrl.tx_head != tx_tail) {
            UART2_DMA_StartTx();
        } else if (uart2_dma_ctrl.tx_complete_callback) {
            uart2_dma_ctrl.tx_complete_callback();
        }
    }

    // RX DMA完成（缓冲区满，重新启动）
    if (tc_status & (1 << UART2_RX_DMA_CH)) {
        // 清除中断标志
        LPC_GPDMA->DMACIntTCClear = (1 << UART2_RX_DMA_CH);
        
        current_pos = get_dma_rx_current_position();
		last_pos = uart2_dma_ctrl.last_processed_pos;
		uint32_t len = BUFFER_SIZE - last_pos;
//		if (current_pos != last_pos) 
		{
			CopyDataToRingBuffer(&dma_buffer[last_pos], len);
			uart2_dma_ctrl.total_received += len;
		}
		uart2_dma_ctrl.last_processed_pos = 0;

        // 重新启动RX DMA(循环接收)
        UART2_DMA_SetupRx();
        GPDMA_ChannelCmd(UART2_RX_DMA_CH, ENABLE);

        // 调用回调
        if (uart2_dma_ctrl.rx_callback) {
            uart2_dma_ctrl.rx_callback();
        }
    }

    // 错误处理
    if (err_status & (1 << UART2_TX_DMA_CH)) {
        LPC_GPDMA->DMACIntErrClr = (1 << UART2_TX_DMA_CH);
        uart2_dma_ctrl.tx_state = DMA_STATE_ERROR;
    }

    if (err_status & (1 << UART2_RX_DMA_CH)) {
        LPC_GPDMA->DMACIntErrClr = (1 << UART2_RX_DMA_CH);
        // RX错误，重新启动
        UART2_DMA_SetupRx();
        GPDMA_ChannelCmd(UART2_RX_DMA_CH, ENABLE);
    }
}

uint32_t get_dma_rx_current_position(void)
{
    LPC_GPDMACH_TypeDef *pDMA = (LPC_GPDMACH_TypeDef *)LPC_GPDMACH1;
    
    // 第一次读取
    uint32_t addr1 = pDMA->DMACCDestAddr;
    
    // 短暂延迟，确保DMA寄存器稳定
    for(volatile int i = 0; i < 10; i++);  // 小延迟
    
    // 第二次读取
    uint32_t addr2 = pDMA->DMACCDestAddr;
    
    // 如果两次读取一致，说明数据稳定
    if (addr1 == addr2) {
        return addr1 - (uint32_t)dma_buffer;
    } else {
        // 如果不一致，返回较新的值或进行错误处理
        return addr2 - (uint32_t)dma_buffer;
    }
}

void check_dma_progress_periodically(void)
{
	uint32_t current_pos = 0;
	uint32_t last_pos = 0;
	uint32_t len = 0;
	
	
	NVIC_DisableIRQ(DMA_IRQn);
	current_pos = get_dma_rx_current_position();
	last_pos = uart2_dma_ctrl.last_processed_pos;
	len = current_pos - last_pos;
	if (current_pos != last_pos) {
		CopyDataToRingBuffer(&dma_buffer[last_pos], len);
		uart2_dma_ctrl.total_received += len;
		uart2_dma_ctrl.last_processed_pos = current_pos;
	}
	
	NVIC_EnableIRQ(DMA_IRQn);
}

uint32_t GetRxAvailable(void)
{
    uint32_t head = uart2_dma_ctrl.rx_head;
    uint32_t tail = uart2_dma_ctrl.rx_tail;
    
    if (head >= tail) {
        return head - tail;
    } else {
        return (RX_BUFFER_SIZE - tail) + head;
    }
}

uint32_t CopyDataToRingBuffer(uint8_t *src_data, uint32_t src_len)
{
    uint32_t free_space = RX_BUFFER_SIZE - GetRxAvailable();
    uint32_t copy_len = (src_len < free_space) ? src_len : free_space;
    
    if (copy_len == 0) return 0;
    
    uint32_t head = uart2_dma_ctrl.rx_head;
    
    uint32_t space_to_end = RX_BUFFER_SIZE - head;
    
    if (copy_len <= space_to_end) {
        memcpy(&dma_rx_buffer[head], src_data, copy_len);
        uart2_dma_ctrl.rx_head = (head + copy_len) % RX_BUFFER_SIZE;
    } else {
        memcpy(&dma_rx_buffer[head], src_data, space_to_end);
        memcpy(dma_rx_buffer, &src_data[space_to_end], copy_len - space_to_end);
        uart2_dma_ctrl.rx_head = copy_len - space_to_end;
    }
    
    return copy_len;
}

uint32_t UART2_ReadData(uint8_t *data, uint32_t max_len)
{
	uint32_t available = 0;
	uint32_t read_len = 0;
	
    available = GetRxAvailable();
    read_len = (available < max_len) ? available : max_len;
    
    if (read_len == 0) return 0;
    
    uint32_t tail = uart2_dma_ctrl.rx_tail;
    
    uint32_t data_to_end = RX_BUFFER_SIZE - tail;
    
    if (read_len <= data_to_end) {
        memcpy(data, &dma_rx_buffer[tail], read_len);
        uart2_dma_ctrl.rx_tail = (tail + read_len) % RX_BUFFER_SIZE;
    } else {
        memcpy(data, &dma_rx_buffer[tail], data_to_end);
        memcpy(&data[data_to_end], dma_rx_buffer, read_len - data_to_end);
        uart2_dma_ctrl.rx_tail = read_len - data_to_end;
    }
    
    return read_len;
}