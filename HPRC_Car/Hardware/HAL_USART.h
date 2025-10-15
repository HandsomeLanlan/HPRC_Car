#ifndef __HAL_USART_H
#define __HAL_USART_H

#include "main.h"
#include <string.h>
#include <stdint.h>
#include "HAL_OLED.h"

// 协议定义
#define PROTOCOL_HEADER_1      0xAA    // 包头字节1
#define PROTOCOL_HEADER_2      0x55    // 包头字节2
#define PROTOCOL_TAIL_1        0x0D    // 包尾字节1
#define PROTOCOL_TAIL_2        0x0A    // 包尾字节2

#define MAX_DATA_LENGTH        128     // 最大数据长度
#define PROTOCOL_OVERHEAD      5       // 协议开销(包头2+长度1+包尾2)

// 接收状态机
typedef enum {
    RX_STATE_HEADER1 = 0,
    RX_STATE_HEADER2,
    RX_STATE_LENGTH,
    RX_STATE_DATA,
    RX_STATE_TAIL1,
    RX_STATE_TAIL2
} rx_state_t;

// 数据包结构
#pragma pack(push, 1)
typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint8_t length;
    uint8_t data[MAX_DATA_LENGTH];
    uint8_t tail1;
    uint8_t tail2;
} protocol_packet_t;
#pragma pack(pop)

// 协议处理结构体
typedef struct {
    UART_HandleTypeDef *huart;
    rx_state_t rx_state;
    uint8_t rx_buffer[MAX_DATA_LENGTH + PROTOCOL_OVERHEAD];
    uint16_t rx_index;
    uint8_t expected_length;
    
    // 回调函数指针
    void (*data_received_callback)(uint8_t *data, uint8_t length);
} usart_protocol_t;

extern usart_protocol_t usart_protocol;

// 函数声明
void HAL_USART_Init(usart_protocol_t *protocol, UART_HandleTypeDef *huart);
void usart_protocol_set_callback(usart_protocol_t *protocol, void (*callback)(uint8_t *, uint8_t));
void usart_protocol_send_data(usart_protocol_t *protocol, uint8_t *data, uint8_t length);
void usart_protocol_receive_handler(usart_protocol_t *protocol);
void my_data_received_callback(uint8_t *data, uint8_t length);

#endif
