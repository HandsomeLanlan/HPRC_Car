#include "HAL_USART.h"

usart_protocol_t usart_protocol;

// 初始化协议处理结构体
void HAL_USART_Init(usart_protocol_t *protocol, UART_HandleTypeDef *huart)
{
    if (protocol == NULL || huart == NULL) {
        return;
    }
    
    protocol->huart = huart;
    protocol->rx_state = RX_STATE_HEADER1;
    protocol->rx_index = 0;
    protocol->expected_length = 0;
    protocol->data_received_callback = NULL;
    
    // 启动串口接收中断
    HAL_UART_Receive_IT(huart, &protocol->rx_buffer[0], 1);
}

// 设置数据接收回调函数
void usart_protocol_set_callback(usart_protocol_t *protocol, void (*callback)(uint8_t *, uint8_t))
{
    if (protocol != NULL) {
        protocol->data_received_callback = callback;
    }
}

// 发送数据 (封装成协议格式)
void usart_protocol_send_data(usart_protocol_t *protocol, uint8_t *data, uint8_t length)
{
    if (protocol == NULL || data == NULL || length == 0 || length > MAX_DATA_LENGTH) {
        return;
    }
    
    uint8_t tx_buffer[MAX_DATA_LENGTH + PROTOCOL_OVERHEAD];
    uint16_t index = 0;
    
    // 填充包头
    tx_buffer[index ++] = PROTOCOL_HEADER_1;
    tx_buffer[index ++] = PROTOCOL_HEADER_2;
    
    // 填充数据长度
    tx_buffer[index ++] = length;
    
    // 填充数据
    memcpy(&tx_buffer[index], data, length);
    index += length;
    
    // 填充包尾
    tx_buffer[index ++] = PROTOCOL_TAIL_1;
    tx_buffer[index ++] = PROTOCOL_TAIL_2;
    
    // 发送数据
    HAL_UART_Transmit(protocol->huart, tx_buffer, index, 1000);
}

// 接收数据处理函数 (在串口中断回调中调用)
void usart_protocol_receive_handler(usart_protocol_t *protocol)
{
    if (protocol == NULL) return;
    
    uint8_t received_byte = protocol->rx_buffer[0];
    
    switch (protocol->rx_state) {
        case RX_STATE_HEADER1:
            if (received_byte == PROTOCOL_HEADER_1) {
                protocol->rx_state = RX_STATE_HEADER2;
                protocol->rx_index = 0;
                protocol->rx_buffer[protocol->rx_index ++] = received_byte;
            }
            break;
            
        case RX_STATE_HEADER2:
            if (received_byte == PROTOCOL_HEADER_2) {
                protocol->rx_state = RX_STATE_LENGTH;
                protocol->rx_buffer[protocol->rx_index ++] = received_byte;
            } else {
                protocol->rx_state = RX_STATE_HEADER1;
            }
            break;
            
        case RX_STATE_LENGTH:
            if (received_byte <= MAX_DATA_LENGTH) {
                protocol->expected_length = received_byte;
                protocol->rx_state = RX_STATE_DATA;
                protocol->rx_buffer[protocol->rx_index ++] = received_byte;
            } else {
                protocol->rx_state = RX_STATE_HEADER1;
            }
            break;
            
        case RX_STATE_DATA:
            protocol->rx_buffer[protocol->rx_index ++] = received_byte;
            
            if (protocol->rx_index >= (3 + protocol->expected_length)) {
                protocol->rx_state = RX_STATE_TAIL1;
            }
            break;
            
        case RX_STATE_TAIL1:
            if (received_byte == PROTOCOL_TAIL_1) {
                protocol->rx_state = RX_STATE_TAIL2;
                protocol->rx_buffer[protocol->rx_index ++] = received_byte;
            } else {
                protocol->rx_state = RX_STATE_HEADER1;
            }
            break;
            
        case RX_STATE_TAIL2:
            if (received_byte == PROTOCOL_TAIL_2) {
                protocol->rx_buffer[protocol->rx_index ++] = received_byte;
                
                // 完整数据包接收完成
                if (protocol->data_received_callback != NULL) {
                    // 调用回调函数，传递数据部分 (跳过包头和长度字节)
                    protocol->data_received_callback(&protocol->rx_buffer[3], protocol->expected_length);
                }
            }
            // 无论包尾是否正确，都重新开始
            protocol->rx_state = RX_STATE_HEADER1;
            break;
            
        default:
            protocol->rx_state = RX_STATE_HEADER1;
            break;
    }
    
    // 重新启动接收中断
    HAL_UART_Receive_IT(protocol->huart, &protocol->rx_buffer[0], 1);
}

// 数据接收回调函数
void my_data_received_callback(uint8_t *data, uint8_t length)
{
    // 处理接收到的数据
    
    // 回显接收到的数据
    //usart_protocol_send_data(&usart3_protocol, data, length);
    
    OLED_ShowString(2, 1, "OK");
}

// 串口接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        usart_protocol_receive_handler(&usart_protocol);
    }
}