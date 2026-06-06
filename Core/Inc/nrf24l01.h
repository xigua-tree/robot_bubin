
#ifndef __24L01_H
#define __24L01_H

#include "main.h"

/******************************************************************************************/
/* NRF24L01 驱动接口 定义(硬件SPI_SCK/MISO/MISO连接引脚) */

#define NRF24L01_CE_GPIO_PORT              GPIOD
#define NRF24L01_CE_GPIO_PIN               GPIO_PIN_15
#define NRF24L01_CE_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)   /* PD口使能 */

#define NRF24L01_CSN_GPIO_PORT             GPIOB
#define NRF24L01_CSN_GPIO_PIN              GPIO_PIN_12
#define NRF24L01_CSN_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

#define NRF24L01_IRQ_GPIO_PORT             GPIOD
#define NRF24L01_IRQ_GPIO_PIN              GPIO_PIN_14
#define NRF24L01_IRQ_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)   /* PD口时钟使能 */

/******************************************************************************************/

/* 24L01操作宏 */
#define NRF24L01_CE(x)    do{ x ? \
                              HAL_GPIO_WritePin(NRF24L01_CE_GPIO_PORT, NRF24L01_CE_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(NRF24L01_CE_GPIO_PORT, NRF24L01_CE_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* 24L01模式选择信号 */

#define NRF24L01_CSN(x)   do{ x ? \
                              HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_PORT, NRF24L01_CSN_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_PORT, NRF24L01_CSN_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* 24L01片选信号 */

#define NRF24L01_IRQ      HAL_GPIO_ReadPin(NRF24L01_IRQ_GPIO_PORT, NRF24L01_IRQ_GPIO_PIN) /* IRQ主机输入信号 */


/* 24L01发送接收数据宽度定义
 * 用户可根据实际情况修改数据宽度和数据长度
 * 发送端&接收端必须保持一致, 否则将会导致通信失败!!!!
 */
#define TX_ADR_WIDTH    5       /* 5字节的地址长度 */
#define RX_ADR_WIDTH    5       /* 5字节的地址长度 */
#define TX_PLOAD_WIDTH  32      /* 32字节的用户数据宽度 */
#define RX_PLOAD_WIDTH  32      /* 32字节的用户数据宽度 */


/******************************************************************************************/
/* NRF24L01寄存器操作指令 */
#define NRF_READ_REG    0x00    /* 读配置寄存器,低5位为寄存器地址 */
#define NRF_WRITE_REG   0x20    /* 写配置寄存器,低5位为寄存器地址 */
#define RD_RX_PLOAD     0x61    /* 读RX有效数据,1~32字节 */
#define WR_TX_PLOAD     0xA0    /* 写TX有效数据,1~32字节 */
#define FLUSH_TX        0xE1    /* 清除TX FIFO寄存器.发送模式可用 */
#define FLUSH_RX        0xE2    /* 清除RX FIFO寄存器.接收模式可用 */
#define REUSE_TX_PL     0xE3    /* 重新使用上一包数据,CE为高,数据包不断重发. */
#define NOP             0xFF    /* 空操作,可以用来读状态寄存器 */

/* SPI(NRF24L01)寄存器地址 */
#define CONFIG          0x00    /* 配置寄存器地址;bit0:1接收模式,0发送模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能; */
                                /* bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能 */
#define EN_AA           0x01    /* 使能自动应答  bit0~5,对应通道0~5 */
#define EN_RXADDR       0x02    /* 接收地址允许,bit0~5,对应通道0~5 */
#define SETUP_AW        0x03    /* 设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节; */
#define SETUP_RETR      0x04    /* 设置自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us */
#define RF_CH           0x05    /* RF通道,bit6:0,工作通道频率; */
#define RF_SETUP        0x06    /* RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益 */
#define STATUS          0x07    /* 状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(高:6);bit4,达到最大重发 */
                                /* bit5:数据发送完成中断;bit6:数据接收完成中断; */
#define MAX_TX          0x10    /* 达到最大发送次数中断 */
#define TX_OK           0x20    /* TX发送完成中断 */
#define RX_OK           0x40    /* 接收到数据中断 */

#define OBSERVE_TX      0x08    /* 发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器 */
#define CD              0x09    /* 载波检测寄存器,bit0,载波检测; */
#define RX_ADDR_P0      0x0A    /* 数据通道0接收地址,最大长度5个字节,低字节在前 */
#define RX_ADDR_P1      0x0B    /* 数据通道1接收地址,最大长度5个字节,低字节在前 */
#define RX_ADDR_P2      0x0C    /* 数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]一致; */
#define RX_ADDR_P3      0x0D    /* 数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]一致; */
#define RX_ADDR_P4      0x0E    /* 数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]一致; */
#define RX_ADDR_P5      0x0F    /* 数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]一致; */
#define TX_ADDR         0x10    /* 发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等 */
#define RX_PW_P0        0x11    /* 接收数据通道0有效数据宽度(1~32字节),设置为0则非法 */
#define RX_PW_P1        0x12    /* 接收数据通道1有效数据宽度(1~32字节),设置为0则非法 */
#define RX_PW_P2        0x13    /* 接收数据通道2有效数据宽度(1~32字节),设置为0则非法 */
#define RX_PW_P3        0x14    /* 接收数据通道3有效数据宽度(1~32字节),设置为0则非法 */
#define RX_PW_P4        0x15    /* 接收数据通道4有效数据宽度(1~32字节),设置为0则非法 */
#define RX_PW_P5        0x16    /* 接收数据通道5有效数据宽度(1~32字节),设置为0则非法 */
#define NRF_FIFO_STATUS 0x17    /* FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留 */
                                /* bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环; */
/******************************************************************************************/


void nrf24l01_spi_init(void);   /* 针对NRF24L01修改SPI2设置 */
void nrf24l01_init(void);       /* 初始化 */
void nrf24l01_rx_mode(void);    /* 配置为接收模式 */
void nrf24l01_tx_mode(void);    /* 配置为发送模式 */
uint8_t nrf24l01_check(void);   /* 检测24L01是否存在 */
uint8_t nrf24l01_tx_packet(uint8_t *ptxbuf);     /* 发送一个包的数据 */
uint8_t nrf24l01_rx_packet(uint8_t *prxbuf);     /* 接收一个包的数据 */
void B_nrf24l01_switch_rx_from_A(void);
void B_nrf24l01_switch_tx_to_C(void);
void NRF_check(void);
uint8_t B_receive_from_host(uint8_t *pbuf);  /* B模块：接收主机A发来的数据包（非阻塞轮询） */
uint8_t nrf_receive_task(void);             /* B模块：NRF数据接收解析任务（主循环调用） */
void floatToTwoSint8(float num, uint8_t *high_byte, uint8_t *low_byte);
float twoSint8ToFloat(uint8_t high_byte, uint8_t low_byte);

/* 全局变量 — 供 main.c 等模块读取解析结果 */
extern uint8_t nrf_rx_buf[32];
extern uint8_t key_mode;
extern uint8_t encoderl_value;
extern uint8_t encoderr_value;
extern float rocker_lx;
extern float rocker_ly;
extern float rocker_rx;
extern float rocker_ry;


#endif
