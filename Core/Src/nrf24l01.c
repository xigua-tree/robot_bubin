#include "spi.h"
#include "nrf24l01.h"

#define ADDR_AB_LEN    5   // 与宏定义TX_ADR_WIDTH=5一致
#define ADDR_BC_LEN    5   // 与宏定义TX_ADR_WIDTH=5一致
#define DATA_WIDTH     32  // 与宏定义TX_PLOAD_WIDTH=32一致

// A与B 通信参数(A与B模块共用)
uint8_t ADDR_AB[ADDR_AB_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55};
#define CH_AB          30  // A-B通道频率

// B与C 通信参数(B与C模块共用)
uint8_t ADDR_BC[ADDR_BC_LEN] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
#define CH_BC          40  // B-C通道频率

/* SPI句柄 — 来自spi.c */
extern SPI_HandleTypeDef hspi2;

const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01};    /* 接收地址 */
const uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01};    /* 发送地址 */

/* ---- 内部函数前向声明 ---- */
static uint8_t nrf24l01_read_buf(uint8_t reg, uint8_t *pbuf, uint8_t len);
static uint8_t nrf24l01_write_buf(uint8_t reg, uint8_t *pbuf, uint8_t len);

uint8_t nrf_rx_buf[32];

/* ---- 移植层：SPI 底层操作 (用 HAL 实现) ---- */

/**
 * @brief       SPI读写一个字节
 * @param       tx_data : 发送的数据
 * @retval      接收到的数据
 */
static uint8_t spi2_read_write_byte(uint8_t tx_data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi2, &tx_data, &rx_data, 1, 100);
    return rx_data;
}

/**
 * @brief       设置SPI2速度(修改分频系数)
 * @param       prescaler : SPI分频系数(SPI_BAUDRATEPRESCALER_xx)
 * @retval      无
 */
static void spi2_set_speed(uint32_t prescaler)
{
    hspi2.Init.BaudRatePrescaler = prescaler;
    HAL_SPI_Init(&hspi2);
}
/* ---- 移植层结束 ---- */


/**
 * @brief       针对NRF24L01修改SPI1设置
 * @param       无
 * @retval      无
 */
void nrf24l01_spi_init(void)
{
    /* NRF24L01 要求 SPI Mode 0: CPOL=0, CPHA=0 (第1个边沿采样).
     * 项目的 MX_SPI2_Init() 已配置 CPOL=LOW, CPHA=1EDGE,
     * 只需确保速度不超过 10MHz 即可. 使用8分频: 42MHz/8 = 5.25MHz */
    spi2_set_speed(SPI_BAUDRATEPRESCALER_8);
}

/**
 * @brief       初始化24L01的IO口
 *   @note      将SPI1模式的改成SCK空闲低电平,即SPI 模式0
 * @param       无
 * @retval      无
 */
void nrf24l01_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    NRF24L01_CE_GPIO_CLK_ENABLE();  /* CE口时钟使能 */
    NRF24L01_CSN_GPIO_CLK_ENABLE(); /* CSN口时钟使能 */
    NRF24L01_IRQ_GPIO_CLK_ENABLE(); /* IRQ口时钟使能 */

    gpio_init_struct.Pin = NRF24L01_CE_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;             /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                     /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;           /* 高速 */
    HAL_GPIO_Init(NRF24L01_CE_GPIO_PORT, &gpio_init_struct); /* 初始化CE引脚 */

    gpio_init_struct.Pin = NRF24L01_CSN_GPIO_PIN;
    HAL_GPIO_Init(NRF24L01_CSN_GPIO_PORT, &gpio_init_struct);/* 初始化CSN引脚 */

    gpio_init_struct.Pin = NRF24L01_IRQ_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                 /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                     /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;           /* 高速 */
    HAL_GPIO_Init(NRF24L01_IRQ_GPIO_PORT, &gpio_init_struct);/* 初始化IRQ引脚 */

    /* SPI2 已在 MX_SPI2_Init() 中初始化, 这里只需针对 NRF 调整设置 */
    nrf24l01_spi_init();        /* 针对NRF特点修改SPI设置 */
    NRF24L01_CE(0);             /* 使能24L01 */
    NRF24L01_CSN(1);            /* SPI片选取消 */
}

/**
 * @brief       检测24L01是否存在
 * @param       无
 * @retval      0, 成功; 1, 失败;
 */
uint8_t nrf24l01_check(void)
{
    uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
    uint8_t i;

    spi2_set_speed(SPI_BAUDRATEPRESCALER_32);                /* spi速度为7.5Mhz(24L01的最大SPI时钟为10Mhz) */
    nrf24l01_write_buf(NRF_WRITE_REG + TX_ADDR, buf, 5);  /* 写入5个字节的地址. */
    nrf24l01_read_buf(TX_ADDR, buf, 5);                   /* 读出写入的地址 */

    for (i = 0; i < 5; i++)
    {
        if (buf[i] != 0XA5) break;
    }

    if (i != 5) return 1;   /* 检测24L01错误 */

    return 0;               /* 检测到24L01 */
}

/**
 * @brief       NRF24L01写寄存器
 * @param       reg   : 寄存器地址
 * @param       value : 写入寄存器的值
 * @retval      状态寄存器值
 */
static uint8_t nrf24l01_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t status;

    NRF24L01_CSN(0);                    /* 使能SPI传输 */
    status = spi2_read_write_byte(reg); /* 发送寄存器号 */
    spi2_read_write_byte(value);        /* 写入寄存器的值 */
    NRF24L01_CSN(1);                    /* 禁止SPI传输 */

    return status;                      /* 返回状态值 */
}

/**
 * @brief       NRF24L01读寄存器
 * @param       reg   : 寄存器地址
 * @retval      读取到的寄存器值;
 */
static uint8_t nrf24l01_read_reg(uint8_t reg)
{
    uint8_t reg_val;

    NRF24L01_CSN(0);            /* 使能SPI传输 */
    spi2_read_write_byte(reg);  /* 发送寄存器号 */
    reg_val = spi2_read_write_byte(0XFF);   /* 读取寄存器内容 */
    NRF24L01_CSN(1);            /* 禁止SPI传输 */

    return reg_val;             /* 返回状态值 */
}

/**
 * @brief       在指定位置读出指定长度的数据
 * @param       reg   : 寄存器地址
 * @param       pbuf  : 数据指针
 * @param       len   : 数据长度
 * @retval      状态寄存器值
 */
static uint8_t nrf24l01_read_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
    uint8_t status, i;

    NRF24L01_CSN(0);                            /* 使能SPI传输 */
    status = spi2_read_write_byte(reg);         /* 发送寄存器值(位置),并读取状态值 */

    for (i = 0; i < len; i++)
    {
        pbuf[i] = spi2_read_write_byte(0XFF);   /* 读取数据 */
    }

    NRF24L01_CSN(1);    /* 关闭SPI传输 */

    return status;      /* 返回读到的状态值 */
}

/**
 * @brief       在指定位置写指定长度的数据
 * @param       reg   : 寄存器地址
 * @param       pbuf  : 数据指针
 * @param       len   : 数据长度
 * @retval      状态寄存器值
 */
static uint8_t nrf24l01_write_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
    uint8_t status, i;

    NRF24L01_CSN(0);    /* 使能SPI传输 */
    status = spi2_read_write_byte(reg);/* 发送寄存器值(位置),并读取状态值 */

    for (i = 0; i < len; i++)
    {
        spi2_read_write_byte(*pbuf++); /* 写入数据 */
    }

    NRF24L01_CSN(1);    /* 关闭SPI传输 */

    return status;      /* 返回读到的状态值 */
}

/**
 * @brief       启动NRF24L01发送一次数据(数据长度 = TX_PLOAD_WIDTH)
 * @param       ptxbuf : 待发送数据首地址
 * @retval      发送完成状态
 *   @arg       0    : 发送成功
 *   @arg       1    : 达到最大次数,失败
 *   @arg       0XFF : 发送中
 */
uint8_t nrf24l01_tx_packet(uint8_t *ptxbuf)
{
    uint8_t sta;
    uint8_t rval = 0XFF;
    uint32_t timeout;

    /* 发送前清除所有旧中断标志，确保 IRQ 从高电平开始 */
    sta = nrf24l01_read_reg(STATUS);
    nrf24l01_write_reg(NRF_WRITE_REG + STATUS, sta);

    NRF24L01_CE(0);
    nrf24l01_write_buf(WR_TX_PLOAD, ptxbuf, TX_PLOAD_WIDTH);    /* 写数据到TX BUF  TX_PLOAD_WIDTH个字节 */
    NRF24L01_CE(1);                     /* 启动发送 */

    /* 等待发送完成，超时 100ms（正常 TX_DS<1ms，MAX_RT<5ms） */
    timeout = HAL_GetTick() + 100;
    while (NRF24L01_IRQ != 0) {
        if (HAL_GetTick() >= timeout) {
            NRF24L01_CE(0);                             /* 停止发送 */
            sta = nrf24l01_read_reg(STATUS);            /* 读并清除中断标志 */
            nrf24l01_write_reg(NRF_WRITE_REG + STATUS, sta);
            nrf24l01_write_reg(FLUSH_TX, 0xff);         /* 清空TX FIFO */
            return 0xFF;
        }
    }

    sta = nrf24l01_read_reg(STATUS);    /* 读取状态寄存器的值 */
    nrf24l01_write_reg(NRF_WRITE_REG + STATUS, sta);    /* 清除TX_DS或MAX_RT中断标志 */

    if (sta & MAX_TX)   /* 达到最大重发次数 */
    {
        nrf24l01_write_reg(FLUSH_TX, 0xff);             /* 清除TX FIFO寄存器 */
        rval = 1;
    }

    if (sta & TX_OK)    /* 发送完成 */
    {
        rval = 0;       /* 标记发送成功 */
    }

    return rval;        /* 返回结果 */
}

/**
 * @brief       启动NRF24L01接收一次数据(数据长度 = RX_PLOAD_WIDTH)
 * @param       prxbuf : 接收数据缓冲区首地址
 * @retval      接收完成状态
 *   @arg       0 : 接收成功
 *   @arg       1 : 失败
 */
uint8_t nrf24l01_rx_packet(uint8_t *prxbuf)
{
    uint8_t sta;
    uint8_t rval = 1;

    sta = nrf24l01_read_reg(STATUS);                            /* 读取状态寄存器的值 */
    nrf24l01_write_reg(NRF_WRITE_REG + STATUS, sta);            /* 清除TX_DS或MAX_RT中断标志 */

    if (sta & RX_OK)    /* 接收到数据 */
    {
        nrf24l01_read_buf(RD_RX_PLOAD, prxbuf, RX_PLOAD_WIDTH); /* 读取数据 */
        nrf24l01_write_reg(FLUSH_RX, 0xff);                     /* 清除RX FIFO寄存器 */
        rval = 0;       /* 标记接收成功 */
    }

    return rval;        /* 返回结果 */
}

/**
 * @brief       NRF24L01进入接收模式
 *   @note      设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
 *              当CE变高后,即进入RX模式,并可以接收数据了
 * @param       无
 * @retval      无
 */
void nrf24l01_rx_mode(void)
{
    NRF24L01_CE(0);
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH);    /* 写RX节点地址 */

    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);        /* 使能通道0的自动应答 */
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    /* 使能通道0的接收地址 */
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, 40);          /* 设置RF通信频率 */
    nrf24l01_write_reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);   /* 选择通道0的有效数据宽度 */
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     /* 设置TX发射参数,0db增益,2Mbps,低噪声放大器开启 */
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0f);       /* 配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 */
    NRF24L01_CE(1); /* CE为高,进入接收模式 */
}

/**
 * @brief       NRF24L01进入发送模式
 *   @note      设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送参数,选择RF频道,波特率和
 *              LNA HCURR,PWR_UP,CRC使能
 *              当CE变高后,即进入TX模式,并可以发送数据了, CE为高大于10us,则启动发送.
 * @param       无
 * @retval      无
 */
void nrf24l01_tx_mode(void)
{
    NRF24L01_CE(0);
    nrf24l01_write_buf(NRF_WRITE_REG + TX_ADDR, (uint8_t *)TX_ADDRESS, TX_ADR_WIDTH);       /* 写TX节点地址 */
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH);    /* 设置RX节点地址,主要为了使能ACK */

    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);        /* 使能通道0的自动应答 */
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    /* 使能通道0的接收地址 */
    nrf24l01_write_reg(NRF_WRITE_REG + SETUP_RETR, 0x1a);   /* 设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次 */
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, 40);          /* 设置RF通道为40 */
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     /* 设置TX发射参数,0db增益,2Mbps,低噪声放大器开启 */
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0e);       /* 配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发送模式,开启所有中断 */
    NRF24L01_CE(1); /* CE为高,10us后启动发送 */
}


/**
 * @brief  B模块第1步：切换为"接收A数据"模式(A-B通信)
 * @note   基于库函数nrf24l01_rx_mode()修改，使用ADDR_AB和CH_AB
 */
void B_nrf24l01_switch_rx_from_A(void)
{
    NRF24L01_CE(0);  // 拉低CE，进入配置模式(停止当前模式)

    // 1. 写RX_ADDR_P0为A与B的地址(ADDR_AB)
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_AB, ADDR_AB_LEN);

    // 2. 库函数默认设置，保持不变，仅修改频道和地址
    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);        // 使能通道0自动应答(对A回复ACK)
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // 使能通道0接收地址
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, CH_AB);       // 切换到A-B频道 30
    nrf24l01_write_reg(NRF_WRITE_REG + RX_PW_P0, DATA_WIDTH); // 数据长度32字节
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db增益，2Mbps，低噪声放大器开启
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0f);       // 接收模式，PWR_UP，EN_CRC

    NRF24L01_CE(1);  // 拉高CE，进入接收状态
//    HAL_Delay(10);   // 延时保证模式切换完成，避免模式冲突
}

/**
 * @brief  B模块第2步：切换为"发送数据给C"模式(B-C通信)
 * @note   基于库函数nrf24l01_tx_mode()修改，使用ADDR_BC和CH_BC
 */
void B_nrf24l01_switch_tx_to_C(void)
{
    NRF24L01_CE(0);  // 拉低CE，进入配置模式(停止当前模式)

    // 1. 写TX地址为B与C的地址(ADDR_BC)
    nrf24l01_write_buf(NRF_WRITE_REG + TX_ADDR, ADDR_BC, ADDR_BC_LEN);
    // 2. 写RX_ADDR_P0(用于ACK)，必须与C的TX地址一致(即ADDR_BC)
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_BC, ADDR_BC_LEN);

    // 3. 库函数默认设置，保持不变，仅修改频道和地址
    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);        // 使能通道0自动应答
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // 使能通道0接收地址
    nrf24l01_write_reg(NRF_WRITE_REG + SETUP_RETR, 0x1a);   // 自动重发10次，间隔500us+86us
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, CH_BC);       // 切换到B-C频道 40
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db增益，2Mbps，低噪声放大器开启
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0e);       // 发送模式，PWR_UP，EN_CRC

//    HAL_Delay(10);   // 延时保证模式切换完成，注意:CE置高在nrf24l01_tx_packet里
}

void NRF_check(void)
{
    while (nrf24l01_check()){
     HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_5);
        HAL_Delay(1000);
    } /* 检测NRF24L01是否存在 */

    B_nrf24l01_switch_rx_from_A();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
 * @brief  B模块：接收主机A发来的数据包（非阻塞轮询, 适合主循环调用）
 * @note   调用前需确保已切换到接收A模式(B_nrf24l01_switch_rx_from_A)
 * @param  pbuf : 接收数据缓冲区(至少 RX_PLOAD_WIDTH=32 字节)
 * @retval 0    : 收到新数据包, 数据已写入 pbuf
 *         1    : 当前无新数据
 */
uint8_t B_receive_from_host(uint8_t *pbuf)
{
    uint8_t sta;

    /* 读取 STATUS, 检查 RX_DR 标志 (bit6) */
    sta = nrf24l01_read_reg(STATUS);

    if (sta & RX_OK)    /* 有数据收到 */
    {
        /* 清除 RX_DR 中断标志 */
        nrf24l01_write_reg(NRF_WRITE_REG + STATUS, sta);

        /* 读出有效数据 */
        nrf24l01_read_buf(RD_RX_PLOAD, pbuf, RX_PLOAD_WIDTH);

        /* 清空 RX FIFO, 准备接收下一包 */
        nrf24l01_write_reg(FLUSH_RX, 0xff);

        return 0;       /* 收到数据 */
    }

    return 1;           /* 无新数据 */
}

/**
 * @brief  浮点数转两个uint8_t (精度0.01, 范围-327.68~327.67)
 */
void floatToTwoSint8(float num, uint8_t *high_byte, uint8_t *low_byte)
{
    if (num > 327.67f) num = 327.67f;
    if (num < -327.68f) num = -327.68f;

    int16_t scaled = (int16_t)(round(num * 100.0f));

    *high_byte = (scaled >> 8) & 0xFF;
    *low_byte = scaled & 0xFF;
}

/**
 * @brief  两个uint8_t还原为浮点数
 */
float twoSint8ToFloat(uint8_t high_byte, uint8_t low_byte)
{
    int16_t scaled = (int16_t)((high_byte << 8) | low_byte);
    return (float)scaled / 100.0f;
}

uint8_t key_mode = 0;
uint8_t encoderl_value = 0, encoderr_value = 0;
float rocker_lx, rocker_ly, rocker_rx, rocker_ry;

/**
 * @brief  B模块：NRF 数据接收任务（主循环轮询调用）
 * @note   一包数据内完成帧头校验 + 数据解析
 * @retval 0: 收到有效数据包并解析完成
 *         1: 当前无新数据或帧校验失败
 */
uint8_t nrf_receive_task(void)
{
    if (B_receive_from_host(nrf_rx_buf) != 0) {
        return 1;   /* 无新数据 */
    }

    /* 帧头尾校验：0x55 开头, 0xFF 结尾(第20字节) */
    if (nrf_rx_buf[0] != 0x55 || nrf_rx_buf[19] != 0xFF) {
        return 1;   /* 帧校验失败 */
    }

    /* 解析数据 */
    key_mode       = nrf_rx_buf[1];
    encoderl_value = nrf_rx_buf[2];
    encoderr_value = nrf_rx_buf[3];
    rocker_lx      = twoSint8ToFloat(nrf_rx_buf[4],  nrf_rx_buf[5]);
    rocker_ly      = twoSint8ToFloat(nrf_rx_buf[6],  nrf_rx_buf[7]);
    rocker_rx      = twoSint8ToFloat(nrf_rx_buf[8],  nrf_rx_buf[9]);
    rocker_ry      = twoSint8ToFloat(nrf_rx_buf[10], nrf_rx_buf[11]);

    return 0;   /* 解析成功 */
}