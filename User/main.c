/**
 * @author Dmitry Proshutinsky (sodaspace@yandex.ru)
 * @brief I2C to SPI/GPIO interface converter
 * @version 0.02
 *
 * @todo:
 * - second i2c address or mode pin
 * - settings: CPOL, CPHA, frequency
 * - several CSs and buffers (?)
 * - get, reset counters
 *
 * @note connection diagram:
 *           SWIO  o  o
 *       ©°©¤©¤©¤©¤©¤©¤©Ø©¤©¤©Ø©¤©¤©Ø©¤©¤©¤©¤©¤©¤©´
 *       ©¦   SWIO  V  G      ©¦
 *  SCK ©¤©È PC5           PC4 ©À©¤ ?
 * MOSI ©¤©È PC6           PC3 ©À©¤ ?
 * MISO ©¤©È PC7           PC2 ©À©¤ SCL
 *  NSS ©¤©È PD0           PC1 ©À©¤ SDA
 * SWIO ©¤©È PD1           PC0 ©À©¤ BUSY
 *    ? ©¤©È PD2           PA2 ©À©¤ ?
 *    ? ©¤©È PD3           PA1 ©À©¤ ?
 *    ? ©¤©È PD4           PD6 ©À©¤ RX
 * NRST ©¤©È PD7           PD5 ©À©¤ TX
 *    o ©¤©È V               V ©À©¤ o
 *    o ©¤©È G               G ©À©¤ o
 *       ©¦      VT1772       ©¦
 *       ©¦   CH32V003F4P6    ©¦
 *       ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
 */

#include <ch32v00x.h>
#include "debug.h"

#define I2C_ADDRESS 0x51

#define MAP_SIZE 1024

uint8_t map[MAP_SIZE];
volatile size_t len = 0;

volatile size_t rxc = 0;
volatile size_t txc = 0;


/* clock initialization *******************************************************/
static void init_clock(void)
{
    RCC_PLLConfig(RCC_PLLSource_HSI_MUL2);
    RCC_PLLCmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != SET);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    SystemCoreClockUpdate();
}

/* i2c initialization *********************************************************/
static void init_i2c(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef I2C_InitSturcture = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // PC1 - SDA
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // PC2 - SCL
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // I2C
    I2C_InitSturcture.I2C_ClockSpeed = 400000;
    I2C_InitSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitSturcture.I2C_OwnAddress1 = I2C_ADDRESS << 1;
    I2C_InitSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitSturcture);

    I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    I2C_Cmd(I2C1, ENABLE);
}

/* gpio busy initialization ***************************************************/
/**
 * @note gpio busy states:
 * - HIGH - busy
 * - LOW - ready
 */
static void init_gpio_busy(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // PC0 - BUSY
    GPIO_ResetBits(GPIOC, GPIO_Pin_0);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* spi initialization *********************************************************/
static void init_spi(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef SPI_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_SPI1, ENABLE);

    // PD0 - NSS
    GPIO_SetBits(GPIOD, GPIO_Pin_0);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // PC5 - SCK
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // PC6 - MOSI
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // PC7 - MISO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // SPI
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    // DMA
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

    SPI_Cmd(SPI1, ENABLE);
}

/* dma spi tx initialization **************************************************/
static void init_dma_spi_tx(void)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &SPI1->DATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) map;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
}

/* dma spi rx initialization **************************************************/
static void init_dma_spi_rx(void)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &SPI1->DATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t) map;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TE, ENABLE);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

/* busy active ****************************************************************/
inline static void busy_active(void)
{
    GPIO_SetBits(GPIOC, GPIO_Pin_0);
}

/* busy inactive ****************************************************************/
inline static void busy_inactive(void)
{
    GPIO_ResetBits(GPIOC, GPIO_Pin_0);
}

/* spi cs active **************************************************************/
inline static void spi_cs_active(void)
{
    GPIO_ResetBits(GPIOD, GPIO_Pin_0);
}

/* spi cs inactive ************************************************************/
inline static void spi_cs_inactive(void)
{
    GPIO_SetBits(GPIOD, GPIO_Pin_0);
}

/* start spi dma transfer *****************************************************/
static void spi_dma_transfer(void)
{
    DMA_Cmd(DMA1_Channel2, DISABLE); // RX
    DMA_Cmd(DMA1_Channel3, DISABLE); // TX

    DMA_SetCurrDataCounter(DMA1_Channel2, len); // RX
    DMA_SetCurrDataCounter(DMA1_Channel3, len); // TX

    DMA_Cmd(DMA1_Channel2, ENABLE); // RX
    DMA_Cmd(DMA1_Channel3, ENABLE); // TX
}

/* i2c irq ********************************************************************/
void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void)
{
    uint32_t status = I2C_GetLastEvent(I2C1);
    uint8_t tmp = 0x00;

    // Address match
    if (status & I2C_STAR1_ADDR)
    {
        len = 0;
    }

    // Byte received
    if (status & I2C_STAR1_RXNE)
    {
        tmp = I2C_ReceiveData(I2C1);
        rxc++;

        if (len < sizeof(map))
        {
            map[len] = tmp;
            len++;
        }
    }

    // Byte requested
    if (status & I2C_STAR1_TXE)
    {
        // Skip first
        if (len)
            txc++;

        if (len < sizeof(map))
        {
            tmp = map[len];
            len++;
        }

        I2C_SendData(I2C1, tmp);
    }

    // Stop condition
    if (status & I2C_STAR1_STOPF)
    {
        busy_active();

        I2C_AcknowledgeConfig(I2C1, DISABLE);
        I2C_Cmd(I2C1, ENABLE); // Not nesessary

        spi_cs_active();
        spi_dma_transfer();
    }
}

/* i2c err irq ****************************************************************/
void I2C1_ER_IRQHandler(void) __attribute__((interrupt));
void I2C1_ER_IRQHandler(void)
{
    I2C_ClearFlag(I2C1, I2C_FLAG_AF);
    I2C_ClearFlag(I2C1, I2C_FLAG_ARLO);
    I2C_ClearFlag(I2C1, I2C_FLAG_BERR);
}

/* dma spi rx irq *************************************************************/
void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt));
void DMA1_Channel2_IRQHandler(void)
{
    DMA_ClearFlag(DMA1_FLAG_GL2);
    DMA_ClearFlag(DMA1_FLAG_TC2);
    DMA_ClearFlag(DMA1_FLAG_TE2);

    spi_cs_inactive();
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    busy_inactive();
}

/* main ***********************************************************************/
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    init_clock();
    init_i2c();
    init_spi();
    init_dma_spi_tx();
    init_dma_spi_rx();
    init_gpio_busy();

    Delay_Init();
    USART_Printf_Init(115200);

    printf("SystemClk: %d\r\n", SystemCoreClock);
    printf("ChipID: %08x\r\n", DBGMCU_GetCHIPID());

    for (;;)
    {
        Delay_Ms(1000);
        printf("rxc: %08u, txc: %08u\r\n", rxc, txc);
    }
}
