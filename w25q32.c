/*
 * w25q32.c - Minimal Winbond W25Q32 SPI NOR flash driver (STM32 HAL, polling)
 * SPI settings: Mode 0 (CPOL=Low, CPHA=1Edge), 8-bit, MSB first, software CS.
 */
#include "w25q32.h"

#define CMD_WRITE_ENABLE   0x06
#define CMD_READ_SR1       0x05
#define CMD_READ_DATA      0x03
#define CMD_PAGE_PROGRAM   0x02
#define CMD_SECTOR_ERASE   0x20
#define CMD_BLOCK_ERASE64  0xD8
#define CMD_CHIP_ERASE     0xC7
#define CMD_JEDEC_ID       0x9F
#define CMD_RELEASE_PD     0xAB
#define CMD_ENABLE_RESET   0x66
#define CMD_RESET          0x99

#define SR1_BUSY           0x01

#define SPI_TIMEOUT_MS     100u
#define TPP_TIMEOUT_MS     10u      /* page program, max 3 ms   */
#define TSE_TIMEOUT_MS     500u     /* 4K sector erase, max 400 ms */
#define TBE_TIMEOUT_MS     3000u    /* 64K block erase, max 2 s  */
#define TCE_TIMEOUT_MS     100000u  /* chip erase, max 50 s      */

static SPI_HandleTypeDef *s_hspi;
static GPIO_TypeDef      *s_cs_port;
static uint16_t           s_cs_pin;

static inline void cs_low(void)  { HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_SET);   }

static HAL_StatusTypeDef spi_tx(const uint8_t *d, uint16_t n)
{
    return HAL_SPI_Transmit(s_hspi, (uint8_t *)d, n, SPI_TIMEOUT_MS);
}

static HAL_StatusTypeDef spi_rx(uint8_t *d, uint16_t n)
{
    return HAL_SPI_Receive(s_hspi, d, n, SPI_TIMEOUT_MS);
}

static void send_cmd(uint8_t cmd)
{
    cs_low();
    spi_tx(&cmd, 1);
    cs_high();
}

static void fill_addr_cmd(uint8_t *h, uint8_t cmd, uint32_t addr)
{
    h[0] = cmd;
    h[1] = (uint8_t)(addr >> 16);
    h[2] = (uint8_t)(addr >> 8);
    h[3] = (uint8_t)(addr);
}

static HAL_StatusTypeDef wait_busy(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    do {
        if ((W25Q_ReadStatus1() & SR1_BUSY) == 0) return HAL_OK;
    } while ((HAL_GetTick() - t0) < timeout_ms);
    return HAL_TIMEOUT;
}

void W25Q_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    s_hspi    = hspi;
    s_cs_port = cs_port;
    s_cs_pin  = cs_pin;

    cs_high();
    HAL_Delay(5);                 /* power-up time */
    send_cmd(CMD_RELEASE_PD);     /* wake up in case it was powered down */
    HAL_Delay(1);
    send_cmd(CMD_ENABLE_RESET);   /* software reset */
    send_cmd(CMD_RESET);
    HAL_Delay(1);
}

uint32_t W25Q_ReadJedecId(void)
{
    uint8_t cmd = CMD_JEDEC_ID, id[3] = {0};
    cs_low();
    spi_tx(&cmd, 1);
    spi_rx(id, 3);
    cs_high();
    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

uint8_t W25Q_ReadStatus1(void)
{
    uint8_t cmd = CMD_READ_SR1, sr = 0xFF;
    cs_low();
    spi_tx(&cmd, 1);
    spi_rx(&sr, 1);
    cs_high();
    return sr;
}

HAL_StatusTypeDef W25Q_ReadData(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t h[4];
    HAL_StatusTypeDef st;

    fill_addr_cmd(h, CMD_READ_DATA, addr);
    cs_low();
    st = spi_tx(h, 4);
    while (st == HAL_OK && len) {
        uint16_t n = (len > 0xFFFFu) ? 0xFFFFu : (uint16_t)len;
        st   = spi_rx(buf, n);
        buf += n;
        len -= n;
    }
    cs_high();
    return st;
}

HAL_StatusTypeDef W25Q_PageProgram(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint8_t h[4];
    HAL_StatusTypeDef st;

    if (len == 0 || len > W25Q_PAGE_SIZE) return HAL_ERROR;

    fill_addr_cmd(h, CMD_PAGE_PROGRAM, addr);
    send_cmd(CMD_WRITE_ENABLE);
    cs_low();
    st = spi_tx(h, 4);
    if (st == HAL_OK) st = spi_tx(buf, (uint16_t)len);
    cs_high();
    if (st != HAL_OK) return st;
    return wait_busy(TPP_TIMEOUT_MS);
}

HAL_StatusTypeDef W25Q_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    while (len) {
        uint32_t room = W25Q_PAGE_SIZE - (addr % W25Q_PAGE_SIZE);  /* never cross a page */
        uint32_t n    = (len < room) ? len : room;
        HAL_StatusTypeDef st = W25Q_PageProgram(addr, buf, n);
        if (st != HAL_OK) return st;
        addr += n;
        buf  += n;
        len  -= n;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef erase_addr(uint8_t cmd, uint32_t addr, uint32_t timeout_ms)
{
    uint8_t h[4];
    HAL_StatusTypeDef st;

    fill_addr_cmd(h, cmd, addr);
    send_cmd(CMD_WRITE_ENABLE);
    cs_low();
    st = spi_tx(h, 4);
    cs_high();
    if (st != HAL_OK) return st;
    return wait_busy(timeout_ms);
}

HAL_StatusTypeDef W25Q_EraseSector4K(uint32_t addr)
{
    return erase_addr(CMD_SECTOR_ERASE, addr & ~(W25Q_SECTOR_SIZE - 1u), TSE_TIMEOUT_MS);
}

HAL_StatusTypeDef W25Q_EraseBlock64K(uint32_t addr)
{
    return erase_addr(CMD_BLOCK_ERASE64, addr & ~(W25Q_BLOCK_SIZE - 1u), TBE_TIMEOUT_MS);
}

HAL_StatusTypeDef W25Q_EraseChip(void)
{
    send_cmd(CMD_WRITE_ENABLE);
    send_cmd(CMD_CHIP_ERASE);
    return wait_busy(TCE_TIMEOUT_MS);
}
