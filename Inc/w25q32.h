/*
 * w25q32.h - Minimal Winbond W25Q32 (4 MB) SPI NOR flash driver for STM32 HAL
 *
 * Wiring (NUCLEO-F411RE, SPI1):
 *   Module CLK -> D13 (PA5, SPI1_SCK)
 *   Module DO  -> D12 (PA6, SPI1_MISO)
 *   Module DI  -> D11 (PA7, SPI1_MOSI)
 *   Module CS  -> D10 (PB6, GPIO output, label FLASH_CS)
 *   Module VCC -> 3V3   (NOT 5V)
 *   Module GND -> GND
 */
#ifndef W25Q32_H
#define W25Q32_H

#include "main.h"
#include <stdint.h>

#define W25Q_PAGE_SIZE     256u
#define W25Q_SECTOR_SIZE   4096u
#define W25Q_BLOCK_SIZE    65536u
#define W25Q_TOTAL_SIZE    (4u * 1024u * 1024u)   /* 32 Mbit = 4 MB */
#define W25Q32_JEDEC_ID    0xEF4016u              /* Winbond, W25Q32 */

void              W25Q_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
uint32_t          W25Q_ReadJedecId(void);
uint8_t           W25Q_ReadStatus1(void);

HAL_StatusTypeDef W25Q_ReadData(uint32_t addr, uint8_t *buf, uint32_t len);
HAL_StatusTypeDef W25Q_PageProgram(uint32_t addr, const uint8_t *buf, uint32_t len); /* len <= 256, same page */
HAL_StatusTypeDef W25Q_Write(uint32_t addr, const uint8_t *buf, uint32_t len);       /* area must be erased */

HAL_StatusTypeDef W25Q_EraseSector4K(uint32_t addr);
HAL_StatusTypeDef W25Q_EraseBlock64K(uint32_t addr);
HAL_StatusTypeDef W25Q_EraseChip(void);

#endif /* W25Q32_H */
