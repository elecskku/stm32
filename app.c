/*
 * app.c - Binary transfer protocol over USART2 (ST-LINK virtual COM port)
 *
 * Commands (PC -> STM32, 1 byte):
 *   'I'  Info      -> ACK, JEDEC ID[3], SR1
 *   'W'  Write     PC: 'W' + size(u32)
 *                  STM: erases needed blocks, then ACK (ready)
 *                  PC repeats per chunk: idx(u32) len(u16) data[len] crc32(u32)
 *                  STM: writes, reads back, compares -> ACK / NAK (resend) / CAN (fatal)
 *                  After last chunk: ACK + crc32_of_whole_file(u32)
 *   'R'  Read      -> ACK size(u32) stored_crc(u32) data[size] crc_of_data_read(u32)
 *                  (or NAK if no valid file is stored)
 *   'V'  Verify    -> ACK size(u32) stored_crc(u32) recomputed_crc(u32)
 *   'E'  Erase all -> ACK when done (up to ~50 s)
 *
 * All integers are little-endian. CRC32 is the standard zlib/PNG CRC32,
 * identical to Python's zlib.crc32().
 *
 * Flash layout (W25Q32, 4 MB):
 *   0x000000 .. 0x3FEFFF  file data (max 4,190,208 bytes)
 *   0x3FF000 .. 0x3FFFFF  metadata sector (magic, size, crc32)
 */
#include "app.h"
#include "main.h"
#include "w25q32.h"
#include <string.h>

extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef  hspi1;

#define ACK  0x06
#define NAK  0x15
#define CAN  0x18

#define CHUNK_SIZE   4096u
#define META_ADDR    (W25Q_TOTAL_SIZE - W25Q_SECTOR_SIZE)
#define DATA_MAX     META_ADDR
#define META_MAGIC   0x31504D42u            /* "BMP1" */
#define MAX_FAILS    10

typedef struct {
    uint32_t magic;
    uint32_t size;
    uint32_t crc32;
    uint32_t magic_inv;
} meta_t;

static uint8_t  rx_buf[CHUNK_SIZE + 4];
static uint8_t  io_buf[CHUNK_SIZE];
static uint32_t crc_table[256];

/* ---------------- CRC32 (zlib compatible) ---------------- */
static void crc32_init(void)
{
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++)
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        crc_table[i] = c;
    }
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *p, uint32_t n)
{
    crc = ~crc;
    while (n--) crc = crc_table[(crc ^ *p++) & 0xFFu] ^ (crc >> 8);
    return ~crc;
}

/* ---------------- little-endian helpers ---------------- */
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ---------------- UART helpers ---------------- */
static void uart_send(const uint8_t *p, uint16_t n)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)p, n, 1000);
}

static void send_byte(uint8_t b)
{
    uart_send(&b, 1);
}

static HAL_StatusTypeDef uart_recv(uint8_t *p, uint16_t n, uint32_t timeout_ms)
{
    HAL_StatusTypeDef st = HAL_UART_Receive(&huart2, p, n, timeout_ms);
    if (st != HAL_OK) __HAL_UART_CLEAR_OREFLAG(&huart2);
    return st;
}

/* Discard incoming bytes until the line has been silent for 50 ms. */
static void uart_drain(void)
{
    uint8_t d;
    while (HAL_UART_Receive(&huart2, &d, 1, 50) == HAL_OK) { }
    __HAL_UART_CLEAR_OREFLAG(&huart2);
}

/* ---------------- metadata ---------------- */
static int meta_read(meta_t *m)
{
    if (W25Q_ReadData(META_ADDR, (uint8_t *)m, sizeof(*m)) != HAL_OK) return 0;
    return m->magic == META_MAGIC && m->magic_inv == ~META_MAGIC &&
           m->size > 0 && m->size <= DATA_MAX;
}

/* ---------------- commands ---------------- */
static void cmd_info(void)
{
    uint32_t id = W25Q_ReadJedecId();
    uint8_t  r[5];
    r[0] = ACK;
    r[1] = (uint8_t)(id >> 16);
    r[2] = (uint8_t)(id >> 8);
    r[3] = (uint8_t)id;
    r[4] = W25Q_ReadStatus1();
    uart_send(r, sizeof(r));
}

static void cmd_write(void)
{
    uint8_t h[6];
    if (uart_recv(h, 4, 500) != HAL_OK) { uart_drain(); send_byte(NAK); return; }

    uint32_t size = get_u32(h);
    if (size == 0 || size > DATA_MAX) { send_byte(NAK); return; }

    /* Erase only what is needed (64 KB blocks) + metadata sector */
    for (uint32_t a = 0; a < size; a += W25Q_BLOCK_SIZE)
        if (W25Q_EraseBlock64K(a) != HAL_OK) { send_byte(CAN); return; }
    if (W25Q_EraseSector4K(META_ADDR) != HAL_OK) { send_byte(CAN); return; }

    send_byte(ACK);   /* ready for data */

    uint32_t nchunks = (size + CHUNK_SIZE - 1u) / CHUNK_SIZE;
    uint32_t next = 0, crc = 0;
    int fails = 0;

    while (next < nchunks) {
        if (fails > MAX_FAILS) return;               /* give up, back to command loop */

        if (uart_recv(h, 6, 5000) != HAL_OK) { fails++; uart_drain(); send_byte(NAK); continue; }

        uint32_t idx = get_u32(h);
        uint16_t len = (uint16_t)(h[4] | (h[5] << 8));
        if (len == 0 || len > CHUNK_SIZE) { fails++; uart_drain(); send_byte(NAK); continue; }

        if (uart_recv(rx_buf, (uint16_t)(len + 4u), 1000) != HAL_OK) { fails++; uart_drain(); send_byte(NAK); continue; }

        if (crc32_update(0, rx_buf, len) != get_u32(rx_buf + len)) { fails++; send_byte(NAK); continue; }

        if (idx < next) { send_byte(ACK); continue; }  /* duplicate (our ACK was lost) */

        uint32_t expect_len = (idx == nchunks - 1u) ? (size - idx * CHUNK_SIZE) : CHUNK_SIZE;
        if (idx != next || len != expect_len) { fails++; send_byte(NAK); continue; }

        /* Program, then read back and compare (write verification) */
        uint32_t addr = idx * CHUNK_SIZE;
        if (W25Q_Write(addr, rx_buf, len) != HAL_OK ||
            W25Q_ReadData(addr, io_buf, len) != HAL_OK ||
            memcmp(rx_buf, io_buf, len) != 0) {
            send_byte(CAN);
            return;
        }

        crc = crc32_update(crc, rx_buf, len);
        next++;
        fails = 0;
        send_byte(ACK);
    }

    meta_t m = { META_MAGIC, size, crc, ~META_MAGIC };
    if (W25Q_Write(META_ADDR, (const uint8_t *)&m, sizeof(m)) != HAL_OK) { send_byte(CAN); return; }

    uint8_t r[5];
    r[0] = ACK;
    put_u32(r + 1, crc);
    uart_send(r, sizeof(r));
}

static void cmd_read(void)
{
    meta_t m;
    if (!meta_read(&m)) { send_byte(NAK); return; }

    uint8_t h[9];
    h[0] = ACK;
    put_u32(h + 1, m.size);
    put_u32(h + 5, m.crc32);
    uart_send(h, sizeof(h));

    uint32_t crc = 0;
    for (uint32_t a = 0; a < m.size; a += CHUNK_SIZE) {
        uint32_t n = m.size - a;
        if (n > CHUNK_SIZE) n = CHUNK_SIZE;
        W25Q_ReadData(a, io_buf, n);
        crc = crc32_update(crc, io_buf, n);
        uart_send(io_buf, (uint16_t)n);
    }

    uint8_t t[4];
    put_u32(t, crc);
    uart_send(t, sizeof(t));
}

static void cmd_verify(void)
{
    meta_t m;
    if (!meta_read(&m)) { send_byte(NAK); return; }

    uint32_t crc = 0;
    for (uint32_t a = 0; a < m.size; a += CHUNK_SIZE) {
        uint32_t n = m.size - a;
        if (n > CHUNK_SIZE) n = CHUNK_SIZE;
        W25Q_ReadData(a, io_buf, n);
        crc = crc32_update(crc, io_buf, n);
    }

    uint8_t r[13];
    r[0] = ACK;
    put_u32(r + 1, m.size);
    put_u32(r + 5, m.crc32);
    put_u32(r + 9, crc);
    uart_send(r, sizeof(r));
}

static void cmd_erase(void)
{
    send_byte(W25Q_EraseChip() == HAL_OK ? ACK : CAN);
}

/* ---------------- public ---------------- */
void App_Init(void)
{
    crc32_init();
    W25Q_Init(&hspi1, FLASH_CS_GPIO_Port, FLASH_CS_Pin);
    uart_drain();
}

void App_Loop(void)
{
    uint8_t c;
    if (uart_recv(&c, 1, 100) != HAL_OK) return;

    switch (c) {
    case 'I': cmd_info();   break;
    case 'W': cmd_write();  break;
    case 'R': cmd_read();   break;
    case 'V': cmd_verify(); break;
    case 'E': cmd_erase();  break;
    default:  break;        /* ignore unknown bytes */
    }
}
