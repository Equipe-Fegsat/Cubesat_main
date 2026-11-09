/**
 * logger.c - Log CSV em cartao SD (FatFs sobre SDMMC1) para o Peabiru
 *
 * IMPORTANTE (STM32H7): o DMA interno do SDMMC1 so enxerga a AXI SRAM
 * (0x24000000). Por isso FATFS, FIL e o buffer de log ficam em ENDERECOS
 * FIXOS dentro dessa regiao, sem mexer no linker script (uma secao extra
 * no ELF fazia o STM32CubeProgrammer falhar ao gravar via USB).
 *
 * Mapa usado na AXI SRAM (nada mais do projeto usa essa regiao):
 *   0x24000000 - 0x24001FFF : FATFS (8 KB reservados)
 *   0x24002000 - 0x24003FFF : FIL   (8 KB reservados)
 *   0x24004000 - 0x24007FFF : buffer de log (16 KB)
 *
 * Assume D-Cache desligado (nao ha SCB_EnableDCache no projeto).
 */
#include "logger.h"
#include "fatfs.h"          /* gerado pelo CubeMX: SDPath, tipos do FatFs */
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 1 = imprime pela UART4 cada etapa da gravacao (use so para diagnostico) */
#define LOGGER_DEBUG   0 /*<------------------------------------------------------------ Mudar para 0 no Lancamento-------------------------------*/

extern UART_HandleTypeDef huart4;

#define AXI_BASE       0x24000000u
#define FS_ADDR        (AXI_BASE + 0x0000u)
#define FIL_ADDR       (AXI_BASE + 0x2000u)
#define BUF_ADDR       (AXI_BASE + 0x4000u)

#define LOG_BUF_SIZE   16384u   /* ~1,6 s de dados a 100 Hz */
#define LOG_FLUSH_MS   250u

_Static_assert(sizeof(FATFS) <= 0x2000u, "FATFS maior que o espaco reservado");
_Static_assert(sizeof(FIL)   <= 0x2000u, "FIL maior que o espaco reservado");

static FATFS   *const pfs = (FATFS *)FS_ADDR;
static FIL     *const pfil = (FIL *)FIL_ADDR;
static uint8_t *const buf = (uint8_t *)BUF_ADDR;

static uint32_t head;           /* bytes pendentes no buffer */
static uint32_t last_flush_ms;
static uint32_t dropped, errors;
static uint8_t  opened;
static char     file_name[16];

uint8_t Logger_Init(void)
{
    opened = 0;
    head = 0;

    if (f_mount(pfs, SDPath, 1) != FR_OK) return 0;

    /* procura o primeiro nome livre: LOG000.CSV, LOG001.CSV, ... */
    for (int i = 0; i < 1000; i++)
    {
        snprintf(file_name, sizeof(file_name), "LOG%03d.CSV", i);
        FRESULT r = f_open(pfil, file_name, FA_CREATE_NEW | FA_WRITE);
        if (r == FR_OK)
        {
            opened = 1;
            last_flush_ms = HAL_GetTick();
            return 1;
        }
        if (r != FR_EXIST) return 0;    /* erro de verdade, nao so "ja existe" */
    }
    return 0;
}

void Logger_Print(const char *fmt, ...)
{
    if (!opened) return;

    uint32_t livre = LOG_BUF_SIZE - head;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf((char *)&buf[head], livre, fmt, ap);
    va_end(ap);

    if (n < 0 || (uint32_t)n >= livre) { dropped++; return; }   /* nao coube */
    head += (uint32_t)n;
}

void Logger_Poll(void)
{
    if (!opened || head == 0) return;
    if (HAL_GetTick() - last_flush_ms < LOG_FLUSH_MS) return;
    last_flush_ms = HAL_GetTick();

#if LOGGER_DEBUG
    char m[96];
    snprintf(m, sizeof(m), "[LOG] f_write de %lu bytes...\r\n", (unsigned long)head);
    HAL_UART_Transmit(&huart4, (uint8_t *)m, strlen(m), 100);
#endif

    UINT bw = 0;
    FRESULT r = f_write(pfil, buf, head, &bw);
    if (r != FR_OK || bw != head) errors++;

#if LOGGER_DEBUG
    snprintf(m, sizeof(m), "[LOG] f_write=%d bw=%lu\r\n", (int)r, (unsigned long)bw);
    HAL_UART_Transmit(&huart4, (uint8_t *)m, strlen(m), 100);
#endif

    head = 0;                       /* descarta mesmo em erro, para nao travar o buffer */

    FRESULT rs = f_sync(pfil);
    if (rs != FR_OK) errors++;

#if LOGGER_DEBUG
    snprintf(m, sizeof(m), "[LOG] f_sync=%d\r\n", (int)rs);
    HAL_UART_Transmit(&huart4, (uint8_t *)m, strlen(m), 100);
#endif
}

const char *Logger_FileName(void) { return file_name; }
uint32_t    Logger_Dropped(void)  { return dropped; }
uint32_t    Logger_Errors(void)   { return errors; }