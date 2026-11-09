/**
 * logger.h - Log CSV em cartao SD (FatFs sobre SDMMC1) para o Peabiru
 *
 * Uso:
 *   Logger_Init();                 // monta o cartao e cria LOGxxx.CSV
 *   Logger_Print("fmt", ...);      // acumula uma linha num buffer em RAM (rapido)
 *   Logger_Poll();                 // chamar sempre no loop: grava em bloco a cada ~250 ms
 */
#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t     Logger_Init(void);              /* 1 = ok, 0 = falhou */
void        Logger_Print(const char *fmt, ...);
void        Logger_Poll(void);
const char *Logger_FileName(void);
uint32_t    Logger_Dropped(void);           /* linhas perdidas (buffer cheio) */
uint32_t    Logger_Errors(void);            /* erros de escrita no cartao */

#ifdef __cplusplus
}
#endif
#endif
