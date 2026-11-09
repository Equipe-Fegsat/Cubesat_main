/**
  ******************************************************************************
  * @file    BMI323_STM32.h
  * @brief   Driver para o IMU BMI323 (Bosch) via I2C usando HAL STM32.
  *          Substitui o driver ICM42688 - o sensor da placa GY-601N1 do
  *          projeto é na verdade um BMI323, confirmado via CHIP_ID (0x43).
  ******************************************************************************
  */

#ifndef BMI323_STM32_H
#define BMI323_STM32_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---------------- Configuração de Hardware ---------------- */
#define BMI323_I2C_HANDLE      hi2c4
/* Endereco I2C: 0x68 se SA0 = GND, 0x69 se SA0 = VCC (fiacao atual do projeto) */
#define BMI323_I2C_ADDR        0x69

/* Guarda o ultimo CHIP_ID lido (byte baixo), para debug via UART */
extern uint8_t g_bmi323_debug_chip_id;

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temp_c;
} BMI323_Data;

/* Inicializa o sensor: soft reset + configura accel (+-2g) e giro (+-125dps)
 * a 800Hz em modo alta performance. Retorna 0 em sucesso.
 * 1 = falha de comunicacao no soft reset
 * 2 = CHIP_ID nao bateu com 0x43 (confira endereco/fiacao)
 * 3 = falha ao configurar ACC_CONF
 * 4 = falha ao configurar GYR_CONF
 */
uint8_t BMI323_Init(void);

/* Le accel + giro + temperatura. Retorna 0 em sucesso. */
uint8_t BMI323_ReadData(BMI323_Data *data);

#ifdef __cplusplus
}
#endif

#endif /* BMI323_STM32_H */