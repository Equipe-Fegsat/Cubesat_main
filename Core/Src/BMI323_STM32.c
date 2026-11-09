/**
  ******************************************************************************
  * @file    BMI323_STM32.c
  * @brief   Driver para o IMU BMI323 (Bosch) via I2C (HAL STM32).
  *
  * Peculiaridade importante do BMI323: todo registrador tem 16 bits (2 bytes,
  * LSB primeiro), e toda LEITURA por I2C insere 2 bytes "dummy" (sempre 0x00)
  * antes do dado real. Ou seja, para ler N bytes de dado real, e preciso
  * pedir N+2 bytes e descartar os 2 primeiros. Escritas NAO tem esse dummy.
  ******************************************************************************
  */

#include "BMI323_STM32.h"

extern I2C_HandleTypeDef hi2c4;
uint8_t g_bmi323_debug_chip_id = 0;

#define BMI323_REG_CHIP_ID     0x00
#define BMI323_REG_STATUS      0x02
#define BMI323_REG_ACC_DATA_X  0x03
#define BMI323_REG_ACC_CONF    0x20
#define BMI323_REG_GYR_CONF    0x21
#define BMI323_REG_CMD         0x7E

#define BMI323_CMD_SOFT_RESET  0xDEAF

/* Config usada: modo alta performance, sem media, filtro ODR/4,
 * accel +-2g, giro +-125dps, ODR 800Hz (0x708B para os dois registradores) */
#define BMI323_ACC_CONF_VALUE  0x708B
#define BMI323_GYR_CONF_VALUE  0x708B

/* Sensibilidades resultantes da config acima */
#define BMI323_ACCEL_LSB_PER_G   16384.0f   /* +-2g em 16 bits com sinal */
#define BMI323_GYRO_LSB_PER_DPS  262.144f   /* +-125dps em 16 bits com sinal */

/* Le 'len' bytes de dado real a partir do registrador 'reg', descartando
 * os 2 bytes dummy que o BMI323 sempre manda primeiro numa leitura I2C. */
static uint8_t BMI323_ReadRegs(uint8_t reg, uint8_t *out, uint16_t len)
{
    uint8_t staging[32];

    if (len + 2 > sizeof(staging))
        return 1;

    if (HAL_I2C_Mem_Read(&BMI323_I2C_HANDLE, (uint16_t)(BMI323_I2C_ADDR << 1),
                          reg, I2C_MEMADD_SIZE_8BIT, staging, len + 2, 100) != HAL_OK)
        return 1;

    for (uint16_t i = 0; i < len; i++)
        out[i] = staging[i + 2];

    return 0;
}

/* Escreve uma palavra de 16 bits (LSB primeiro) num registrador. Escritas
 * NAO tem bytes dummy. */
static uint8_t BMI323_WriteReg16(uint8_t reg, uint16_t value)
{
    uint8_t tx[2];
    tx[0] = (uint8_t)(value & 0xFF);
    tx[1] = (uint8_t)((value >> 8) & 0xFF);

    if (HAL_I2C_Mem_Write(&BMI323_I2C_HANDLE, (uint16_t)(BMI323_I2C_ADDR << 1),
                           reg, I2C_MEMADD_SIZE_8BIT, tx, 2, 100) != HAL_OK)
        return 1;

    return 0;
}

uint8_t BMI323_Init(void)
{
    uint8_t chip_id_raw[2];

    HAL_Delay(10); /* tempo de power-up do sensor */

    /* 1) Soft reset */
    if (BMI323_WriteReg16(BMI323_REG_CMD, BMI323_CMD_SOFT_RESET) != 0)
        return 1;
    HAL_Delay(5);

    /* 2) Confere CHIP_ID (deve ser 0x43 no byte baixo) */
    if (BMI323_ReadRegs(BMI323_REG_CHIP_ID, chip_id_raw, 2) != 0)
        return 1;

    g_bmi323_debug_chip_id = chip_id_raw[0];

    if (chip_id_raw[0] != 0x43)
        return 2; /* endereco/fiacao errados, ou nao e um BMI323 */

    /* 3) Configura accel: alta performance, +-2g, ODR 800Hz */
    if (BMI323_WriteReg16(BMI323_REG_ACC_CONF, BMI323_ACC_CONF_VALUE) != 0)
        return 3;

    /* 4) Configura giro: alta performance, +-125dps, ODR 800Hz */
    if (BMI323_WriteReg16(BMI323_REG_GYR_CONF, BMI323_GYR_CONF_VALUE) != 0)
        return 4;

    HAL_Delay(5); /* tempo de estabilizacao apos configurar */

    return 0;
}

uint8_t BMI323_ReadData(BMI323_Data *data)
{
    uint8_t raw[14]; /* AccelXYZ(6) + GyroXYZ(6) + Temp(2), 16 bits cada, LSB primeiro */
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, temp_raw;

    if (BMI323_ReadRegs(BMI323_REG_ACC_DATA_X, raw, 14) != 0)
        return 1;

    ax_raw   = (int16_t)(raw[0]  | (raw[1]  << 8));
    ay_raw   = (int16_t)(raw[2]  | (raw[3]  << 8));
    az_raw   = (int16_t)(raw[4]  | (raw[5]  << 8));
    gx_raw   = (int16_t)(raw[6]  | (raw[7]  << 8));
    gy_raw   = (int16_t)(raw[8]  | (raw[9]  << 8));
    gz_raw   = (int16_t)(raw[10] | (raw[11] << 8));
    temp_raw = (int16_t)(raw[12] | (raw[13] << 8));

    data->accel_x_g  = ax_raw / BMI323_ACCEL_LSB_PER_G;
    data->accel_y_g  = ay_raw / BMI323_ACCEL_LSB_PER_G;
    data->accel_z_g  = az_raw / BMI323_ACCEL_LSB_PER_G;

    data->gyro_x_dps = gx_raw / BMI323_GYRO_LSB_PER_DPS;
    data->gyro_y_dps = gy_raw / BMI323_GYRO_LSB_PER_DPS;
    data->gyro_z_dps = gz_raw / BMI323_GYRO_LSB_PER_DPS;

    /* Formula oficial da Bosch: Temp(C) = (raw / 512) + 23 */
    data->temp_c = (temp_raw / 512.0f) + 23.0f;

    return 0;
}