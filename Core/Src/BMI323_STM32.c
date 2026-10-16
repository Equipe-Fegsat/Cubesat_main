/**
  ******************************************************************************
  * @file    BMI323_STM32.c
  * @brief   Driver para o IMU BMI323 (Bosch) via SPI2 (HAL STM32).
  *
  * Peculiaridade importante do BMI323: todo registrador tem 16 bits (2 bytes,
  * LSB primeiro), e toda LEITURA insere 2 bytes "dummy" (sempre 0x00) antes
  * do dado real. No modo SPI, o quadro de leitura fica:
  *     [endereco | 0x80]  [dummy]  [dummy]  [dado0] [dado1] ...
  * Ou seja, para ler N bytes de dado real, e git preciso clocar 1 (endereco) +
  * 2 (dummy) + N bytes, com CS mantido em nivel baixo durante toda a
  * transacao. Escritas NAO tem dummy: [endereco] [dado0] [dado1] ...
  *
  * Alem disso, apos o power-on/reset o BMI323 comeca em modo I2C/I3C por
  * padrao; uma leitura SPI "em vazio" logo no inicio faz o sensor detectar
  * a borda em CS e trocar para o modo SPI (recomendacao da Bosch).
  ******************************************************************************
  */

#include "BMI323_STM32.h"

extern SPI_HandleTypeDef BMI323_SPI_HANDLE;
uint8_t g_bmi323_debug_chip_id = 0;

#define BMI323_REG_CHIP_ID     0x00
#define BMI323_REG_STATUS      0x02
#define BMI323_REG_ACC_DATA_X  0x03
#define BMI323_REG_ACC_CONF    0x20
#define BMI323_REG_GYR_CONF    0x21
#define BMI323_REG_CMD         0x7E

#define BMI323_CMD_SOFT_RESET  0xDEAF

#define BMI323_READ_BIT        0x80U
#define BMI323_SPI_TIMEOUT     100U

/* Config usada: modo alta performance, sem media, filtro ODR/4,
 * accel +-2g, giro +-125dps, ODR 800Hz (0x708B para os dois registradores) */
#define BMI323_ACC_CONF_VALUE  0x708B
#define BMI323_GYR_CONF_VALUE  0x708B

/* Sensibilidades resultantes da config acima */
#define BMI323_ACCEL_LSB_PER_G   16384.0f   /* +-2g em 16 bits com sinal */
#define BMI323_GYRO_LSB_PER_DPS  262.144f   /* +-125dps em 16 bits com sinal */

static inline void BMI323_CS_Low(void)
{
    HAL_GPIO_WritePin(BMI323_CS_GPIO_Port, BMI323_CS_Pin, GPIO_PIN_RESET);
}

static inline void BMI323_CS_High(void)
{
    HAL_GPIO_WritePin(BMI323_CS_GPIO_Port, BMI323_CS_Pin, GPIO_PIN_SET);
}

/* Le 'len' bytes de dado real a partir do registrador 'reg', descartando
 * os 2 bytes dummy que o BMI323 sempre manda primeiro numa leitura. */
static uint8_t BMI323_ReadRegs(uint8_t reg, uint8_t *out, uint16_t len)
{
    uint8_t tx[32] = {0};
    uint8_t rx[32] = {0};
    uint16_t total = (uint16_t)(len + 3); /* 1 endereco + 2 dummy + N dados */
    HAL_StatusTypeDef st;

    if (total > sizeof(tx))
        return 1;

    tx[0] = reg | BMI323_READ_BIT;

    BMI323_CS_Low();
    st = HAL_SPI_TransmitReceive(&BMI323_SPI_HANDLE, tx, rx, total, BMI323_SPI_TIMEOUT);
    BMI323_CS_High();

    if (st != HAL_OK)
        return 1;

    for (uint16_t i = 0; i < len; i++)
        out[i] = rx[i + 3];

    return 0;
}

/* Escreve uma palavra de 16 bits (LSB primeiro) num registrador. Escritas
 * NAO tem bytes dummy. */
static uint8_t BMI323_WriteReg16(uint8_t reg, uint16_t value)
{
    uint8_t tx[3];
    HAL_StatusTypeDef st;

    tx[0] = reg & 0x7F; /* bit 7 = 0 -> escrita */
    tx[1] = (uint8_t)(value & 0xFF);
    tx[2] = (uint8_t)((value >> 8) & 0xFF);

    BMI323_CS_Low();
    st = HAL_SPI_Transmit(&BMI323_SPI_HANDLE, tx, sizeof(tx), BMI323_SPI_TIMEOUT);
    BMI323_CS_High();

    return (st == HAL_OK) ? 0 : 1;
}

uint8_t BMI323_Init(void)
{
    uint8_t chip_id_raw[2];
    uint8_t dummy[2];

    HAL_Delay(10);      /* tempo de power-up do sensor */
    BMI323_CS_High();   /* garante CS em repouso antes da 1a transacao */

    /* Leitura "em vazio": a borda de CS faz o sensor sair do modo I2C/I3C
     * padrao e passar a responder em SPI. O conteudo lido aqui e ignorado. */
    BMI323_ReadRegs(BMI323_REG_CHIP_ID, dummy, 2);

    /* 1) Soft reset */
    if (BMI323_WriteReg16(BMI323_REG_CMD, BMI323_CMD_SOFT_RESET) != 0)
        return 1;
    HAL_Delay(5);

    /* 2) Confere CHIP_ID (deve ser 0x43 no byte baixo) */
    if (BMI323_ReadRegs(BMI323_REG_CHIP_ID, chip_id_raw, 2) != 0)
        return 1;

    g_bmi323_debug_chip_id = chip_id_raw[0];

    if (chip_id_raw[0] != 0x43)
        return 2; /* fiacao errada (CS/SCLK/SDI/SDO), ou nao e um BMI323 */

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