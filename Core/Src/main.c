/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - MS5611, BME68X & BMI323
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI323_STM32.h"
#include "ms5611.h"
#include "bme68x_app.h"
#include "model.h"
#include <string.h>
#include <stdio.h>
#include "logger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_PERIOD_US     10000u   /* 100 Hz: taxa de UpdateIMU                */
#define BARO_PERIOD_MS    50u      /* 20 Hz: taxa de UpdateBaro                */
#define TELEM_PERIOD_MS   200u     /* 5 Hz: telemetria pela UART               */
#define BME_PERIOD_MS     1000u    /* 1 Hz: BME68X (so fora de voo)            */
#define CAL_SAMPLES       300u     /* amostras de calibracao (~3 s parado)     */
#define CAL_MAX_TRIES     5u
#define LIVE_ROTACAO      1        /*<------------------------------------------------------------ Mudar para 0 no lancamento-------------------------------*/
#define LIVE_PERIOD_MS    40u     /* 25 Hz */

/* Descomente para testar na bancada: baixa o limiar de decolagem para 1.3 g,
   assim uma "sacudida" da placa dispara a fase BOOST. */
#define PB_BENCH_TEST 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c4;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
float temperature_ms5611, pressure_ms5611;   /* pressure_ms5611 em Pa */
BMI323_Data imu_data;
BME68X_Data bme68x_data;
uint32_t led_last_toggle = 0;
 
Peabiru_t peabiru;
static Peabiru_Phase last_phase = PB_PHASE_PAD;
static const char *phase_name[] = { "PAD", "BOOST", "COAST", "DESCENT", "LANDED" };
 
static uint32_t imu_period_cyc;   /* periodo do IMU em ciclos do DWT */
static uint32_t last_imu_cyc;
static uint32_t last_baro_ms;
static uint32_t last_telem_ms;
static uint32_t last_bme_ms;
static uint32_t last_live_ms;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_SDMMC1_SD_Init(void);
/* USER CODE BEGIN PFP */
void I2C_Scanner(I2C_HandleTypeDef *hi2c);
static void    DWT_Init(void);
static uint8_t Peabiru_SetupAndCalibrate(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void I2C_Scanner(I2C_HandleTypeDef *hi2c)
{
    char buffer[64];
    uint8_t found = 0;

    sprintf(buffer, "\r\n--- Escaneando barramento I2C ---\r\n");
    HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);

    for (uint8_t addr = 1; addr < 128; addr++)
    {
        if (HAL_I2C_IsDeviceReady(hi2c, (addr << 1), 2, 10) == HAL_OK)
        {
            sprintf(buffer, "Dispositivo encontrado no endereco 0x%02X\r\n", addr);
            HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);
            found++;
        }
    }

    if (found == 0)
    {
        sprintf(buffer, "Nenhum dispositivo respondeu no barramento.\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);
    }

    sprintf(buffer, "--- Fim do scan (%d encontrado(s)) ---\r\n\r\n", found);
    HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);
}
/* Contador de ciclos da CPU: da um dt com resolucao de nanossegundos,
   bem melhor que o HAL_GetTick() de 1 ms para integrar aceleracao. */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;               /* destrava o DWT no Cortex-M7 */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
 
/* Inicializa o modelo e calibra com o Peabiru PARADO. Retorna 1 se deu certo. */
static uint8_t Peabiru_SetupAndCalibrate(void)
{
    Peabiru_Init(&peabiru);              /* zera tudo, inclusive acumuladores */
 
    /* Opcional: gravidade local (padrao 9.786 = Iacanga-SP) */
    /* Peabiru_SetLocalGravity(&peabiru, 9.78f); */
 
#ifdef PB_BENCH_TEST
    peabiru.boost_thresh_g = 1.4f;
#endif
 
    uint32_t n = 0;
    while (n < CAL_SAMPLES)
    {
        /* O barometro e lento: le a cada 10 amostras e reaproveita o valor */
        if ((n % 10u) == 0u)
        {
            MS5611_Measure(&hi2c4, &temperature_ms5611, &pressure_ms5611);
            if (pressure_ms5611 < 30000.0f) return 0;   /* leitura invalida */
        }
 
        if (BMI323_ReadData(&imu_data) == 0)
        {
            Peabiru_CalibAddSample(&peabiru,
                                   imu_data.accel_x_g, imu_data.accel_y_g, imu_data.accel_z_g,
                                   imu_data.gyro_x_dps, imu_data.gyro_y_dps, imu_data.gyro_z_dps,
                                   pressure_ms5611);
            n++;
        }
        HAL_Delay(10);
    }
 
    return Peabiru_CalibFinish(&peabiru);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C4_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  char buffer[256];

  /* Scan de dispositivos I2C no barramento 4 */
  I2C_Scanner(&hi2c4);

  /* ------------------------------- MS5611 ------------------------------- */
  MS5611_Init(&hi2c4, 1); /* teste: 1 = formula MS5607. Se a pressao ficar certa, mantenha assim; se piorar, volte para 0 */

  /* ------------------------------- BME68X ------------------------------- */
  if (BME68X_App_Init(&hi2c4) != BME68X_OK) 
  {
      sprintf(buffer, "Erro ao inicializar BME68X!\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);
  }

  /* -------------------------------- BMI323 -------------------------------- */
  uint8_t bmi_status = BMI323_Init();

  sprintf(buffer, "BMI323_Init() retornou: %d\r\n", bmi_status);
  HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);

  sprintf(buffer, "BMI323 CHIP_ID lido: 0x%02X (esperado: 0x43)\r\n", g_bmi323_debug_chip_id);
  HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);

   /* ------------------------------ MODELO -------------------------------- */
  DWT_Init();
  HAL_Delay(1000);   /* deixa o sensor assentar antes de calibrar */
 
  sprintf(buffer, "Calibrando o modelo: mantenha o Peabiru PARADO...\r\n");
  HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);
 
  uint8_t cal_ok = 0;
  for (uint32_t tentativa = 1; tentativa <= CAL_MAX_TRIES && !cal_ok; tentativa++)
  {
      cal_ok = Peabiru_SetupAndCalibrate();
      if (!cal_ok)
      {
          sprintf(buffer, "Calibracao falhou (tentativa %lu/%u)\r\n",
                  (unsigned long)tentativa, (unsigned)CAL_MAX_TRIES);
          HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);
          HAL_Delay(500);
      }
  }
 
  if (cal_ok)
  {
      sprintf(buffer, "Modelo calibrado. p0=%.2f hPa | escala acel=%.4f | giro bias(rad/s)=%.4f %.4f %.4f\r\n",
              peabiru.p0_pa / 100.0f, peabiru.accel_scale,
              peabiru.gbx, peabiru.gby, peabiru.gbz);
  }
  else
  {
      sprintf(buffer, "ERRO: modelo NAO calibrado (UpdateIMU/UpdateBaro serao ignorados)\r\n");
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);
 
  /* Temporizacao do loop principal */
  imu_period_cyc = (SystemCoreClock / 1000000u) * IMU_PERIOD_US;
  last_imu_cyc   = DWT->CYCCNT;
  last_baro_ms   = HAL_GetTick();
  last_telem_ms  = last_baro_ms;
  last_bme_ms    = last_baro_ms;

  /* ------------------------------ Logger -------------------------------- */
  if (Logger_Init())
  {
    snprintf(buffer, sizeof(buffer), "SD ok: gravando em %s\r\n", Logger_FileName());
    Logger_Print("t_ms,fase,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,press_pa,alt_m,vel_ms,acc_g,tilt_deg,x_m,y_m,vx_ms,vy_ms,roll_deg,pitch_deg,yaw_deg,acc_up_ms2,flight_time_s\n");
  }
  else
  {
    snprintf(buffer, sizeof(buffer), "ERRO: cartao SD nao montou\r\n");
  }
HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      /*----------------------------- LED ----------------------------------*/
    if (HAL_GetTick() - led_last_toggle >= 1000)
    {
      HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
      led_last_toggle = HAL_GetTick();
    }
 
    /*------------------- IMU a taxa fixa (100 Hz) -----------------------*/
    uint32_t now_cyc = DWT->CYCCNT;
    if ((uint32_t)(now_cyc - last_imu_cyc) >= imu_period_cyc)
    {
        /* dt REAL medido pelo DWT (a diferenca em uint32 ja trata o overflow) */
        float dt = (float)(uint32_t)(now_cyc - last_imu_cyc) / (float)SystemCoreClock;
        last_imu_cyc = now_cyc;
 
          if (BMI323_ReadData(&imu_data) == 0)
        {
            Peabiru_UpdateIMU(&peabiru,
                              imu_data.accel_x_g, imu_data.accel_y_g, imu_data.accel_z_g,
                              imu_data.gyro_x_dps, imu_data.gyro_y_dps, imu_data.gyro_z_dps,
                              dt);
 
            /* <<< ADICIONAR: uma linha de log por amostra do IMU >>> */
            Logger_Print("%lu,%d,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,"
             "%.1f,%.3f,%.3f,%.3f,%.2f,"
             "%.3f,%.3f,%.3f,%.3f,"
             "%.2f,%.2f,%.2f,%.3f,%.3f\n",
             (unsigned long)HAL_GetTick(),
             (int)peabiru.phase,
             imu_data.accel_x_g,
             imu_data.accel_y_g,
             imu_data.accel_z_g,
             imu_data.gyro_x_dps,
             imu_data.gyro_y_dps,
             imu_data.gyro_z_dps,
             pressure_ms5611,
             peabiru.alt_m,
             peabiru.vel_up_ms,
             peabiru.acc_mag_g,
             peabiru.tilt_deg,
             peabiru.x_m,
             peabiru.y_m,
             peabiru.vx_ms,
             peabiru.vy_ms,
             peabiru.roll_deg,
             peabiru.pitch_deg,
             peabiru.yaw_deg,
             peabiru.acc_up_ms2,
             peabiru.flight_time_s);
        }
    }
    /*------------------- Mudança Maquina de Estados -----------------------*/
    if (peabiru.phase != last_phase)
    {
    snprintf(buffer, sizeof(buffer),
             ">>> FASE: %s -> %s | t=%.2fs |a|=%.2fg vel=%.2fm/s alt=%.2fm\r\n",
             phase_name[last_phase], phase_name[peabiru.phase],
             peabiru.flight_time_s, peabiru.acc_mag_g,
             peabiru.vel_up_ms, peabiru.alt_m);
    HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);
    last_phase = peabiru.phase;
    }

    /*------------------- Barometro (20 Hz) ------------------------------*/
    if (HAL_GetTick() - last_baro_ms >= BARO_PERIOD_MS)
    {
        last_baro_ms = HAL_GetTick();
        MS5611_Measure(&hi2c4, &temperature_ms5611, &pressure_ms5611);   /* Pa */
        Peabiru_UpdateBaro(&peabiru, pressure_ms5611);
    }
 
    /*------------------- BME68X (so fora de voo) ------------------------*/
    if ((peabiru.phase == PB_PHASE_PAD || peabiru.phase == PB_PHASE_LANDED) &&
        (HAL_GetTick() - last_bme_ms >= BME_PERIOD_MS))
    {
        last_bme_ms = HAL_GetTick();
        if (BME68X_App_ReadData(&bme68x_data))
        {
            sprintf(buffer, "BME68X  -> Temp: %.2fC | Press: %.2fhPa | Hum: %.2f%% | Gas: %.0f Ohm\r\n",
                    bme68x_data.temperature, bme68x_data.pressure,
                    bme68x_data.humidity, bme68x_data.gas_resistance);
            HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);
        }
    }
 
    /*------------------- Telemetria (5 Hz) ------------------------------*/
    if (HAL_GetTick() - last_telem_ms >= TELEM_PERIOD_MS)
    {
        last_telem_ms = HAL_GetTick();
        snprintf(buffer, sizeof(buffer),
                 "[%s] alt=%.2fm vel=%.2fm/s |a|=%.2fg tilt=%.1fdeg x=%.1f y=%.1f max=%.1fm\r\n",
                 phase_name[peabiru.phase],
                 peabiru.alt_m, peabiru.vel_up_ms, peabiru.acc_mag_g,
                 peabiru.tilt_deg, peabiru.x_m, peabiru.y_m, peabiru.max_alt_m);
        HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);
    }

    #if LIVE_ROTACAO
    if (HAL_GetTick() - last_live_ms >= LIVE_PERIOD_MS)
    {
        last_live_ms = HAL_GetTick();
        snprintf(buffer, sizeof(buffer), "R,%lu,%.2f,%.2f,%.2f,%d,%.2f,%.2f\r\n",
                 (unsigned long)last_live_ms,
                 peabiru.roll_deg, peabiru.pitch_deg, peabiru.yaw_deg,
                 (int)peabiru.phase,
                 peabiru.alt_m, peabiru.vel_up_ms);
        HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);
    }
#endif
    Logger_Poll();
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x307075B1;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 8;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
