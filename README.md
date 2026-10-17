# CubeSat Peabiru – FegSat

Firmware do **CubeSat Peabiru**, desenvolvido pela equipe FegSat (Unesp Guaratinguetá).

O firmware roda em um microcontrolador **STM32H743VIT6**, lê sensores de pressão, temperatura, umidade, qualidade do ar e movimento, e envia os dados pela UART para acompanhamento e depuração.

## Visão geral

```
 ┌──────────┐
 │  MS5611  │ ──┐ I2C4
 ├──────────┤   │         ┌───────────────┐  UART4    ┌────────────┐
 │  BME68X  │ ──┴───────► │               │ ────────► │  Terminal  │
 ├──────────┤             │  STM32H743    │ 115200    │  (debug)   │
 │  BMI323  │ ──── SPI ─► │               │           └────────────┘
 └──────────┘             └───────────────┘
```

## Hardware

| Componente | Grandeza medida | Interface |
|---|---|---|
| **STM32H743VIT6** (placa WeAct) | Microcontrolador principal | – |
| **MS5611** | Pressão e temperatura | I2C4 |
| **BME68X** (família BME680/688) | Temperatura, pressão, umidade e resistência de gás | I2C4 |
| **BMI323** (módulo GY-601) | Acelerômetro e giroscópio (3 eixos) + temperatura | SPI [confirmar: SPI2 ou SPI3] |
| **UART4** | Saída serial de depuração | 115200 baud, 8N1 |
| **LED de status** | Pisca a cada 1 s para indicar que o firmware está rodando | PE3 |

### Pinagem

| Função | Pinos |
|---|---|
| UART4 TX / RX | PA0 / PA1 |
| I2C4 SCL / SDA (MS5611 e BME68X) | PB8 / PB9 |
| I2C1 SCL / SDA | PB6 / PB7 (configurado no `.ioc`, ainda não usado no `main.c`) |
| SPI2 SCK / MISO / MOSI | PB10 / PC2 / PC1 |
| SPI3 SCK / MISO / MOSI | PC10 / PC11 / PB2 |
| CS do BMI323 (GPIO, nível alto em repouso) | PB11 (`.ioc`) / PB12 (`main.c` atual) [confirmar qual está ligado] |
| ADC1 (canal 3) | PA6 |
| ADC (canal 4, apenas configurado) | PC4 |
| LED de status | PE3 |
| Entrada com pull-down | PC13 |
| Depuração SWD | PA13 (SWDIO) / PA14 (SWCLK) |

## Funcionamento atual do firmware

O `main.c` foi gerado com o **STM32CubeMX** (HAL). Depois da inicialização dos periféricos, o firmware faz o seguinte:

1. **Scanner I2C:** varre o barramento I2C4 e lista na serial os endereços dos dispositivos que responderem.
2. **Inicialização dos sensores:**
   - MS5611 (com a fórmula de compensação do MS5607 em teste; veja a nota abaixo).
   - BME68X, com mensagem de erro caso a inicialização falhe.
   - BMI323, imprimindo o retorno da inicialização e o `CHIP_ID` lido (o esperado é `0x43`).
3. **Loop principal (a cada 1 s):**
   - Alterna o LED de status.
   - Lê e imprime os dados do BME68X, do MS5611 e do BMI323.

> **Nota sobre o MS5611:** a inicialização é feita com `MS5611_Init(&hi2c4, 1)`, usando a fórmula do MS5607 como teste. Se a pressão medida estiver correta, mantenha; se piorar, volte o parâmetro para `0`.

### Exemplo de saída serial

```
BME68X  -> Temp: 25.10C | Press: 1012.80hPa | Hum: 47.30% | Gas: 85000 Ohm
MS5611  -> Temp: 25.30 C | Press: 1013.10 hPa
BMI323  -> Accel[g]: X=0.010 Y=-0.004 Z=0.999 | Gyro[dps]: X=0.12 Y=-0.05 Z=0.03 | Temp=26.0C
------------------------------------------------------------------------------------
```

### Configuração do microcontrolador

| Item | Valor |
|---|---|
| Fonte de clock | HSI (64 MHz) → PLL1 (÷4, ×60, ÷2) |
| SYSCLK | 480 MHz |
| HCLK / barramentos APB | 240 MHz / 120 MHz |
| Regulador | LDO, escala de tensão VOS0 |
| I2C4 | 100 kHz (modo padrão), filtro analógico ativado |
| SPI2 / SPI3 | Mestre, 8 bits, prescaler 32 (≈ 6 Mbit/s) |
| UART4 | 115200 baud, 8N1, sem controle de fluxo |
| ADC1 | 16 bits, canal 3 (inicializado, ainda não usado no `main.c`) |

## Estrutura do projeto

O projeto foi gerado pelo STM32CubeMX 6.17.0, com o pacote STM32Cube FW_H7 V1.13.0 e toolchain **CMake**.

```
.
├── README.md
├── Cubesat_fegsat.ioc    # Configuração do CubeMX (pinos, clocks, periféricos)
├── CMakeLists.txt
├── Core/
│   ├── Inc/              # Headers (main.h, ms5611.h, bme68x_app.h, BMI323_STM32.h, ...)
│   └── Src/              # main.c e drivers dos sensores
└── Drivers/              # HAL / CMSIS (gerados pelo CubeMX)
```

> Ajuste a árvore acima para refletir a estrutura real do seu repositório.

## Como compilar e gravar

1. Abra a pasta do projeto no **VS Code** com a extensão **STM32Cube** (ou em outra IDE com suporte a CMake).
2. Compile o projeto.
3. Conecte a placa por **ST-Link** (SWD: PA13 e PA14) e grave o firmware.
4. Abra um terminal serial a **115200 baud**, na UART4 (PA0/PA1), para acompanhar as leituras.

Para alterar pinos, clocks ou periféricos, abra o `Cubesat_fegsat.ioc` no **STM32CubeMX** e gere o código de novo. O código dentro dos blocos `USER CODE BEGIN/END` é preservado.

## Próximos passos

- [ ] Validar a fórmula de compensação do MS5611 (MS5611 × MS5607)
- [ ] [Comunicação com a estação de solo / outros subsistemas]
- [ ] [Armazenamento de dados, telemetria e demais itens]

## Equipe

Projeto desenvolvido pela **FegSat**, equipe de nanossatélites da Unesp Guaratinguetá.
