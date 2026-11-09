# CubeSat Peabiru – FegSat

Firmware do **CubeSat Peabiru**, desenvolvido pela equipe FegSat (Unesp Guaratinguetá).

O firmware roda em um microcontrolador **STM32H743VIT6**, lê sensores de pressão, temperatura, umidade, qualidade do ar e movimento, **estima a trajetória** (atitude, altura e velocidade vertical), **grava os dados em cartão SD** e envia telemetria pela UART para acompanhamento e depuração.

> **Versões:** esta é a versão **com estimador de trajetória**, a que a equipe usa. A versão anterior, que só lê e imprime os sensores, está guardada na tag `v1.0-sem-estimador` (no GitHub, escolha a tag no menu de branches, ou rode `git checkout v1.0-sem-estimador`).

## Visão geral

```
 ┌──────────┐
 │  MS5611  │ ──┐
 ├──────────┤   │ I2C4     ┌───────────────┐  UART4    ┌─────────────────┐
 │  BME68X  │ ──┼────────► │               │ ────────► │ Terminal /      │
 ├──────────┤   │          │  STM32H743    │ 115200    │ simuladores     │
 │  BMI323  │ ──┘          │               │           │ Python          │
 └──────────┘              │               │           └─────────────────┘
                           │               │  SDMMC1   ┌─────────────────┐
                           │               │ ────────► │ Cartão SD (CSV) │
                           └───────────────┘  (4 bits) └─────────────────┘
```

## Hardware

| Componente | Grandeza medida | Interface |
|---|---|---|
| **STM32H743VIT6** (placa WeAct) | Microcontrolador principal | – |
| **MS5611** | Pressão e temperatura (a pressão alimenta o estimador) | I2C4 |
| **BME68X** (família BME680/688) | Temperatura, pressão, umidade e resistência de gás | I2C4 |
| **BMI323** (módulo GY-601) | Acelerômetro e giroscópio (3 eixos) + temperatura | I2C4 |
| **Cartão SD** (slot da placa WeAct) | Gravação do log em CSV | SDMMC1, 4 bits |
| **UART4** | Telemetria e depuração | 115200 baud, 8N1 |
| **LED de status** | Pisca a cada 1 s para indicar que o firmware está rodando | PE3 |

### Pinagem

| Função | Pinos |
|---|---|
| UART4 TX / RX | PA0 / PA1 |
| I2C4 SCL / SDA (MS5611, BME68X e BMI323) | PB8 / PB9 |
| SDMMC1 (cartão SD, 4 bits) | PC8, PC9, PC10, PC11 (dados), PC12 (clock), PD2 (comando) |
| LED de status | PE3 |
| I2C1 SCL / SDA | PB6 / PB7 (configurado no `.ioc`, ainda não usado no `main.c`) |
| SPI2 SCK / MISO / MOSI | PB10 / PC2 / PC1 (configurado no `.ioc`, não usado no `main.c`) |
| PB12 | Saída em nível alto no `main.c` (era o CS do BMI323 quando se previa SPI) [confirmar se ainda é usado] |
| ADC1 (canal 3) | PA6 (inicializado, não usado no `main.c`) |
| ADC (canal 4, apenas configurado) | PC4 |
| Entrada com pull-down | PC13 |
| Depuração SWD | PA13 (SWDIO) / PA14 (SWCLK) |

> Os pinos PC10 a PC12 pertencem ao slot SD da placa. Eles **não podem ser usados por outro periférico** (por exemplo, SPI3), senão o cartão deixa de funcionar.

## Funcionamento do firmware

O `main.c` foi gerado com o **STM32CubeMX** (HAL) e o código do usuário fica nos blocos `USER CODE`. Depois da inicialização dos periféricos, o firmware faz o seguinte:

1. **Scanner I2C:** varre o barramento I2C4 e lista na serial os endereços que responderem (o BMI323 aparece em `0x69`, e o MS5611 e o BME68X em `0x76` e `0x77`).
2. **Inicialização dos sensores:** MS5611 (`MS5611_Init(&hi2c4, 1)`, com a fórmula do MS5607 em teste), BME68X e BMI323 (imprime o retorno e o `CHIP_ID`, esperado `0x43`).
3. **Calibração do estimador:** com o satélite **parado**, o firmware coleta 300 amostras (cerca de 3 s), com até 5 tentativas. Ela calcula o bias do giroscópio, a escala do acelerômetro (parado, `|a|` deve ser 1 g), a pressão de referência do solo (`p0`) e a atitude inicial pela gravidade.
4. **Log em SD:** monta o cartão e cria o próximo arquivo `LOGxxx.CSV` livre.
5. **Loop principal**, com tempos fixos:

| Tarefa | Taxa |
|---|---|
| Leitura do IMU e `Peabiru_UpdateIMU` (o `dt` é medido pelo contador de ciclos da CPU, o DWT) | 100 Hz (alvo) |
| Leitura do MS5611 e `Peabiru_UpdateBaro` | 20 Hz |
| Linha de log no CSV (uma por amostra do IMU), gravada em bloco no cartão | a cada 250 ms |
| Telemetria legível pela UART | 5 Hz |
| Linha `R,...` para o simulador ao vivo (opcional, `LIVE_ROTACAO`) | 25 Hz |
| BME68X (só nas fases `PAD` e `LANDED`) | 1 Hz |

> **Antes de ligar a placa, o cartão SD precisa estar no slot.** Se não estiver, `HAL_SD_Init` falha e o programa para no `Error_Handler()` (a serial fica muda).

### Estimador de trajetória (`model.c` / `model.h`)

Funde **1 IMU + barômetro**, sem dependência de HAL (só `math.h`):

- **Atitude:** filtro de Mahony (quaternion), inicializado pela gravidade. O acelerômetro só corrige a atitude quando mede cerca de 1 g.
- **Vertical:** Kalman de 2 estados (altura e velocidade), com a aceleração como entrada e o barômetro como medida.
- **Horizontal:** dead-reckoning (integração da aceleração). **Deriva com o tempo** e só é confiável por poucos segundos.
- **Fases do voo:** `PAD → BOOST → COAST → DESCENT → LANDED`.

| Transição | Condição (valores padrão) |
|---|---|
| `PAD → BOOST` | `\|a\|` > 2 g por 50 ms |
| `BOOST → COAST` | `\|a\|` < 1 g por 50 ms |
| `COAST → DESCENT` | velocidade vertical estimada negativa por 150 ms |
| `DESCENT → LANDED` | `\|v\|` < 1 m/s e `\|a\|` ≈ 1 g (±0,15) por 3 s |

Na fase `PAD`, o modelo força a velocidade e a posição a zero para não acumular deriva com o satélite parado.

Parâmetros ajustáveis (campos de `Peabiru_t`, definidos em `Peabiru_Init`):

| Parâmetro | Padrão | O que faz |
|---|---|---|
| `g_local` | 9.786 m/s² | Gravidade local (padrão: Iacanga-SP) |
| `boost_thresh_g` / `boost_time_s` | 2.0 g / 0.05 s | Detecção de decolagem |
| `burnout_thresh_g` / `burnout_time_s` | 1.0 g / 0.05 s | Fim da propulsão |
| `apogee_time_s` | 0.15 s | Tempo de velocidade negativa para declarar o apogeu |
| `landed_time_s` | 3.0 s | Tempo parado para declarar o pouso |
| `sigma_accel` | 0.5 m/s² | Ruído do acelerômetro (maior = confia mais no barômetro) |
| `sigma_baro` | 1.0 m | Ruído do barômetro (maior = confia mais no acelerômetro) |
| `kp_pad` / `kp_flight` | 2.0 / 0.3 | Ganho do Mahony na rampa e em voo |

Unidades de entrada: aceleração em **g**, giroscópio em **°/s**, pressão em **Pa** e `dt` em **s**. O eixo Z do mundo aponta para cima e X/Y são arbitrários (sem magnetômetro, o yaw não tem referência absoluta).

### Log em cartão SD (`logger.c` / `logger.h`)

- Usa **FatFs** sobre o **SDMMC1** (4 bits). Cartão em **FAT32**.
- A cada partida, cria `LOG000.CSV`, `LOG001.CSV`, ... (o primeiro nome livre). Nada é sobrescrito.
- As linhas são acumuladas num buffer de 16 KB e gravadas em bloco a cada 250 ms, com `f_sync` em seguida.
- O DMA interno do SDMMC1 do STM32H7 só enxerga a **AXI SRAM** (`0x24000000`). Por isso o `FATFS`, o `FIL` e o buffer ficam em **endereços fixos** nessa região (`0x24000000` a `0x24007FFF`), sem alteração no linker script.
- `LOGGER_DEBUG` (em `logger.c`) imprime cada etapa da gravação pela UART. Deixe em `0` em uso normal.
- **Retire o cartão só com a placa desligada**, porque o arquivo não é fechado com `f_close`.

Colunas do CSV (uma linha por amostra do IMU):

| Coluna | Descrição |
|---|---|
| `t_ms` | Tempo desde o boot (ms) |
| `fase` | 0 = PAD, 1 = BOOST, 2 = COAST, 3 = DESCENT, 4 = LANDED |
| `ax_g`, `ay_g`, `az_g` | Aceleração medida (g) |
| `gx_dps`, `gy_dps`, `gz_dps` | Velocidade angular medida (°/s) |
| `press_pa` | Pressão do MS5611 (Pa) |
| `alt_m` | Altura filtrada acima do ponto de calibração (m) |
| `vel_ms` | Velocidade vertical (m/s). Fica em 0 na fase `PAD` |
| `acc_g` | Módulo da aceleração (g) |
| `tilt_deg` | Inclinação do eixo Z do corpo em relação à vertical (°) |
| `x_m`, `y_m`, `vx_ms`, `vy_ms` | Posição e velocidade horizontais (dead-reckoning, derivam). Ficam em 0 na fase `PAD` |
| `roll_deg`, `pitch_deg`, `yaw_deg` | Ângulos de Euler (convenção Rz·Ry·Rx). O yaw deriva |
| `acc_up_ms2` | Aceleração vertical sem a gravidade (m/s²) |
| `flight_time_s` | Tempo desde a decolagem (s) |

### Saída serial

```
SD ok: gravando em LOG000.CSV
Modelo calibrado. p0=949.09 hPa | escala acel=0.9995 | giro bias(rad/s)=0.0043 -0.0025 0.0012
[PAD] alt=0.04m vel=0.00m/s |a|=1.00g tilt=5.0deg x=0.0 y=0.0 max=0.0m
>>> FASE: PAD -> BOOST | t=0.05s |a|=2.08g vel=0.00m/s alt=1.23m
BME68X  -> Temp: 24.98C | Press: 946.28hPa | Hum: 57.27% | Gas: 4772930 Ohm
R,34210,-3.02,-4.11,-1.70,0,0.04,0.00
```

- `[FASE] ...`: telemetria a 5 Hz.
- `>>> FASE: A -> B`: aparece a cada mudança de fase.
- `R,<t_ms>,<roll>,<pitch>,<yaw>,<fase>,<altura_m>,<vel_ms>`: linha para o simulador ao vivo. Pode ser desligada com `LIVE_ROTACAO 0`, e **deve ser desligada em voo real**, porque bloqueia a UART por alguns ms.

### Configuração do microcontrolador

| Item | Valor |
|---|---|
| Fonte de clock | HSI (64 MHz) → PLL1 (÷4, ×60, ÷2) |
| SYSCLK | 480 MHz |
| HCLK / barramentos APB | 240 MHz / 120 MHz |
| Regulador | LDO, escala de tensão VOS0 |
| I2C4 | Modo padrão (100 kHz), filtro analógico ativado |
| UART4 | 115200 baud, 8N1, sem controle de fluxo |
| SDMMC1 | 4 bits, divisor de clock configurado no `.ioc` [confirmar o valor final] |
| ADC1 | 16 bits, canal 3 (inicializado, ainda não usado no `main.c`) |

## Limitações conhecidas

Observadas em testes de bancada. **Nenhuma foi validada com um voo real.**

- **Limiares de foguete.** Os limites de 2 g e 1 g foram pensados para foguete. Em outro tipo de voo, a fase `BOOST` pode nunca ser detectada e o estimador fica preso em `PAD`.
- **Posição horizontal e yaw derivam.** `x_m`, `y_m` e `yaw_deg` não têm correção externa.
- **Deriva do barômetro.** A altura variou cerca de 2 m em 30 s logo após ligar a placa. Calibre (reinicie) **perto do lançamento**.
- **Taxa efetiva de amostragem menor que 100 Hz** (cerca de 50 a 70 Hz nos testes). A leitura do MS5611 bloqueia o loop por uns 20 ms, e a do BME68X (no solo) por cerca de 150 ms a cada segundo. O `dt` é medido, então a integração não quebra, mas há menos amostras.
- **Escrita no SD bloqueia o loop** por alguns ms a cada gravação (a cada 250 ms).
- **Leituras corrompidas do MS5611.** Já apareceram pressões negativas, e o estimador só rejeita pressões abaixo de 10000 Pa.
- **Recuperação após o pouso.** Em dados sintéticos, a altura estimada afunda abaixo do solo e o `LANDED` demora uns 8 s depois do toque. Falta confirmar se acontece em voo real.

## Estrutura do projeto

O projeto foi gerado pelo STM32CubeMX 6.17.0, com o pacote STM32Cube FW_H7 V1.13.0 e toolchain **CMake**.

```
.
├── README.md
├── Cubesat_fegsat.ioc        # Configuração do CubeMX (pinos, clocks, periféricos)
├── CMakeLists.txt
├── STM32H743XX_FLASH.ld      # Linker script
├── Core/
│   ├── Inc/                  # main.h, model.h, logger.h, ms5611.h, bme68x_app.h, BMI323_STM32.h, ...
│   └── Src/                  # main.c, model.c, logger.c e drivers dos sensores
├── Drivers/                  # HAL / CMSIS (gerados pelo CubeMX)
└── ferramentas/              # Scripts Python (veja abaixo)
```

## Como compilar e gravar

1. Abra a pasta do projeto no **VS Code** com a extensão **STM32Cube** (ou em outra IDE com suporte a CMake).
2. Compile o projeto. Os arquivos `model.c` e `logger.c` precisam estar listados nas fontes do `CMakeLists.txt`.
3. Conecte a placa por **ST-Link** (SWD: PA13 e PA14) e grave o firmware.
4. **Insira o cartão SD** (FAT32) e ligue a placa.
5. Mantenha o satélite **parado** durante a calibração (cerca de 3 s). A serial mostra `Modelo calibrado...` e `SD ok...` quando termina.
6. Abra um terminal serial a **115200 baud**, na UART4 (PA0/PA1), para acompanhar as leituras.

Para alterar pinos, clocks ou periféricos, abra o `Cubesat_fegsat.ioc` no **STM32CubeMX** e gere o código de novo. O código dentro dos blocos `USER CODE BEGIN/END` é preservado.

## Ferramentas Python (`ferramentas/`)

Scripts para analisar os CSVs do cartão SD e acompanhar o satélite em tempo real. Dependências:

```
python -m pip install pandas plotly numpy matplotlib pyserial
```

| Script | O que faz |
|---|---|
| `plot_trajetoria.py` | Gráfico 3D da trajetória (X, Y e altura), colorido por fase, com início e apogeu marcados. Salva um HTML. |
| `simulador_rotacao.py` | Animação 3D de um cubo girando como o modelo estimou, com os gráficos de altura e velocidade sincronizados. Salva um HTML. |
| `simulador_ao_vivo.py` | Mesma ideia, **em tempo real** pela serial (cubo, altura, velocidade, altura máxima e fase). `--simular` roda sem a placa. |

Uso:

```
python plot_trajetoria.py LOG000.CSV
python simulador_rotacao.py LOG000.CSV
python simulador_ao_vivo.py
```

O `simulador_ao_vivo.py` precisa estar na mesma pasta do `simulador_rotacao.py`. **Feche o Serial Monitor do VS Code antes de usá-lo**, porque só um programa pode abrir a porta serial por vez. Os três scripts exigem o CSV no formato novo (com as colunas `roll_deg`, `pitch_deg`, `yaw_deg`, `x_m`, `y_m` e `alt_m`).

## Próximos passos

- [ ] Validar os limiares do estimador com um voo ou queda reais
- [ ] Validar a fórmula de compensação do MS5611 (MS5611 × MS5607)
- [ ] Tornar a leitura do MS5611 não bloqueante
- [ ] Avaliar o uso do BME68X como segundo barômetro (corrigindo o offset entre os sensores)
- [ ] [Comunicação com a estação de solo / outros subsistemas]
- [ ] [Armazenamento de dados, telemetria e demais itens]

## Equipe

Projeto desenvolvido pela **FegSat**, equipe de nanossatélites da Unesp Guaratinguetá.
