/**
 * model.h - Estimador de trajetoria do FEGSAT / PEABIRU (STM32H743)
 *
 * Fusao: 1 IMU + barometro
 *  - Atitude: filtro Mahony (quaternion), inicializado pela gravidade na rampa
 *  - Vertical: Kalman 2 estados [altura, velocidade], acel como entrada, baro como medida
 *  - Horizontal: dead-reckoning (DERIVA com o tempo! so confiavel por poucos segundos)
 *  - Fases de voo: PAD -> BOOST -> COAST -> DESCENT -> LANDED
 *
 *  - Escala do acelerometro calibrada na rampa (|a| parado vira exatamente 1 g)
 *  - Gravidade local configuravel (padrao Iacanga-SP, 9.786 m/s^2)
 *  - Altitude sobre o nivel do mar opcional (so telemetria, nao afeta o filtro)
 *
 * Unidades de entrada: acel [g], giro [dps], pressao [Pa], dt [s]
 * Eixo mundo: Z = para cima, X/Y = arbitrarios (sem magnetometro, yaw nao tem referencia)
 */
#ifndef MODEL_H
#define MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    PB_PHASE_PAD = 0,   /* parado na rampa */
    PB_PHASE_BOOST,     /* motor ligado */
    PB_PHASE_COAST,     /* subindo sem propulsao */
    PB_PHASE_DESCENT,   /* apos apogeu */
    PB_PHASE_LANDED
} Peabiru_Phase;

typedef struct {
    /* ---------- parametros (ajuste antes do voo) ---------- */
    float g_local;            /* gravidade local [m/s^2] (padrao 9.786 = Iacanga-SP) */
    float boost_thresh_g;     /* |a| acima disso => decolagem (padrao 2.0) */
    float boost_time_s;       /* tempo sustentado p/ confirmar (0.05) */
    float burnout_thresh_g;   /* |a| abaixo disso => fim do motor (1.0) */
    float burnout_time_s;     /* (0.05) */
    float apogee_time_s;      /* v<0 sustentada p/ declarar apogeu (0.15) */
    float landed_time_s;      /* parado por esse tempo => pousou (3.0) */
    float sigma_accel;        /* ruido do acel [m/s^2] (padrao 0.5) */
    float sigma_baro;         /* ruido do baro [m]     (padrao 1.0) */
    float kp_pad;             /* ganho Mahony na rampa (2.0) */
    float kp_flight;          /* ganho Mahony em voo   (0.3) */

    /* ---------- estado interno ---------- */
    float q0, q1, q2, q3;     /* quaternion corpo->mundo */
    float gbx, gby, gbz;      /* bias do giro [rad/s] */
    float accel_scale;        /* correcao de escala do acel (1/|a| parado) */
    float p0_pa;              /* pressao de referencia (solo) */
    float psl_pa;             /* pressao ao nivel do mar informada (0 = nao informada) */
    float pad_elev_m;         /* elevacao da rampa sobre o nivel do mar */
    uint8_t calibrated;

    float kh, kv;             /* Kalman: altura e vel. vertical */
    float P00, P01, P10, P11;

    float timer_boost, timer_burnout, timer_apogee, timer_landed;

    /* ---------- saidas ---------- */
    Peabiru_Phase phase;
    float alt_m;              /* altura filtrada acima do solo [m] */
    float alt_msl_m;          /* altitude sobre o nivel do mar [m] (se SetSeaLevelPressure) */
    float vel_up_ms;          /* velocidade vertical [m/s] */
    float alt_baro_m;         /* altura bruta do barometro [m] */
    float acc_up_ms2;         /* aceleracao vertical sem gravidade [m/s^2] */
    float acc_mag_g;          /* |a| total [g] */
    float x_m, y_m;           /* posicao horizontal (dead-reckoning) */
    float vx_ms, vy_ms;
    float roll_deg, pitch_deg, yaw_deg;
    float tilt_deg;           /* angulo do eixo Z do corpo em relacao a vertical */

    float max_alt_m, max_vel_ms, max_acc_g;
    float flight_time_s;      /* tempo desde a decolagem */
    float apogee_time_flight_s;

    /* acumuladores de calibracao */
    float cal_ax, cal_ay, cal_az, cal_gx, cal_gy, cal_gz, cal_p;
    uint32_t cal_n;
} Peabiru_t;

void Peabiru_Init(Peabiru_t *p);

/* Opcionais, chamar a qualquer momento */
void Peabiru_SetLocalGravity(Peabiru_t *p, float g_ms2);
void Peabiru_SetSeaLevelPressure(Peabiru_t *p, float psl_pa);  /* so p/ alt_msl_m */

/* Calibracao na rampa, rocket PARADO: chame ~200-500 vezes e depois Finish */
void Peabiru_CalibAddSample(Peabiru_t *p, float ax_g, float ay_g, float az_g,
                            float gx_dps, float gy_dps, float gz_dps, float press_pa);
uint8_t Peabiru_CalibFinish(Peabiru_t *p);

/* Chamar em taxa fixa (100-200 Hz) a cada leitura do IMU */
void Peabiru_UpdateIMU(Peabiru_t *p, float ax_g, float ay_g, float az_g,
                       float gx_dps, float gy_dps, float gz_dps, float dt);

/* Chamar sempre que chegar pressao nova do barometro (20-50 Hz) */
void Peabiru_UpdateBaro(Peabiru_t *p, float press_pa);

#ifdef __cplusplus
}
#endif
#endif 