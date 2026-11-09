/**
 * model.c - Estimador de trajetoria do FEGSAT / PEABIRU (1 IMU)
 * Sem dependencia de HAL. Usa apenas math.h (FPU do H743 em single precision).
 */
#include "model.h"
#include <math.h>
#include <string.h>

#define G_IACANGA   9.786f      /* ~22S, ~450 m */
#define DEG2RAD     0.01745329252f
#define RAD2DEG     57.2957795f

static float baro_alt(float press_pa, float ref_pa)
{
    /* formula barometrica internacional */
    return 44330.0f * (1.0f - powf(press_pa / ref_pa, 0.190295f));
}

static void update_msl(Peabiru_t *p)
{
    if (p->calibrated && p->psl_pa > 10000.0f)
        p->pad_elev_m = baro_alt(p->p0_pa, p->psl_pa);
}

void Peabiru_Init(Peabiru_t *p)
{
    memset(p, 0, sizeof(*p));

    p->g_local          = G_IACANGA;
    p->boost_thresh_g   = 2.0f;
    p->boost_time_s     = 0.05f;
    p->burnout_thresh_g = 1.0f;
    p->burnout_time_s   = 0.05f;
    p->apogee_time_s    = 0.15f;
    p->landed_time_s    = 3.0f;
    p->sigma_accel      = 0.5f;
    p->sigma_baro       = 1.0f;
    p->kp_pad           = 2.0f;
    p->kp_flight        = 0.3f;

    p->accel_scale = 1.0f;

    p->q0 = 1.0f;
    p->p0_pa = 101325.0f;
    p->P00 = 1.0f;
    p->P11 = 1.0f;
    p->phase = PB_PHASE_PAD;
}

void Peabiru_SetLocalGravity(Peabiru_t *p, float g_ms2)
{
    if (g_ms2 > 9.0f && g_ms2 < 10.5f) p->g_local = g_ms2;
}

void Peabiru_SetSeaLevelPressure(Peabiru_t *p, float psl_pa)
{
    p->psl_pa = psl_pa;
    update_msl(p);
}

void Peabiru_CalibAddSample(Peabiru_t *p, float ax, float ay, float az,
                            float gx, float gy, float gz, float press_pa)
{
    p->cal_ax += ax; p->cal_ay += ay; p->cal_az += az;
    p->cal_gx += gx; p->cal_gy += gy; p->cal_gz += gz;
    p->cal_p  += press_pa;
    p->cal_n++;
}

uint8_t Peabiru_CalibFinish(Peabiru_t *p)
{
    if (p->cal_n < 20) return 0;

    float n  = (float)p->cal_n;
    float ax = p->cal_ax / n, ay = p->cal_ay / n, az = p->cal_az / n;

    p->gbx = (p->cal_gx / n) * DEG2RAD;
    p->gby = (p->cal_gy / n) * DEG2RAD;
    p->gbz = (p->cal_gz / n) * DEG2RAD;
    p->p0_pa = p->cal_p / n;

    float m = sqrtf(ax*ax + ay*ay + az*az);
    if (m < 0.5f) return 0;                 /* acelerometro sem gravidade: algo errado */

    /* Escala: parado, |a| tem que ser exatamente 1 g. Isso absorve o erro de
       sensibilidade do sensor; a gravidade local fica em g_local. */
    p->accel_scale = 1.0f / m;

    /* quaternion inicial: gira o vetor "cima" medido ate o eixo Z do mundo */
    ax /= m; ay /= m; az /= m;

    float ux = ay, uy = -ax;                /* a x ez */
    float s  = sqrtf(ux*ux + uy*uy);
    if (s < 1e-5f) {
        if (az > 0.0f) { p->q0 = 1; p->q1 = 0; p->q2 = 0; p->q3 = 0; }
        else           { p->q0 = 0; p->q1 = 1; p->q2 = 0; p->q3 = 0; }  /* de cabeca p/ baixo */
    } else {
        float th = atan2f(s, az);
        float sh = sinf(0.5f * th) / s;
        p->q0 = cosf(0.5f * th);
        p->q1 = ux * sh;
        p->q2 = uy * sh;
        p->q3 = 0.0f;
    }

    p->kh = 0; p->kv = 0;
    p->P00 = 1; p->P01 = 0; p->P10 = 0; p->P11 = 1;
    p->x_m = p->y_m = p->vx_ms = p->vy_ms = 0;
    p->phase = PB_PHASE_PAD;
    p->calibrated = 1;
    update_msl(p);
    return 1;
}

void Peabiru_UpdateIMU(Peabiru_t *p, float ax, float ay, float az,
                       float gx_dps, float gy_dps, float gz_dps, float dt)
{
    if (!p->calibrated || dt <= 0.0f || dt > 0.5f) return;

    const float G = p->g_local;

    /* escala corrigida: parado, |a| = 1 g */
    ax *= p->accel_scale; ay *= p->accel_scale; az *= p->accel_scale;

    float gx = gx_dps * DEG2RAD - p->gbx;
    float gy = gy_dps * DEG2RAD - p->gby;
    float gz = gz_dps * DEG2RAD - p->gbz;

    float amag = sqrtf(ax*ax + ay*ay + az*az);
    p->acc_mag_g = amag;

    /* ---------------- Atitude: Mahony ---------------- */
    float q0 = p->q0, q1 = p->q1, q2 = p->q2, q3 = p->q3;

    /* so corrige com o acel quando ele realmente mede ~1g (sem propulsao/arrasto forte) */
    if (amag > 0.5f && fabsf(amag - 1.0f) < 0.1f) {
        float kp = (p->phase == PB_PHASE_PAD) ? p->kp_pad : p->kp_flight;
        float inv = 1.0f / amag;
        float nx = ax * inv, ny = ay * inv, nz = az * inv;

        float vx = 2.0f * (q1*q3 - q0*q2);
        float vy = 2.0f * (q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        gx += kp * (ny*vz - nz*vy);
        gy += kp * (nz*vx - nx*vz);
        gz += kp * (nx*vy - ny*vx);
    }

    float hdt = 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz) * hdt;
    q1 += ( qa*gx + qc*gz - q3*gy) * hdt;
    q2 += ( qa*gy - qb*gz + q3*gx) * hdt;
    q3 += ( qa*gz + qb*gy - qc*gx) * hdt;

    float qn = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= qn; q1 *= qn; q2 *= qn; q3 *= qn;
    p->q0 = q0; p->q1 = q1; p->q2 = q2; p->q3 = q3;

    /* Matriz de rotacao corpo->mundo */
    float r00 = 1.0f - 2.0f*(q2*q2 + q3*q3);
    float r01 = 2.0f*(q1*q2 - q0*q3);
    float r02 = 2.0f*(q1*q3 + q0*q2);
    float r10 = 2.0f*(q1*q2 + q0*q3);
    float r11 = 1.0f - 2.0f*(q1*q1 + q3*q3);
    float r12 = 2.0f*(q2*q3 - q0*q1);
    float r20 = 2.0f*(q1*q3 - q0*q2);
    float r21 = 2.0f*(q2*q3 + q0*q1);
    float r22 = 1.0f - 2.0f*(q1*q1 + q2*q2);

    /* Euler (so para telemetria) */
    p->roll_deg  = atan2f(r21, r22) * RAD2DEG;
    p->pitch_deg = -asinf(fmaxf(-1.0f, fminf(1.0f, r20))) * RAD2DEG;
    p->yaw_deg   = atan2f(r10, r00) * RAD2DEG;
    p->tilt_deg  = acosf(fmaxf(-1.0f, fminf(1.0f, r22))) * RAD2DEG;

    /* ---------------- Aceleracao no referencial do mundo ---------------- */
    float awx = (r00*ax + r01*ay + r02*az) * G;
    float awy = (r10*ax + r11*ay + r12*az) * G;
    float awz = (r20*ax + r21*ay + r22*az - 1.0f) * G;   /* tira a gravidade */
    p->acc_up_ms2 = awz;

    /* ---------------- Kalman vertical: predicao ---------------- */
    p->kh += p->kv * dt + 0.5f * awz * dt * dt;
    p->kv += awz * dt;

    float dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt2*dt2;
    float qs  = p->sigma_accel * p->sigma_accel;
    float P00 = p->P00, P01 = p->P01, P10 = p->P10, P11 = p->P11;
    p->P00 = P00 + dt*(P10 + P01) + dt2*P11 + qs*dt4*0.25f;
    p->P01 = P01 + dt*P11 + qs*dt3*0.5f;
    p->P10 = P10 + dt*P11 + qs*dt3*0.5f;
    p->P11 = P11 + qs*dt2;

    /* ---------------- Maquina de estados ---------------- */
    switch (p->phase) {
    case PB_PHASE_PAD:
        p->kv = 0.0f;                 /* parado: evita deriva da integracao */
        p->vx_ms = p->vy_ms = 0.0f;
        p->x_m = p->y_m = 0.0f;
        if (amag > p->boost_thresh_g) p->timer_boost += dt; else p->timer_boost = 0.0f;
        if (p->timer_boost >= p->boost_time_s) {
            p->phase = PB_PHASE_BOOST;
            p->flight_time_s = p->timer_boost;
        }
        break;

    case PB_PHASE_BOOST:
        p->flight_time_s += dt;
        if (amag < p->burnout_thresh_g) p->timer_burnout += dt; else p->timer_burnout = 0.0f;
        if (p->timer_burnout >= p->burnout_time_s) p->phase = PB_PHASE_COAST;
        break;

    case PB_PHASE_COAST:
        p->flight_time_s += dt;
        if (p->kv < 0.0f) p->timer_apogee += dt; else p->timer_apogee = 0.0f;
        if (p->timer_apogee >= p->apogee_time_s) {
            p->phase = PB_PHASE_DESCENT;
            p->apogee_time_flight_s = p->flight_time_s;
        }
        break;

    case PB_PHASE_DESCENT:
        p->flight_time_s += dt;
        if (fabsf(p->kv) < 1.0f && fabsf(amag - 1.0f) < 0.15f) p->timer_landed += dt;
        else p->timer_landed = 0.0f;
        if (p->timer_landed >= p->landed_time_s) p->phase = PB_PHASE_LANDED;
        break;

    case PB_PHASE_LANDED:
    default:
        p->kv = 0.0f;
        p->vx_ms = p->vy_ms = 0.0f;
        break;
    }

    /* ---------------- Horizontal: dead-reckoning (deriva!) ---------------- */
    if (p->phase == PB_PHASE_BOOST || p->phase == PB_PHASE_COAST || p->phase == PB_PHASE_DESCENT) {
        p->x_m  += p->vx_ms * dt + 0.5f * awx * dt2;
        p->y_m  += p->vy_ms * dt + 0.5f * awy * dt2;
        p->vx_ms += awx * dt;
        p->vy_ms += awy * dt;
    }

    /* ---------------- Saidas / maximos ---------------- */
    p->alt_m     = p->kh;
    p->alt_msl_m = p->pad_elev_m + p->kh;
    p->vel_up_ms = p->kv;
    if (p->kh  > p->max_alt_m)  p->max_alt_m  = p->kh;
    if (p->kv  > p->max_vel_ms) p->max_vel_ms = p->kv;
    if (amag   > p->max_acc_g)  p->max_acc_g  = amag;
}

void Peabiru_UpdateBaro(Peabiru_t *p, float press_pa)
{
    if (!p->calibrated || press_pa < 10000.0f) return;

    float z = baro_alt(press_pa, p->p0_pa);
    p->alt_baro_m = z;

    float R  = p->sigma_baro * p->sigma_baro;
    float S  = p->P00 + R;
    float K0 = p->P00 / S;
    float K1 = p->P10 / S;
    float y  = z - p->kh;

    p->kh += K0 * y;
    p->kv += K1 * y;

    float P00 = p->P00, P01 = p->P01, P10 = p->P10, P11 = p->P11;
    p->P00 = P00 - K0*P00;
    p->P01 = P01 - K0*P01;
    p->P10 = P10 - K1*P00;
    p->P11 = P11 - K1*P01;
}