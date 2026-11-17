#include "fir_filter.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI_F 3.14159265358979f

static float window_coef(fir_window_t w, int n, int N)
{
    float a = 2.0f * PI_F * (float)n / (float)(N - 1);
    switch (w) {
        case FIR_WIN_HANN:     return 0.5f  - 0.5f  * cosf(a);
        case FIR_WIN_HAMMING:  return 0.54f - 0.46f * cosf(a);
        case FIR_WIN_BLACKMAN: return 0.42f - 0.5f * cosf(a) + 0.08f * cosf(2.0f * a);
        default:               return 1.0f;
    }
}

bool fir_init_lowpass(fir_filter_t *f, int ntaps, float fc_hz, float fs_hz, fir_window_t win)
{
    if (!f || ntaps < 3 || ntaps > FIR_MAX_TAPS || (ntaps % 2) == 0) return false;
    if (fs_hz <= 0.0f || fc_hz <= 0.0f || fc_hz >= fs_hz / 2.0f)      return false;

    memset(f, 0, sizeof(*f));
    f->ntaps = (uint8_t)ntaps;

    const float wc = 2.0f * PI_F * fc_hz / fs_hz;   /* frecuencia de corte digital [rad/muestra] */
    const int   M  = (ntaps - 1) / 2;
    float sum = 0.0f;

    for (int n = 0; n < ntaps; n++) {
        int   m = n - M;
        float ideal = (m == 0) ? (wc / PI_F) : (sinf(wc * (float)m) / (PI_F * (float)m)); /* sinc */
        f->h[n] = ideal * window_coef(win, n, ntaps);
        sum += f->h[n];
    }
    for (int n = 0; n < ntaps; n++) f->h[n] /= sum;  /* ganancia DC = 1 exacta */

    return true;
}

void fir_reset(fir_filter_t *f, float value)
{
    for (int i = 0; i < f->ntaps; i++) f->x[i] = value;
    f->head = 0;
}

float fir_update(fir_filter_t *f, float in)
{
    const int N = f->ntaps;
    f->head = (uint8_t)((f->head + 1) % N);
    f->x[f->head] = in;

    float acc = 0.0f;
    int idx = f->head;
    for (int k = 0; k < N; k++) {
        acc += f->h[k] * f->x[idx];
        idx = (idx == 0) ? (N - 1) : (idx - 1);
    }
    return acc;
}

float fir_group_delay_ms(const fir_filter_t *f, float fs_hz)
{
    return 1000.0f * ((float)(f->ntaps - 1) * 0.5f) / fs_hz;
}

float fir_magnitude_db(const fir_filter_t *f, float f_hz, float fs_hz)
{
    float w = 2.0f * PI_F * f_hz / fs_hz, re = 0.0f, im = 0.0f;
    for (int k = 0; k < f->ntaps; k++) {
        re += f->h[k] * cosf(w * (float)k);
        im -= f->h[k] * sinf(w * (float)k);
    }
    return 20.0f * log10f(sqrtf(re * re + im * im) + 1e-12f);
}

void fir_print_coeffs(const fir_filter_t *f)
{
    printf("FIR %d coeficientes (orden %d):\n", f->ntaps, f->ntaps - 1);
    for (int k = 0; k < f->ntaps; k++) printf("  h[%d] = %.8f\n", k, f->h[k]);
}

/* ---------------- IIR 1er orden ---------------- */
bool iir1_init_lowpass(iir1_filter_t *f, float fc_hz, float fs_hz)
{
    if (!f || fc_hz <= 0.0f || fs_hz <= 0.0f || fc_hz >= fs_hz / 2.0f) return false;
    float dt = 1.0f / fs_hz, rc = 1.0f / (2.0f * PI_F * fc_hz);
    f->alpha = dt / (rc + dt);
    f->y = 0.0f;
    f->primed = false;
    return true;
}

float iir1_update(iir1_filter_t *f, float in)
{
    if (!f->primed) { f->y = in; f->primed = true; return in; }
    f->y += f->alpha * (in - f->y);
    return f->y;
}
