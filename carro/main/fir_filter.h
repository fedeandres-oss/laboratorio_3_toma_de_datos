#ifndef FIR_FILTER_H
#define FIR_FILTER_H

/*
 * Libreria de filtros para ESP32 (float de 32 bits -> usa la FPU por hardware).
 *
 *  1) FIR pasa-bajos de fase lineal, diseno por ventana (equivale a fir1 de MATLAB)
 *  2) IIR de 1er orden (media movil exponencial), util para el lazo de control / encoder
 *
 * Retardo de grupo del FIR = (ntaps-1)/2 muestras  ->  (ntaps-1)/(2*fs) segundos.
 */

#include <stdint.h>
#include <stdbool.h>

#define FIR_MAX_TAPS 41   /* maximo de coeficientes (orden maximo = 40) */

typedef enum {
    FIR_WIN_RECT = 0,
    FIR_WIN_HANN,
    FIR_WIN_HAMMING,    /* recomendada: ~53 dB de atenuacion en banda de rechazo */
    FIR_WIN_BLACKMAN    /* ~74 dB pero transicion mas ancha */
} fir_window_t;

typedef struct {
    uint8_t ntaps;               /* numero de coeficientes (orden = ntaps-1) */
    uint8_t head;                /* posicion de la muestra mas reciente */
    float   h[FIR_MAX_TAPS];     /* coeficientes */
    float   x[FIR_MAX_TAPS];     /* buffer circular de entradas */
} fir_filter_t;

/* Disena un pasa-bajos. ntaps debe ser IMPAR (fase lineal tipo I, retardo entero).
 * Devuelve false si los parametros no son validos (fc >= fs/2, ntaps par, etc.) */
bool  fir_init_lowpass(fir_filter_t *f, int ntaps, float fc_hz, float fs_hz, fir_window_t win);

/* Llena el buffer con 'value' (evita el transitorio de arranque). Llamar con la 1a muestra. */
void  fir_reset(fir_filter_t *f, float value);

/* Entra una muestra, sale una muestra filtrada. Costo: ntaps multiplicaciones. */
float fir_update(fir_filter_t *f, float in);

/* Utilidades para verificar/sustentar el diseno */
float fir_group_delay_ms(const fir_filter_t *f, float fs_hz);
float fir_magnitude_db(const fir_filter_t *f, float f_hz, float fs_hz);
void  fir_print_coeffs(const fir_filter_t *f);   /* imprime h[] para compararlo con MATLAB */

/* ---------------- IIR de 1er orden: y[n] = y[n-1] + a*(x[n]-y[n-1]) ---------------- */
typedef struct {
    float alpha;
    float y;
    bool  primed;
} iir1_filter_t;

bool  iir1_init_lowpass(iir1_filter_t *f, float fc_hz, float fs_hz);
float iir1_update(iir1_filter_t *f, float in);

#endif /* FIR_FILTER_H */
