/*
 * fir_design.c
 *
 *  Created on: Jun 3, 2025
 *      Author: User
 */

#include "fir_design.h"
#include <math.h>

fir_type_t fir_current_type = FIR_LOW; // indicates type of filter

float fir_coeffs[fir_N]; // fir coefficients

float fir_fc1 = 0.2; // lower cutoff frequency (only used in bandpass/bandstop)
float fir_fc2 = 0.4; // upper cutoff frequency (also fc in lpf, hpf)

void design_fir(void)
{
    float m, sinc_val, w;
    // Convert normalized freq from [0..1] to [0..0.5] (Nyquist freq)
    float fc1 = fir_fc1 / 2.0f;
    float fc2 = fir_fc2 / 2.0f;

    for (int i = 0; i < fir_N; i++)
    {
        m = i - fir_M / 2.0f;

        switch (fir_current_type)
        {
        case FIR_LOW:
            if (fabsf(m) < 1e-6f)
            {
                sinc_val = 2.0f * fc2;
            }
            else
            {
                sinc_val = sinf(2.0f * PI * fc2 * m) / (PI * m);
            }
            break;

        case FIR_HIGH:
            if (fabsf(m) < 1e-6f)
            {
                sinc_val = 1.0f - 2.0f * fc2;
            }
            else
            {
                sinc_val = -sinf(2.0f * PI * fc2 * m) / (PI * m);
            }
            break;

        case FIR_BANDPASS:
            if (fabsf(m) < 1e-6f)
            {
                sinc_val = 2.0f * (fc2 - fc1);
            }
            else
            {
                sinc_val = (sinf(2.0f * PI * fc2 * m) - sinf(2.0f * PI * fc1 * m)) / (PI * m);
            }
            break;

        case FIR_BANDSTOP:
            if (fabsf(m) < 1e-6f)
            {
                sinc_val = 1.0f - 2.0f * (fc2 - fc1);
            }
            else
            {
                sinc_val = (sinf(2.0f * PI * fc1 * m) - sinf(2.0f * PI * fc2 * m)) / (PI * m);
            }
            break;

        default:
            sinc_val = 0.0f;
            break;
        }

        // Periodic Hamming window (matches MATLAB)
        w = 0.54f - 0.46f * cosf(2.0f * PI * i / fir_M);

        fir_coeffs[i] = sinc_val * w;
    }
}
