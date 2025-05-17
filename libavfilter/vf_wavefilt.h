/*
 * Copyright (c) 2025 FFmpeg developers
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef AVFILTER_WAVEFILT_H
#define AVFILTER_WAVEFILT_H

#include <stdint.h>
#include "libavutil/pixfmt.h"
#include "libavutil/eval.h"
#include "avfilter.h"

/* Wave mode defines */
enum {
    MODE_SINE,
    MODE_CIRCULAR,
    MODE_RADIAL,
    MODE_SQUARE,
    MODE_CUSTOM,
    NB_MODES
};

/* Interpolation mode defines */
enum {
    INTERP_NEAREST,
    INTERP_BILINEAR,
    INTERP_BICUBIC,
    NB_INTERP_MODES
};

/* Variable index for expressions */
enum {
    VAR_X,
    VAR_Y,
    VAR_W,
    VAR_H,
    VAR_T,
    VAR_PX,
    VAR_PY,
    VAR_CX,
    VAR_CY,
    VAR_A,
    VAR_D,
    VAR_OW,
    VAR_OH,
    VARS_NB
};

/* Wave filter context structure */
typedef struct WaveFiltContext {
    const AVClass *class;
    
    /* Basic filter parameters */
    double amplitude_x;
    double amplitude_y;
    double frequency_x;
    double frequency_y;
    double phase_x;
    double phase_y;
    double speed;
    double decay;
    double center_x;
    double center_y;
    double time_base;
    
    /* Mode and options */
    int mode;
    int interp_mode;
    int mirror;
    
    /* Custom expressions */
    char *dx_expr_str;
    char *dy_expr_str;
    AVExpr *dx_expr;
    AVExpr *dy_expr;
    
    /* Internal data for processing */
    int depth;
    int nb_planes;
    int hsub, vsub;
    int linesize[4];
    int height[4];
    int width[4];
    double time;
    int64_t pts;
    double var_values[VARS_NB];
    
    /* SIMD function pointers */
    float (*bilinear_interp)(const uint8_t *src, int x, int y, int stride, int max_x, int max_y);
    float (*bicubic_interp)(const uint8_t *src, int x, int y, int stride, int max_x, int max_y);
} WaveFiltContext;

/* Function declarations for SIMD optimized versions */
float ff_bilinear_interp_avx2(const uint8_t *src, int x, int y, int stride, int max_x, int max_y);
float ff_bicubic_interp_avx2(const uint8_t *src, int x, int y, int stride, int max_x, int max_y);
void ff_process_wave_line_avx2(WaveFiltContext *s, uint8_t *dst_line, 
                            uint8_t *src_line, int width, int height, 
                            int dst_linesize, int src_linesize,
                            double time, int plane);

/* Initialize SIMD optimized functions */
void ff_wavefilt_init_x86(WaveFiltContext *s);

#endif /* AVFILTER_WAVEFILT_H */
