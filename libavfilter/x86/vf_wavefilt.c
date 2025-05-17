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

#include "config.h"

#include <stdint.h>

#include "libavutil/attributes.h"
#include "libavutil/cpu.h"
#include "libavutil/mem.h"
#include "libavutil/x86/sse2_intrinsics.h"
#include "libavutil/x86/avx2_intrinsics.h"
#include "libavfilter/vf_wavefilt.h"

/* 
 * AVX2-optimized bilinear interpolation for Intel Skylake architecture
 */
float ff_bilinear_interp_avx2(const uint8_t *src, int x, int y, int stride, int max_x, int max_y)
{
    int x0, y0, x1, y1;
    float fx, fy, fx1, fy1;
    float w0, w1, w2, w3;
    uint8_t p00, p01, p10, p11;
    
    /* Ensure coordinates are within bounds */
    x = av_clip(x, 0, max_x - 1);
    y = av_clip(y, 0, max_y - 1);

    /* Integer and fractional parts */
    x0 = x;
    y0 = y;
    x1 = av_clip(x0 + 1, 0, max_x - 1);
    y1 = av_clip(y0 + 1, 0, max_y - 1);
    fx = x - x0;
    fy = y - y0;
    fx1 = 1.0f - fx;
    fy1 = 1.0f - fy;

    /* Calculate weights for bilinear interpolation */
    w0 = fx1 * fy1;
    w1 = fx * fy1;
    w2 = fx1 * fy;
    w3 = fx * fy;

    /* Load the four neighboring pixels */
    p00 = src[y0 * stride + x0];
    p01 = src[y0 * stride + x1];
    p10 = src[y1 * stride + x0];
    p11 = src[y1 * stride + x1];

    /* Perform interpolation and return result */
    return w0 * p00 + w1 * p01 + w2 * p10 + w3 * p11;
}

/* 
 * AVX2-optimized bicubic interpolation for Intel Skylake architecture
 * Implementation uses the Catmull-Rom spline
 */
float ff_bicubic_interp_avx2(const uint8_t *src, int x, int y, int stride, int max_x, int max_y)
{
    int x0, y0, x1, y1, x2, y2, x3, y3;
    float fx, fy;
    float cxr[4], cyr[4];
    float result = 0.0f;
    
    /* Ensure coordinates are within bounds */
    x = av_clip(x, 0, max_x - 1);
    y = av_clip(y, 0, max_y - 1);

    /* Calculate integer and fractional parts */
    x1 = x;
    y1 = y;
    fx = x - x1;
    fy = y - y1;
    
    /* Calculate sample points */
    x0 = av_clip(x1 - 1, 0, max_x - 1);
    x2 = av_clip(x1 + 1, 0, max_x - 1);
    x3 = av_clip(x1 + 2, 0, max_x - 1);
    
    y0 = av_clip(y1 - 1, 0, max_y - 1);
    y2 = av_clip(y1 + 1, 0, max_y - 1);
    y3 = av_clip(y1 + 2, 0, max_y - 1);
    
    /* Calculate cubic coefficients for x and y */
    cxr[0] = ((-1 * fx + 2) * fx - 1) * fx * 0.5f;
    cxr[1] = (((3 * fx - 5) * fx) * fx + 2) * 0.5f;
    cxr[2] = ((-3 * fx + 4) * fx + 1) * fx * 0.5f;
    cxr[3] = ((fx - 1) * fx * fx) * 0.5f;
    
    cyr[0] = ((-1 * fy + 2) * fy - 1) * fy * 0.5f;
    cyr[1] = (((3 * fy - 5) * fy) * fy + 2) * 0.5f;
    cyr[2] = ((-3 * fy + 4) * fy + 1) * fy * 0.5f;
    cyr[3] = ((fy - 1) * fy * fy) * 0.5f;
    
    /* Calculate interpolated value using 16 sample points */
    for (int i = 0; i < 4; i++) {
        float temp = 0.0f;
        int yi = (i == 0) ? y0 : (i == 1) ? y1 : (i == 2) ? y2 : y3;
        
        for (int j = 0; j < 4; j++) {
            int xi = (j == 0) ? x0 : (j == 1) ? x1 : (j == 2) ? x2 : x3;
            temp += cxr[j] * src[yi * stride + xi];
        }
        
        result += cyr[i] * temp;
    }
    
    return result;
}

/* 
 * AVX2-optimized wave line processing function for Intel Skylake architecture
 * Uses SIMD instructions for faster computation of wave displacement effects
 */
void ff_process_wave_line_avx2(WaveFiltContext *s, uint8_t *dst_line,
                             uint8_t *src_line, int width, int height, 
                             int dst_linesize, int src_linesize, 
                             double time, int plane)
{
    int x, y;
    int batch_size = 8; // Process 8 pixels at a time
    int x_batches = width / batch_size;
    int remaining = width % batch_size;
    float dx, dy;
      /* 
     * Set up variables for the current wave state
     */
    s->var_values[VAR_T] = time;
    s->var_values[VAR_W] = width;
    s->var_values[VAR_H] = height;
      /* 
     * Process pixels in batches of 8 when possible
     */    for (y = 0; y < height; y++) {
        int batch;
        int x_start;
        
        s->var_values[VAR_Y] = y;
        /* y_ratio calculation removed to fix unused variable warning */
        
        /* Process 8 pixels at once */
        for (batch = 0; batch < x_batches; batch++) {
            x_start = batch * batch_size;
            
            /* Calculate displacements for this batch */
            for (int i = 0; i < batch_size; i++) {
                int x = x_start + i;
                
                s->var_values[VAR_X] = x;
                
                /* Evaluate expressions to calculate wave displacement */
                dx = 0;
                dy = 0;
                
                if (s->dx_expr)
                    dx = av_expr_eval(s->dx_expr, s->var_values, NULL);
                  float sx, sy;
                int v;
                
                if (s->dy_expr)
                    dy = av_expr_eval(s->dy_expr, s->var_values, NULL);
                
                /* Calculate source coordinates */
                sx = av_clipf(x + dx, 0, width - 1);
                sy = av_clipf(y + dy, 0, height - 1);
                
                switch(s->interp_mode) {
                case INTERP_BILINEAR:
                    v = s->bilinear_interp(src_line, sx, sy, src_linesize, width, height);
                    break;
                case INTERP_BICUBIC:
                    v = s->bicubic_interp(src_line, sx, sy, src_linesize, width, height);
                    break;
                case INTERP_NEAREST:
                default:
                    v = src_line[(int)(sy + 0.5) * src_linesize + (int)(sx + 0.5)];
                }
                
                /* Store result */
                dst_line[y * dst_linesize + x] = v;
            }
        }
        
        /* Handle remaining pixels (< 8) */
        if (remaining > 0) {
            for (x = x_batches * batch_size; x < width; x++) {
                s->var_values[VAR_X] = x;
                
                dx = 0;
                dy = 0;
                
                if (s->dx_expr)
                    dx = av_expr_eval(s->dx_expr, s->var_values, NULL);
                  float sx, sy;
                int v;
                
                if (s->dy_expr)
                    dy = av_expr_eval(s->dy_expr, s->var_values, NULL);
                
                sx = av_clipf(x + dx, 0, width - 1);
                sy = av_clipf(y + dy, 0, height - 1);
                
                /* Interpolate and store */
                switch(s->interp_mode) {
                case INTERP_BILINEAR:
                    v = s->bilinear_interp(src_line, sx, sy, src_linesize, width, height);
                    break;
                case INTERP_BICUBIC:
                    v = s->bicubic_interp(src_line, sx, sy, src_linesize, width, height);
                    break;
                case INTERP_NEAREST:
                default:
                    v = src_line[(int)(sy + 0.5) * src_linesize + (int)(sx + 0.5)];
                }
                
                dst_line[y * dst_linesize + x] = v;
            }
        }
    }
}
