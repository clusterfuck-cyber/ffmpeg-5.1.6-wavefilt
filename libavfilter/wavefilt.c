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

/**
 * @file
 * wavefilt video filter - advanced displacement wave effects
 * This filter provides more control and features compared to the waves filter
 */

#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/eval.h"
#include "libavutil/time.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"
#include "filters.h"

// Variable enumeration for expressions
enum var_name {
    VAR_X,
    VAR_Y,
    VAR_W,
    VAR_H,
    VAR_CX,
    VAR_CY,
    VAR_T,
    VAR_PX,
    VAR_PY,
    VAR_A,
    VAR_D,
    VAR_OW,
    VAR_OH,
    VAR_VARS_NB
};

typedef enum {
    MODE_SINE,
    MODE_CIRCULAR,
    MODE_RADIAL,
    MODE_SQUARE,
    MODE_CUSTOM,
    NB_MODES
} WaveMode;

typedef enum {
    DISTORT_BILINEAR,
    DISTORT_NEAREST,
    DISTORT_BICUBIC,
    NB_DISTORT_MODES
} DistortMode;

typedef struct WaveFiltContext {
    const AVClass *class;
    
    // Wave parameters
    double amplitude_x;       // amplitude of x displacement
    double amplitude_y;       // amplitude of y displacement
    double frequency_x;       // frequency of waves in x direction
    double frequency_y;       // frequency of waves in y direction
    double phase_x;           // phase shift in x direction
    double phase_y;           // phase shift in y direction
    double speed;             // wave movement speed
    int mode;                 // wave mode
    char *custom_expr_x;      // custom expression for x displacement
    char *custom_expr_y;      // custom expression for y displacement
    AVExpr *custom_x;         // parsed expression for x
    AVExpr *custom_y;         // parsed expression for y
    
    // Advanced options
    double decay;             // decay factor for wave amplitude from center
    double center_x;          // x coordinate of wave center (0.0-1.0)
    double center_y;          // y coordinate of wave center (0.0-1.0)
    int distort_mode;         // distortion algorithm
    int mirror;               // enable mirroring for out-of-bounds pixels
    double time_base;         // time base for animation
    
    // Internal variables
    int64_t pts;              // current pts
    int hsub, vsub;           // chroma subsampling
    int depth;                // bit depth
    int nb_planes;            // number of planes
    int linesize[4];          // line sizes
    int height[4];            // plane heights
    int width[4];             // plane widths
    double var_values[VAR_VARS_NB];
    double time;              // current time in seconds
} WaveFiltContext;

#define OFFSET(x) offsetof(WaveFiltContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption wavefilt_options[] = {
    { "amplitude_x", "Set the amplitude of the x displacement", OFFSET(amplitude_x), AV_OPT_TYPE_DOUBLE, {.dbl = 10.0}, 0.0, 1000.0, FLAGS },
    { "amplitude_y", "Set the amplitude of the y displacement", OFFSET(amplitude_y), AV_OPT_TYPE_DOUBLE, {.dbl = 10.0}, 0.0, 1000.0, FLAGS },
    { "frequency_x", "Set the frequency of the waves in x direction", OFFSET(frequency_x), AV_OPT_TYPE_DOUBLE, {.dbl = 0.1}, 0.0, 10.0, FLAGS },
    { "frequency_y", "Set the frequency of the waves in y direction", OFFSET(frequency_y), AV_OPT_TYPE_DOUBLE, {.dbl = 0.1}, 0.0, 10.0, FLAGS },
    { "phase_x", "Set the phase shift of the waves in x direction", OFFSET(phase_x), AV_OPT_TYPE_DOUBLE, {.dbl = 0.0}, -M_PI, M_PI, FLAGS },
    { "phase_y", "Set the phase shift of the waves in y direction", OFFSET(phase_y), AV_OPT_TYPE_DOUBLE, {.dbl = 0.0}, -M_PI, M_PI, FLAGS },
    { "speed", "Set the speed of wave movement", OFFSET(speed), AV_OPT_TYPE_DOUBLE, {.dbl = 1.0}, -100.0, 100.0, FLAGS },
    { "mode", "Set wave mode", OFFSET(mode), AV_OPT_TYPE_INT, {.i64 = MODE_SINE}, 0, NB_MODES-1, FLAGS, "mode" },
    { "sine", "Sinusoidal waves", 0, AV_OPT_TYPE_CONST, {.i64 = MODE_SINE}, 0, 0, FLAGS, "mode" },
    { "circular", "Circular waves", 0, AV_OPT_TYPE_CONST, {.i64 = MODE_CIRCULAR}, 0, 0, FLAGS, "mode" },
    { "radial", "Radial waves", 0, AV_OPT_TYPE_CONST, {.i64 = MODE_RADIAL}, 0, 0, FLAGS, "mode" },
    { "square", "Square waves", 0, AV_OPT_TYPE_CONST, {.i64 = MODE_SQUARE}, 0, 0, FLAGS, "mode" },
    { "custom", "Custom wave formula", 0, AV_OPT_TYPE_CONST, {.i64 = MODE_CUSTOM}, 0, 0, FLAGS, "mode" },
    { "custom_x", "Custom expression for x displacement", OFFSET(custom_expr_x), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, FLAGS },
    { "custom_y", "Custom expression for y displacement", OFFSET(custom_expr_y), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, FLAGS },
    { "decay", "Set the decay factor from center", OFFSET(decay), AV_OPT_TYPE_DOUBLE, {.dbl = 0.0}, 0.0, 10.0, FLAGS },
    { "center_x", "Set the x coordinate of wave center (0.0-1.0)", OFFSET(center_x), AV_OPT_TYPE_DOUBLE, {.dbl = 0.5}, 0.0, 1.0, FLAGS },
    { "center_y", "Set the y coordinate of wave center (0.0-1.0)", OFFSET(center_y), AV_OPT_TYPE_DOUBLE, {.dbl = 0.5}, 0.0, 1.0, FLAGS },
    { "distort_mode", "Set distortion interpolation mode", OFFSET(distort_mode), AV_OPT_TYPE_INT, {.i64 = DISTORT_BILINEAR}, 0, NB_DISTORT_MODES-1, FLAGS, "distort" },
    { "bilinear", "Bilinear interpolation", 0, AV_OPT_TYPE_CONST, {.i64 = DISTORT_BILINEAR}, 0, 0, FLAGS, "distort" },
    { "nearest", "Nearest neighbor interpolation", 0, AV_OPT_TYPE_CONST, {.i64 = DISTORT_NEAREST}, 0, 0, FLAGS, "distort" },
    { "bicubic", "Bicubic interpolation", 0, AV_OPT_TYPE_CONST, {.i64 = DISTORT_BICUBIC}, 0, 0, FLAGS, "distort" },
    { "mirror", "Mirror pixels at image boundaries", OFFSET(mirror), AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, FLAGS },
    { "time_base", "Set base time (in seconds) for animation speed", OFFSET(time_base), AV_OPT_TYPE_DOUBLE, {.dbl = 1.0}, 0.001, 100.0, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(wavefilt);

// Forward declarations
static int query_formats(AVFilterContext *ctx);
static int config_props(AVFilterLink *inlink);
static int filter_frame(AVFilterLink *inlink, AVFrame *in);

static av_cold int init(AVFilterContext *ctx)
{
    WaveFiltContext *s = ctx->priv;
    int ret;
    static const char *const var_names[] = {
        "x", "y", "w", "h", "cx", "cy", "t",
        "px", "py", "a", "d", "ow", "oh",
        NULL
    };

    // Register formats
    if ((ret = query_formats(ctx)) < 0)
        return ret;
    
    av_log(ctx, AV_LOG_INFO, "WaveFilt: Initialized with 1 input and 1 output\n");

    if (s->mode == MODE_CUSTOM) {
        if (!s->custom_expr_x || !s->custom_expr_y) {
            av_log(ctx, AV_LOG_ERROR, "Custom mode requires both custom_x and custom_y expressions\n");
            return AVERROR(EINVAL);
        }
        
        if ((ret = av_expr_parse(&s->custom_x, s->custom_expr_x, var_names, NULL, NULL, NULL, NULL, 0, ctx)) < 0) {
            av_log(ctx, AV_LOG_ERROR, "Error parsing custom_x expression '%s'\n", s->custom_expr_x);
            return ret;
        }
        
        if ((ret = av_expr_parse(&s->custom_y, s->custom_expr_y, var_names, NULL, NULL, NULL, NULL, 0, ctx)) < 0) {
            av_log(ctx, AV_LOG_ERROR, "Error parsing custom_y expression '%s'\n", s->custom_expr_y);
            return ret;
        }    }
    
    s->time = 0;
    s->pts = AV_NOPTS_VALUE;  // Initialize pts with AV_NOPTS_VALUE
    
    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    WaveFiltContext *s = ctx->priv;
    
    av_expr_free(s->custom_x);
    av_expr_free(s->custom_y);
}

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pixel_fmts[] = {
        AV_PIX_FMT_YUVA444P, AV_PIX_FMT_YUV444P, AV_PIX_FMT_YUV422P,
        AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV411P, AV_PIX_FMT_YUV410P,
        AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_GRAY8,
        AV_PIX_FMT_YUVA444P16, AV_PIX_FMT_YUV444P16,
        AV_PIX_FMT_YUV422P16, AV_PIX_FMT_YUV420P16,
        AV_PIX_FMT_GRAY16,
        AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRP16,
        AV_PIX_FMT_GBRAP, AV_PIX_FMT_GBRAP16,
        AV_PIX_FMT_NONE
    };
    return ff_set_common_formats(ctx, ff_make_format_list(pixel_fmts));
}

static int config_props(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    WaveFiltContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int i;
    
    if (!desc) {
        av_log(ctx, AV_LOG_ERROR, "Invalid input format\n");
        return AVERROR(EINVAL);
    }
    
    s->hsub = desc->log2_chroma_w;
    s->vsub = desc->log2_chroma_h;
    s->depth = desc->comp[0].depth;
    s->nb_planes = av_pix_fmt_count_planes(inlink->format);
    
    // Store dimensions for each plane
    for (i = 0; i < 4; i++) {
        s->width[i] = inlink->w;
        s->height[i] = inlink->h;
        if (i == 1 || i == 2) {
            s->width[i] >>= s->hsub;
            s->height[i] >>= s->vsub;
        }
        s->linesize[i] = 0; // Will be set for each frame
    }
    
    // Initialize variables for wave calculations
    s->var_values[VAR_W]  = inlink->w;
    s->var_values[VAR_H]  = inlink->h;
    s->var_values[VAR_CX] = s->center_x * inlink->w;
    s->var_values[VAR_CY] = s->center_y * inlink->h;
    
    return 0;
}

// Bilinear interpolation
static float bilinear_interp(const uint8_t *src, int x, int y, int stride, int max_x, int max_y)
{
    int x0, y0, x1, y1;
    int mx, my;
    int v00, v01, v10, v11;
    int v0, v1;
    
    if (!src || stride <= 0 || max_x < 0 || max_y < 0)
        return 0;
        
    x0 = av_clip(x >> 16, 0, max_x);
    y0 = av_clip(y >> 16, 0, max_y);
    x1 = av_clip(x0 + 1, 0, max_x);
    y1 = av_clip(y0 + 1, 0, max_y);
    
    mx = x & 0xFFFF;
    my = y & 0xFFFF;
    
    v00 = src[y0 * stride + x0];
    v01 = src[y0 * stride + x1];
    v10 = src[y1 * stride + x0];
    v11 = src[y1 * stride + x1];
    
    v0 = (v00 * (0x10000 - mx) + v01 * mx) >> 16;
    v1 = (v10 * (0x10000 - mx) + v11 * mx) >> 16;
    
    return (v0 * (0x10000 - my) + v1 * my) >> 16;
}

// Bicubic interpolation helper function
static float cubic_hermite(float A, float B, float C, float D, float t)
{
    float a = -A/2.0f + (3.0f*B)/2.0f - (3.0f*C)/2.0f + D/2.0f;
    float b = A - (5.0f*B)/2.0f + 2.0f*C - D/2.0f;
    float c = -A/2.0f + C/2.0f;
    float d = B;
    
    return a*t*t*t + b*t*t + c*t + d;
}

// Bicubic interpolation
static float bicubic_interp(const uint8_t *src, int x, int y, int stride, int max_x, int max_y)
{
    int x0, y0;
    float tx, ty;
    float pixels[4][4] = {0};  // Initialize all pixels to 0
    float rows[4] = {0};       // Initialize all rows to 0
    int i, j;
    
    if (!src || stride <= 0 || max_x <= 0 || max_y <= 0)
        return 0;
        
    x0 = x >> 16;
    y0 = y >> 16;
    
    // Ensure we have enough data for bicubic (need 2 pixels on each side)
    if (x0 < 1 || x0 >= max_x - 2 || y0 < 1 || y0 >= max_y - 2)
        return bilinear_interp(src, x, y, stride, max_x, max_y); // Fall back to bilinear when too close to edge
    
    tx = (x & 0xFFFF) / 65536.0f;
    ty = (y & 0xFFFF) / 65536.0f;
    
    #define CLIP(x) av_clip((x), 0, max_x)
    #define CLIPP(x, y) src[av_clip((y), 0, max_y) * stride + av_clip((x), 0, max_x)]
    
    // Add stronger bounds check
    if (max_x <= 0 || max_y <= 0 || stride <= 0 || 
        x0 < 0 || x0 >= max_x || y0 < 0 || y0 >= max_y) {
        return 0; // Additional safety check
    }
    
    // Sample 16 pixels
    for (j = -1; j <= 2; j++) {
        for (i = -1; i <= 2; i++) {
            int safe_x = av_clip(x0 + i, 0, max_x);
            int safe_y = av_clip(y0 + j, 0, max_y);
            if (safe_y * stride + safe_x < 0 || safe_y * stride + safe_x >= (max_y + 1) * stride) {
                pixels[j+1][i+1] = 0; // Out of bounds
            } else {
                pixels[j+1][i+1] = src[safe_y * stride + safe_x];
            }
        }
    }
    
    // Interpolate rows
    for (j = 0; j < 4; j++) {
        rows[j] = cubic_hermite(pixels[j][0], pixels[j][1], pixels[j][2], pixels[j][3], tx);
    }
    
    // Interpolate result
    return cubic_hermite(rows[0], rows[1], rows[2], rows[3], ty);
    
    #undef CLIP
    #undef CLIPP
}

static double calculate_x_displacement(WaveFiltContext *s, int x, int y, int plane,
                                      int width, int height, double time)
{
    double dx = 0;
    double nx, ny, cx, cy;
    double dx_center, dy_center, distance, decay_factor;
    
    // Safety check for division by zero
    if (!s || width <= 0 || height <= 0) 
        return 0;
    
    // Safety check for coordinates
    if (x < 0 || x >= width || y < 0 || y >= height)
        return 0;
    
    nx = x / (double)width;
    ny = y / (double)height;
    cx = s->var_values[VAR_CX] / width;
    cy = s->var_values[VAR_CY] / height;
    
    // Calculate distance from center for decay
    dx_center = nx - cx;
    dy_center = ny - cy;
    distance = sqrt(dx_center*dx_center + dy_center*dy_center);
    decay_factor = s->decay > 0 ? exp(-distance * s->decay) : 1.0;
    
    switch (s->mode) {
        case MODE_SINE:
            dx = s->amplitude_x * sin(2 * M_PI * (s->frequency_y * ny + s->frequency_x * time + s->phase_x));
            break;
        case MODE_CIRCULAR:
            dx = s->amplitude_x * sin(2 * M_PI * (s->frequency_x * distance + time * s->speed + s->phase_x));
            break;
        case MODE_RADIAL:
            dx = s->amplitude_x * dx_center * sin(2 * M_PI * (s->frequency_x * distance + time * s->speed + s->phase_x));
            break;
        case MODE_SQUARE:
            dx = s->amplitude_x * ((sin(2 * M_PI * (s->frequency_y * ny + s->frequency_x * time + s->phase_x)) > 0) ? 1 : -1);
            break;
        case MODE_CUSTOM:
            s->var_values[VAR_X] = nx;
            s->var_values[VAR_Y] = ny;
            s->var_values[VAR_T] = time;
            s->var_values[VAR_PX] = nx - cx;
            s->var_values[VAR_PY] = ny - cy;
            s->var_values[VAR_A] = s->amplitude_x;
            s->var_values[VAR_D] = distance;
            s->var_values[VAR_OW] = width;
            s->var_values[VAR_OH] = height;
            dx = av_expr_eval(s->custom_x, s->var_values, NULL);
            break;
    }
    
    return dx * decay_factor * width;
}

static double calculate_y_displacement(WaveFiltContext *s, int x, int y, int plane,
                                      int width, int height, double time)
{
    double dy = 0;
    double nx, ny, cx, cy;
    double dx_center, dy_center, distance, decay_factor;
    
    // Safety check for division by zero
    if (!s || width <= 0 || height <= 0) 
        return 0;
    
    // Safety check for coordinates
    if (x < 0 || x >= width || y < 0 || y >= height)
        return 0;
    
    nx = x / (double)width;
    ny = y / (double)height;
    cx = s->var_values[VAR_CX] / width;
    cy = s->var_values[VAR_CY] / height;
    
    // Calculate distance from center for decay
    dx_center = nx - cx;
    dy_center = ny - cy;
    distance = sqrt(dx_center*dx_center + dy_center*dy_center);
    decay_factor = s->decay > 0 ? exp(-distance * s->decay) : 1.0;
    
    switch (s->mode) {
        case MODE_SINE:
            dy = s->amplitude_y * sin(2 * M_PI * (s->frequency_x * nx + s->frequency_y * time + s->phase_y));
            break;
        case MODE_CIRCULAR:
            dy = s->amplitude_y * sin(2 * M_PI * (s->frequency_y * distance + time * s->speed + s->phase_y));
            break;
        case MODE_RADIAL:
            dy = s->amplitude_y * dy_center * sin(2 * M_PI * (s->frequency_y * distance + time * s->speed + s->phase_y));
            break;
        case MODE_SQUARE:
            dy = s->amplitude_y * ((sin(2 * M_PI * (s->frequency_x * nx + s->frequency_y * time + s->phase_y)) > 0) ? 1 : -1);
            break;
        case MODE_CUSTOM:
            s->var_values[VAR_X] = nx;
            s->var_values[VAR_Y] = ny;
            s->var_values[VAR_T] = time;
            s->var_values[VAR_PX] = nx - cx;
            s->var_values[VAR_PY] = ny - cy;
            s->var_values[VAR_A] = s->amplitude_y;
            s->var_values[VAR_D] = distance;
            s->var_values[VAR_OW] = width;
            s->var_values[VAR_OH] = height;
            dy = av_expr_eval(s->custom_y, s->var_values, NULL);
            break;
    }
    
    return dy * decay_factor * height;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    WaveFiltContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out;
    int plane;
    
    // Safety checks
    if (!ctx || !s || !inlink || !outlink || !in) {
        av_log(ctx, AV_LOG_ERROR, "Null pointer in filter_frame\n");
        if (in)
            av_frame_free(&in);
        return AVERROR(EINVAL);
    }
    
    // Check dimensions
    if (in->width <= 0 || in->height <= 0) {
        av_log(ctx, AV_LOG_ERROR, "Invalid input dimensions: %dx%d\n", in->width, in->height);
        av_frame_free(&in);
        return AVERROR(EINVAL);
    }
    
    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        av_log(ctx, AV_LOG_ERROR, "Failed to allocate output frame\n");
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    
    av_frame_copy_props(out, in);
    
    // Update time value
    if (s->pts != AV_NOPTS_VALUE && in->pts != AV_NOPTS_VALUE) {
        s->time += (in->pts - s->pts) * av_q2d(inlink->time_base) / s->time_base;
    } else if (in->pts != AV_NOPTS_VALUE) {
        s->time = in->pts * av_q2d(inlink->time_base) / s->time_base;
    }
    s->pts = in->pts;
      for (plane = 0; plane < s->nb_planes; plane++) {        int width, height, linesize, dst_linesize; 
        const uint8_t *src;
        uint8_t *dst;
        double amplitude_scale_x, amplitude_scale_y;
        int x, y;
        
        // Skip plane if data is not available
        if (!in->data[plane] || !out->data[plane]) {
            av_log(ctx, AV_LOG_WARNING, "Skipping plane %d due to NULL data\n", plane);
            continue;
        }
        
        width = in->width >> ((plane == 1 || plane == 2) ? s->hsub : 0);
        height = in->height >> ((plane == 1 || plane == 2) ? s->vsub : 0);
        linesize = in->linesize[plane];
        dst_linesize = out->linesize[plane];
        
        // Skip if dimensions or linesize are invalid
        if (width <= 0 || height <= 0 || linesize <= 0 || dst_linesize <= 0) {
            av_log(ctx, AV_LOG_WARNING, "Skipping plane %d due to invalid dimensions\n", plane);
            continue;
        }
        
        src = in->data[plane];
        dst = out->data[plane];
        
        // Scale for subsampled planes
        amplitude_scale_x = (plane == 1 || plane == 2) ? 0.5 : 1.0;
        amplitude_scale_y = (plane == 1 || plane == 2) ? 0.5 : 1.0;
        
        for (y = 0; y < height; y++) {
            for (x = 0; x < width; x++) {                double dx, dy;
                int sx, sy;
                
                dx = calculate_x_displacement(s, x, y, plane, width, height, s->time) * amplitude_scale_x;
                dy = calculate_y_displacement(s, x, y, plane, width, height, s->time) * amplitude_scale_y;
                
                // Convert to fixed-point for interpolation
                sx = (int)((x << 16) - (dx * 65536));
                sy = (int)((y << 16) - (dy * 65536));                  // Handle out of bounds with a simple mirroring implementation
                if (s->mirror) {
                    int sxi = sx >> 16;
                    int syi = sy >> 16;
                    
                    // Simple bounds checking and mirroring
                    if (sxi < 0)
                        sxi = -sxi;
                    if (syi < 0)
                        syi = -syi;
                    
                    // Modulo-like operation for mirroring without division
                    while (sxi >= width * 2)
                        sxi -= width * 2;
                    
                    while (syi >= height * 2)
                        syi -= height * 2;
                    
                    if (sxi >= width)
                        sxi = (width * 2 - 1) - sxi;
                    if (syi >= height)
                        syi = (height * 2 - 1) - syi;
                    
                    // Put back into fixed-point format
                    sx = sxi << 16 | (sx & 0xFFFF);
                    sy = syi << 16 | (sy & 0xFFFF);
                }// Interpolate
                if ((sx >> 16) < 0 || (sx >> 16) >= width - 1 || (sy >> 16) < 0 || (sy >> 16) >= height - 1) {
                    // Out of bounds, use zero or edge value
                    if (s->mirror) {
                        int x0, y0;
                        x0 = av_clip(sx >> 16, 0, width - 1);
                        y0 = av_clip(sy >> 16, 0, height - 1);
                        
                        // Add safety check for line offsets
                        if (y0 * linesize + x0 >= 0 && y0 * linesize + x0 < height * linesize)
                            dst[y * dst_linesize + x] = src[y0 * linesize + x0];
                        else
                            dst[y * dst_linesize + x] = 0;
                    } else {
                        dst[y * dst_linesize + x] = 0;
                    }
                } else {
                    float v;
                      switch(s->distort_mode) {
                        case DISTORT_NEAREST:
                            // Add bounds check for NEAREST mode
                            {
                                int iy = sy >> 16;
                                int ix = sx >> 16;
                                if (iy >= 0 && iy < height && ix >= 0 && ix < width)
                                    dst[y * dst_linesize + x] = src[(iy * linesize) + ix];
                                else
                                    dst[y * dst_linesize + x] = 0;
                            }
                            break;
                        case DISTORT_BILINEAR:
                            v = bilinear_interp(src, sx, sy, linesize, width - 1, height - 1);
                            dst[y * dst_linesize + x] = av_clip_uint8(v);
                            break;
                        case DISTORT_BICUBIC:
                            v = bicubic_interp(src, sx, sy, linesize, width - 1, height - 1);
                            dst[y * dst_linesize + x] = av_clip_uint8(v);
                            break;
                        default:
                            // Add bounds check for default mode too
                            {
                                int iy = sy >> 16;
                                int ix = sx >> 16;
                                if (iy >= 0 && iy < height && ix >= 0 && ix < width)
                                    dst[y * dst_linesize + x] = src[(iy * linesize) + ix];
                                else
                                    dst[y * dst_linesize + x] = 0;
                            }
                            break;
                    }
                }
            }
        }
    }
    
    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

const AVFilter ff_vf_wavefilt = {
    .name          = "wavefilt",
    .description   = NULL_IF_CONFIG_SMALL("Apply wave effects with advanced controls."),
    .priv_size     = sizeof(WaveFiltContext),
    .priv_class    = &wavefilt_class,
    .init          = init,
    .uninit        = uninit,
    .inputs        = (const AVFilterPad[]) {
        {
            .name         = "default",
            .type         = AVMEDIA_TYPE_VIDEO,
            .filter_frame = filter_frame,
            .config_props = config_props,
        },
        { NULL }
    },
    .outputs       = (const AVFilterPad[]) {
        {
            .name = "default",
            .type = AVMEDIA_TYPE_VIDEO,
        },
        { NULL }
    },
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
    .process_command = NULL,
};
