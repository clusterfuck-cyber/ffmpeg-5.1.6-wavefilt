# wavefilt - Advanced Waveform Filter for FFmpeg

## Overview

The `wavefilt` filter is an advanced video filter for FFmpeg that applies configurable wave displacement effects to video frames. Unlike the basic `waves` filter, it provides precise control over wave patterns, interpolation methods, and animation parameters.

## Features

- Multiple wave modes: sine, circular, radial, square, and custom
- Precise amplitude, frequency, and phase control for X and Y dimensions
- Three interpolation methods: nearest neighbor, bilinear, and bicubic
- Optional edge mirroring
- Custom wave equations through expression evaluation
- CPU-specific optimizations (x86 SIMD for AVX2)
- Skylake-specific optimizations for Intel i5-6600 and similar CPUs

## Requirements

- FFmpeg (compatible with versions 6.0 and above)
- For x86 SIMD optimizations:
  - Intel processor with AVX2 support (Haswell or newer)
  - For full Skylake optimizations: Intel Skylake or newer CPU

## Installation

The filter is included as part of FFmpeg. To build FFmpeg with this filter specifically enabled and optimized for Skylake:

```sh
./configure --enable-gpl --enable-libx264 --enable-alsa --enable-filter=wavefilt --cpu=skylake --extra-cflags='-march=skylake -mtune=skylake -O3' && make -j4
```

Or for a standard build that includes the filter:

```sh
./configure --enable-filter=wavefilt && make -j4
```

## Usage

### Basic Syntax

```sh
ffmpeg -i input.mp4 -vf "wavefilt=<options>" output.mp4
```

### Options

- `mode`: Wave pattern mode [0-4] (default: 0)
  - 0: Sine wave
  - 1: Circular wave
  - 2: Radial wave
  - 3: Square wave
  - 4: Custom wave (requires dx_expr and dy_expr)
  - *Min: 0, Max: 4*

- `amplitude_x`: Amplitude for X dimension 
  - *Default: 10.0, Min: 0.0, Max: 1000.0*
  - Controls the maximum displacement in pixels along the x-axis

- `amplitude_y`: Amplitude for Y dimension 
  - *Default: 10.0, Min: 0.0, Max: 1000.0*
  - Controls the maximum displacement in pixels along the y-axis

- `frequency_x`: Frequency for X dimension 
  - *Default: 0.1, Min: 0.0, Max: 10.0*
  - Higher values create more wave cycles across the width

- `frequency_y`: Frequency for Y dimension 
  - *Default: 0.1, Min: 0.0, Max: 10.0*
  - Higher values create more wave cycles across the height

- `phase_x`: Phase for X dimension 
  - *Default: 0.0, Min: -π (-3.14159), Max: π (3.14159)*
  - Controls the starting position of the wave pattern in x-axis

- `phase_y`: Phase for Y dimension 
  - *Default: 0.0, Min: -π (-3.14159), Max: π (3.14159)*
  - Controls the starting position of the wave pattern in y-axis

- `speed`: Animation speed 
  - *Default: 1.0, Min: -100.0, Max: 100.0*
  - Negative values reverse the direction of motion

- `decay`: Wave amplitude decay from center 
  - *Default: 0.0, Min: 0.0, Max: 10.0*
  - Higher values cause waves to diminish faster with distance from center

- `center_x`: X center coordinate for circular/radial waves 
  - *Default: 0.5, Min: 0.0, Max: 1.0*
  - Normalized coordinate (0.0 = left edge, 1.0 = right edge)

- `center_y`: Y center coordinate for circular/radial waves 
  - *Default: 0.5, Min: 0.0, Max: 1.0*
  - Normalized coordinate (0.0 = top edge, 1.0 = bottom edge)

- `interp_mode`: Interpolation method [0-2] 
  - *Default: 1 (Bilinear), Min: 0, Max: 2*
  - 0: Nearest neighbor (fastest, lowest quality)
  - 1: Bilinear (good balance of speed and quality)
  - 2: Bicubic (highest quality, most CPU intensive)

- `mirror`: Enable edge mirroring 
  - *Default: 1 (enabled), Min: 0, Max: 1*
  - When enabled, prevents black borders by mirroring pixels at image boundaries

- `dx_expr`: Custom X displacement expression 
  - *Default: null (no expression)*
  - AVExpr expression string to calculate x displacement when mode=4

- `dy_expr`: Custom Y displacement expression 
  - *Default: null (no expression)*
  - AVExpr expression string to calculate y displacement when mode=4

- `time_base`: Base time in seconds for animation speed 
  - *Default: 1.0, Min: 0.001, Max: 100.0*
  - Controls the time scale for the animation

### Variables for Custom Expressions

The following variables can be used in `dx_expr` and `dy_expr`:

- `x`, `y`: Current pixel coordinates
- `w`, `h`: Input width and height
- `t`: Current timestamp in seconds
- `cx`, `cy`: Center coordinates
- `a`: Amplitude (combined value)
- `d`: Distance from center (for circular/radial modes)

## Examples

### Simple Sine Wave

```sh
ffmpeg -i input.mp4 -vf "wavefilt=amplitude_x=20:amplitude_y=10:frequency_x=0.05:frequency_y=0.05:speed=0.5" output.mp4
```

### Circular Ripple Effect

```sh
ffmpeg -i input.mp4 -vf "wavefilt=mode=1:amplitude_x=15:frequency_x=0.2:speed=0.8:decay=0.1" output.mp4
```

### Custom Wave Pattern

```sh
ffmpeg -i input.mp4 -vf "wavefilt=mode=4:dx_expr='sin(x/10+t*5)*10':dy_expr='cos(y/20+t*3)*15':interp_mode=2" output.mp4
```

### High-Quality Water Effect

```sh
ffmpeg -i input.mp4 -vf "wavefilt=mode=1:amplitude_x=8:amplitude_y=8:frequency_x=0.2:frequency_y=0.2:speed=0.3:interp_mode=2:mirror=1" output.mp4
```

## Performance Notes

- The filter includes AVX2-optimized code for bilinear and bicubic interpolation that significantly improves performance on compatible CPUs
- Specific optimizations for Intel Skylake architecture through the `--cpu=skylake` option and appropriate compiler flags
- Bicubic interpolation (interp_mode=2) provides the highest quality but is more CPU-intensive
- For real-time processing, consider using nearest neighbor (interp_mode=0) or bilinear (interp_mode=1) interpolation
- At high amplitude values (>100), you may need to adjust the `mirror` parameter to prevent artifacts at image edges

## Technical Implementation

The filter performs displacement mapping by calculating wave patterns and applying them to the source image coordinates. Key components:

1. Wave pattern generators for different modes (sine, circular, etc.)
2. Pixel coordinate transformation based on displacement values
3. Various interpolation methods for sampling source pixels
4. AVX2-optimized implementations of interpolation functions
5. Expression evaluation system for custom wave patterns

## License

GPLv2+/LGPLv2.1+, like the rest of FFmpeg.

## Author

Copyright (c) 2025 FFmpeg developers
