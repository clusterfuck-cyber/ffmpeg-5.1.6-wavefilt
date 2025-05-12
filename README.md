# WaveFilt - Advanced Wave Displacement Filter for FFmpeg

`wavefilt` is a versatile video filter for FFmpeg that applies various wave displacement effects to video frames with advanced controls and customization options.

## Overview

Unlike the basic `waves` filter included in FFmpeg, `wavefilt` provides more comprehensive displacement options including:

- Multiple wave modes (sine, circular, radial, square, and custom)
- Independent X and Y wave controls
- Center-based effects with customizable decay
- Multiple interpolation methods
- Custom wave expressions using libavutil's expression evaluator

## Installation

The filter is designed to be integrated into the FFmpeg build system. Ensure you have properly set up the FFmpeg development environment:

1. Clone FFmpeg from git: `git clone https://git.ffmpeg.org/ffmpeg.git`
2. Place `wavefilt.c` in the `libavfilter/` directory
3. Register the filter:
   - Add `extern const AVFilter ff_vf_wavefilt;` to `libavfilter/allfilters.c`
   - Add `&ff_vf_wavefilt,` to `libavfilter/filter_list.c`
   - Add `REGISTER_FILTER(WAVEFILT, wavefilt, vf)` to `libavfilter/Makefile`
4. Configure and build FFmpeg: `./configure && make`

## Usage

```
ffmpeg -i input.mp4 -vf "wavefilt=options" output.mp4
```

### Parameters

| Parameter     | Description                                   | Default | Range        |
|---------------|-----------------------------------------------|---------|--------------|
| amplitude_x   | Amplitude of X displacement                   | 10.0    | 0.0 to 1000.0|
| amplitude_y   | Amplitude of Y displacement                   | 10.0    | 0.0 to 1000.0|
| frequency_x   | Frequency of waves in X direction             | 0.1     | 0.0 to 10.0  |
| frequency_y   | Frequency of waves in Y direction             | 0.1     | 0.0 to 10.0  |
| phase_x       | Phase shift in X direction                    | 0.0     | -π to π      |
| phase_y       | Phase shift in Y direction                    | 0.0     | -π to π      |
| speed         | Wave movement speed                           | 1.0     | -100 to 100  |
| mode          | Wave mode (sine/circular/radial/square/custom)| sine    | -            |
| custom_x      | Custom expression for X displacement          | NULL    | -            |
| custom_y      | Custom expression for Y displacement          | NULL    | -            |
| decay         | Decay factor for wave amplitude from center   | 0.0     | 0.0 to 10.0  |
| center_x      | X coordinate of wave center (0.0-1.0)         | 0.5     | 0.0 to 1.0   |
| center_y      | Y coordinate of wave center (0.0-1.0)         | 0.5     | 0.0 to 1.0   |
| distort_mode  | Distortion algorithm (bilinear/nearest/bicubic)| bilinear| -           |
| mirror        | Mirror pixels at image boundaries             | 1 (true)| 0 or 1       |
| time_base     | Base time (seconds) for animation speed       | 1.0     | 0.001 to 100 |

### Wave Modes

1. **sine**: Creates sinusoidal waves based on pixel position and time
2. **circular**: Creates circular wave patterns from the center point
3. **radial**: Creates waves that emanate radially from the center
4. **square**: Creates square wave patterns (hard transitions)
5. **custom**: Uses custom expressions for X and Y displacement

### Custom Expressions

When using `mode=custom`, you must provide both `custom_x` and `custom_y` expressions. These expressions can use the following variables:

| Variable | Description                                      |
|----------|--------------------------------------------------|
| x        | Normalized pixel X position (0.0 to 1.0)         |
| y        | Normalized pixel Y position (0.0 to 1.0)         |
| w        | Frame width                                      |
| h        | Frame height                                     |
| cx       | Center X position                                |
| cy       | Center Y position                                |
| t        | Current time (in seconds, scaled by time_base)   |
| px       | X position relative to center (normalized)       |
| py       | Y position relative to center (normalized)       |
| a        | Current amplitude value                          |
| d        | Distance from center point                       |
| ow       | Plane width                                      |
| oh       | Plane height                                     |

## Examples

### Simple Horizontal Wave
```
ffmpeg -i input.mp4 -vf "wavefilt=mode=sine:amplitude_x=20:frequency_x=0.2:amplitude_y=0" output.mp4
```

### Circular Waves from Center
```
ffmpeg -i input.mp4 -vf "wavefilt=mode=circular:amplitude_x=15:amplitude_y=15:frequency_x=0.2:speed=0.5" output.mp4
```

### Radial Effect with Decay
```
ffmpeg -i input.mp4 -vf "wavefilt=mode=radial:center_x=0.5:center_y=0.5:decay=2:amplitude_x=30" output.mp4
```

### Square Wave Pattern
```
ffmpeg -i input.mp4 -vf "wavefilt=mode=square:frequency_x=0.1:frequency_y=0.05" output.mp4
```

### Custom Wave Expression
```
ffmpeg -i input.mp4 -vf "wavefilt=mode=custom:custom_x='sin(2*PI*x*5+t)*20*d':custom_y='cos(2*PI*y*5+t)*20*d'" output.mp4
```

### Combining with Other Filters
```
ffmpeg -i input.mp4 -vf "colorchannelmixer=.393:.769:.189:0:.349:.686:.168:0:.272:.534:.131,wavefilt=mode=circular:amplitude_x=10:amplitude_y=10:frequency_x=0.3:speed=0.4" sepia_waves.mp4
```

## Technical Details

The filter uses fixed-point arithmetic for interpolation and offers three interpolation modes:

- **bilinear**: Smoother result with good performance
- **nearest**: Faster but pixelated result
- **bicubic**: Highest quality but more computationally intensive

For video with chroma subsampling (4:2:0, 4:2:2), the filter automatically scales displacement values for chroma planes.

## Performance Considerations

- Higher amplitude and frequency values increase visible distortion
- Bicubic interpolation provides better quality but is significantly slower
- For real-time applications, use bilinear or nearest neighbor interpolation
- Processing speed depends on resolution and number of planes

## License

This filter is part of FFmpeg and is licensed under the LGPL version 2.1 or later.

## Author

Not sure if Copilot invented this or if used to be part of FFmpeg.
FFmpeg developers - Copyright (c) 2025

## See Also

- [FFmpeg Documentation](https://ffmpeg.org/documentation.html)
- [FFmpeg Filters Documentation](https://ffmpeg.org/ffmpeg-filters.html)
- Related filters: waves, displace
