/*
 * Three-dimensional pattern in C++ based on the "Rings" Processing example.
 *
 * This version samples colors from an image, rather than using the HSV colorspace.
 *
 * Uses noise functions modulated by sinusoidal rings, which themselves
 * wander and shift according to some noise functions.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include <math.h>
#include <time.h>
#include <stdlib.h>
#include "lib/prng.h"
#include "lib/color.h"
#include "lib/effect.h"
#include "lib/noise.h"
#include "lib/texture.h"
#include "lib/camera_flow.h"


class RingsEffect : public Effect
{
public:
    RingsEffect(CameraFlowAnalyzer& flow, const rapidjson::Value &config)
        : xyzSpeed(config["xyzSpeed"].GetDouble()),
          xyzScale(config["xyzScale"].GetDouble()),
          flowScale(config["flowScale"].GetDouble()),
          wSpeed(config["wSpeed"].GetDouble()),
          wRate(config["wRate"].GetDouble()),
          ringScale(config["ringScale"].GetDouble()),
          ringScaleRate(config["ringScaleRate"].GetDouble()),
          ringDepth(config["ringDepth"].GetDouble()),
          wanderSpeed(config["wanderSpeed"].GetDouble()),
          wanderSize(config["wanderSize"].GetDouble()),
          brightnessContrast(config["brightnessContrast"].GetDouble()),
          colorContrast(config["colorContrast"].GetDouble()),
          colorShiftRate(config["colorShiftRate"].GetDouble()),
          targetBrightness(config["targetBrightness"].GetDouble()),
          thresholdGain(config["thresholdGain"].GetDouble()),
          thresholdStepLimit(config["thresholdStepLimit"].GetDouble()),
          initialThreshold(config["initialThreshold"].GetDouble()),
          brightnessOctaves(config["brightnessOctaves"].GetUint()),
          colorOctaves(config["colorOctaves"].GetUint()),
          flow(flow),
          palette(config["palette"].GetString())
    {
        reseed(29);
    }

    float xyzSpeed;
    float xyzScale;
    float flowScale;
    float wSpeed;
    float wRate;
    float ringScale;
    float ringScaleRate;
    float ringDepth;
    float wanderSpeed;
    float wanderSize;
    float brightnessContrast;
    float colorContrast;
    float colorShiftRate;
    float targetBrightness;
    float thresholdGain;
    float thresholdStepLimit;
    float initialThreshold;
    unsigned brightnessOctaves;
    unsigned colorOctaves;
    
    CameraFlowCapture flow;

    // Sample colors along a curved path through a texture
    Texture palette;

    // State variables
    Vec4 d;
    float timer;
    float seed;
    float threshold;

    // Calculated once per frame
    float spacing;
    float colorParam;
    float pixelTotalNumerator;
    unsigned pixelTotalDenominator;
    bool is3D;
    Vec3 center;

    virtual void beginFrame(const FrameInfo &f)
    {
        timer += f.timeDelta;
        flow.capture();

        spacing = sq(0.5 + noise2(timer * ringScaleRate, 1.5)) * ringScale;

        // Rotate movement in the XZ plane
        float angle = noise2(timer * 0.01, seed + 30.5) * 10.0;
        float speed = pow(fabsf(noise2(timer * 0.01, seed + 40.5)), 2.5) * xyzSpeed;
        d[0] += cosf(angle) * speed * f.timeDelta;
        d[2] += sinf(angle) * speed * f.timeDelta;

        // Random wander along the W axis
        d[3] += noise2(timer * wRate, seed + 3.5) * wSpeed * f.timeDelta;

        // Update center position
        center = Vec3(noise2(timer * wanderSpeed, seed + 50.9),
                      noise2(timer * wanderSpeed, seed + 51.4),
                      noise2(timer * wanderSpeed, seed + 51.7)) * wanderSize;

        // Wander around the color palette
        colorParam = seed + timer * colorShiftRate;

        // Reset pixel total accumulators, used for the brightness calc in endFrame
        pixelTotalNumerator = 0;
        pixelTotalDenominator = 0;

        // Is this 2D or 3D?
        is3D = false;
        for (Effect::PixelInfoIter i = f.pixels.begin(), e = f.pixels.end(); i != e; ++i) {
            const Effect::PixelInfo &p = *i;
            if (p.point[1] != 0.0f) {
                is3D = true;
            }
        }
    }

    virtual void shader(Vec3& rgb, const PixelInfo &p) const
    {
        // Noise sampling location
        Vec4 s = Vec4(p.point * xyzScale + flow.model * flowScale, seed) + d;

        // Ring function, displaces the noise sampling coordinate
        float dist = len(p.point - center);
        Vec4 pulse = Vec4(sinf(d[2] + dist * spacing) * ringDepth, 0, 0, 0);

        /*
         * Brightness is calculated by:
         *
         *    n = (fbm_noise4(s + pulse, octaves) + threshold) * brightnessContrast;
         *
         * But if we can determine that n <= 0, we can exit early. Check this after
         * each fbm octave, to see if we can save another costly noise calculation.
         * Also, use 3D noise instead of 4D if the Y axis is unused.
         */

        float n = threshold * brightnessContrast;
        float amplitude = brightnessContrast;
        Vec4 arg = s + pulse;
        unsigned i = brightnessOctaves;

        while (true) {
            n += amplitude * dNoise(arg);
            --i;
            if (!(n > -amplitude * fbmTotal(i))) {
                // Too low for further octaves to bring back above 0.
                // On the last octave, note fbmTotal(0) == 0
                // Should also exit in case of NaN.
                return;
            }
            if (!i) {
                break;
            }

            amplitude *= 0.5f;
            arg *= 2.0f;
        }
        n /= fbmTotal(brightnessOctaves);

        /*
         * Another hybrid 2D/3D fbm for chroma. Use half the octaves.
         */

        float m = 0;
        amplitude = colorContrast;
        arg = s + Vec4(0, 0, 0, 10);
        i = colorOctaves;

        while (true) {
            m += amplitude * dNoise(arg);
            if (--i == 0) {
                break;
            }

            amplitude *= 0.5f;
            arg *= 2.0f;
        }
        m /= fbmTotal(colorOctaves);

        // Assemble color using a lookup through our palette
        rgb = color(colorParam + m, sq(n));
    }

    inline void postProcess(const Vec3& rgb, const PixelInfo& p)
    {
        // Keep a rough approximate brightness total, for closed-loop feedback
        for (unsigned i = 0; i < 3; i++) {
            pixelTotalNumerator += sq(std::min(1.0f, std::max(0.0f, rgb[i])));
            pixelTotalDenominator++;
        }
    }

    virtual void endFrame(const FrameInfo &f)
    {
        // Per-frame brightness calculations.
        // Adjust threshold in brightness-determining noise function, in order
        // to try and keep the average pixel brightness at a particular level.

        float target = targetBrightness;
        float current = pixelTotalDenominator ? pixelTotalNumerator / pixelTotalDenominator : 0.0f;

        // Rate limited servo loop.
        // Disabled if we aren't calculating pixel values.

        if (pixelTotalDenominator) {
            float step = (target - current) * thresholdGain;
            if (step > thresholdStepLimit) step = thresholdStepLimit;
            if (step < -thresholdStepLimit) step = -thresholdStepLimit;
            threshold += step;
        }
    }

    virtual void debug(const DebugInfo &di)
    {
        fprintf(stderr, "\t[rings] %s model\n", is3D ? "3D" : "2D"); 
        fprintf(stderr, "\t[rings] seed = %f\n", seed);
        fprintf(stderr, "\t[rings] timer = %f\n", timer);
        fprintf(stderr, "\t[rings] center = %f, %f, %f\n", center[0], center[1], center[2]);
        fprintf(stderr, "\t[rings] d = %f, %f, %f, %f\n", d[0], d[1], d[2], d[3]);
        fprintf(stderr, "\t[rings] threshold = %f\n", threshold);
    }

    // Totally reinitialize our state variables. We do this periodically
    // during normal operation, during blank periods.

    void reseed(unsigned seed)
    {
        flow.capture();
        flow.origin();

        PRNG prng;
        prng.seed(seed);
        this->seed = prng.uniform(0, 1024);

        // Starting point
        d = Vec4(0,0,0,0);
        timer = 0;

        // Initial threshold gives us time to fade in
        threshold = initialThreshold;
    }

private:

    // Sample a color from our palette, using a lissajous curve within an image texture

    Vec3 color(float parameter, float brightness) const
    {
        return palette.sample( sinf(parameter) * 0.5f + 0.5f,
                               sinf(parameter * 0.86f) * 0.5f + 0.5f) * brightness;
    }

    // Sample 3 or 4 dimensional noise. If (!is3D), we use 3 dimensional noise, ignoring the Y axis.

    float dNoise(Vec4 v) const
    {
        return is3D ? noise4(v) : noise3(v[0], v[2], v[3]);
    }

    // Normalization factor for fractional brownian motion with N octaves

    static float fbmTotal(int i)
    {
        float n = 0;
        float amp = 1.0f;
        while (i > 0) {
            n += amp;
            amp *= 0.5;
            i--;
        }
        return n;
    }
};
