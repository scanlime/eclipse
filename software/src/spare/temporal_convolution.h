/*
 * Convolution kernel in the time dimension, using data from an EffectTap.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include <stdio.h>
#include "lib/effect.h"
#include "lib/effect_tap.h"


class TemporalConvolution : public Effect {
public:
    // Vector of (time, gain) pairs
    typedef std::vector<std::pair<float, float> > kernel_t;

    TemporalConvolution();
    void setTap(const EffectTap *tap);
    void setKernel(const kernel_t& newKernel, float gain = 1.0f);
    void setGaussian(unsigned numSamples, float stdDeviation, float mean = 0.0f, float gain = 1.0f);

    virtual void beginFrame(const FrameInfo& f);
    virtual void shader(Vec3& rgb, const PixelInfo& p) const;
    virtual void debug(const DebugInfo &di);

private:
    const EffectTap *tap;
    kernel_t kernel;
    std::vector<const EffectTap::Frame*> frames;
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline TemporalConvolution::TemporalConvolution()
    : tap(0)
{}

inline void TemporalConvolution::setTap(const EffectTap *tap)
{
    this->tap = tap;
}

inline void TemporalConvolution::setKernel(const kernel_t& newKernel, float gain)
{
    float total = 0;
    for (unsigned i = 0; i < newKernel.size(); ++i) {
        total += newKernel[i].second;
    }

    float scale = gain / total;
    kernel.resize(newKernel.size());
    for (unsigned i = 0; i < newKernel.size(); ++i) {
        kernel[i] = newKernel[i];
        kernel[i].second *= scale;
    }
}

inline void TemporalConvolution::setGaussian(unsigned numSamples, float stdDeviation, float mean, float gain)
{
    kernel_t k;

    // Limits of iteration
    float tMin = std::max(0.0f, mean - stdDeviation * 3.0f);
    float tMax = std::max(0.0f, mean + stdDeviation * 3.0f);
    float tStep = (tMax - tMin) / numSamples;

    for (unsigned i = 0; i < numSamples; i++) {
        float t = tMin + i * tStep;
        k.push_back(std::pair<float, float>(
            t,
            exp(-sq(t - mean) / (2.0f * sq(stdDeviation)))
        ));
    }

    setKernel(k, gain);
}

inline void TemporalConvolution::beginFrame(const FrameInfo& f)
{
    // Begin frame cache
    
    frames.resize(kernel.size());
    for (unsigned i = 0; i < kernel.size(); i++) {
        frames[i] = tap ? tap->get(kernel[i].first) : 0;
    }
}

inline void TemporalConvolution::shader(Vec3& rgb, const PixelInfo& p) const
{
    Vec3 accumulator = Vec3(0,0,0);

    for (unsigned i = 0; i < kernel.size(); i++) {
        const EffectTap::Frame *f = frames[i];
        float v = kernel[i].second;
        if (f && v) {
            accumulator += v * f->colors[p.index];
        }
    }

    rgb = accumulator;
}

inline void TemporalConvolution::debug(const DebugInfo &di)
{
    fprintf(stderr, "\t[temporal-convolution] kernel =");

    for (unsigned i = 0; i < kernel.size(); i++) {
        fprintf(stderr, " (%.02f, %.03f)", kernel[i].first, kernel[i].second);
    }

    fprintf(stderr, "\n");
}
