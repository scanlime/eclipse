/*
 * A tool to measure time.
 * Round trips, signals from LED to Camera.
 * For calibrating memory.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include <math.h>
#include "lib/camera.h"
#include "lib/effect.h"
#include "lib/effect_tap.h"


// Effect to use with the latency timer: a slow square wave.
class LatencyTimerEffect : public Effect
{
public:
    LatencyTimerEffect();

    static constexpr float hz = 2;
    float phase;

    virtual void beginFrame(const FrameInfo &f);
    virtual void shader(Vec3& rgb, const PixelInfo &p) const;
    static float brightnessAtPhase(float p);
};


class LatencyTimer
{
public:
    // Expected symmetric latency. I.e. if we shift video by this much, we'll have
    // symmetric rise and fall times. This turns out to be one NTSC video frame.
    static const float kExpectedDelay = 1.0 / 29.97;

    // Uses EffectTap to test the calibrated delay
    LatencyTimer(const EffectTap &tap);

    // Historgram for calculating camera transfer function and dumping out a CSV.
    void process(const Camera::VideoChunk &chunk);
    void debug();

    // Effect that generates a square wave with a known phase
    LatencyTimerEffect effect;

private:
    // Data type for brightness accumulators
    typedef int64_t total_t;

    // Fast total brightness estimator
    uint8_t frame[Camera::kPixels * 2];
    total_t frameTotal;

    // Filtered brightness average, binned by phase.
    // We want roughly one bin per expected LED frame.
    static const float kNominalMillisecondsPerBin = 1;
    static const int kNumPhaseBins = 1e3 / LatencyTimerEffect::hz /  kNominalMillisecondsPerBin;
    static const float kActualMillisecondsPerBin = 1e3 / LatencyTimerEffect::hz / kNumPhaseBins;
    total_t phaseBinNumerators[kNumPhaseBins];
    total_t phaseBinDenominators[kNumPhaseBins];

    // Testing our tap delay
    const EffectTap &tap;
    float ledTapBins[kNumPhaseBins];

    static float binToPhase(unsigned bin);
    static unsigned phaseToBin(float p);
    float ledBrightness(unsigned bin);
    float cameraBrightness(unsigned bin);
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline LatencyTimerEffect::LatencyTimerEffect()
    : phase(0)
{}

inline void LatencyTimerEffect::beginFrame(const FrameInfo &f)
{
    phase = fmodf(phase + f.timeDelta * hz, 1.0f);
}

inline void LatencyTimerEffect::shader(Vec3& rgb, const PixelInfo &p) const
{
    const float b = brightnessAtPhase(phase);
    rgb = Vec3(b,b,b);
}

inline float LatencyTimerEffect::brightnessAtPhase(float p)
{
    return p < 0.5 ? 0.8 : 0;
}

inline LatencyTimer::LatencyTimer(const EffectTap &tap)
    : tap(tap)
{
    frameTotal = 0;
    memset(frame, 0, sizeof frame);
    memset(phaseBinNumerators, 0, sizeof phaseBinNumerators);
    memset(phaseBinDenominators, 0, sizeof phaseBinDenominators);
}

inline void LatencyTimer::process(const Camera::VideoChunk &chunk)
{
    unsigned offset = chunk.framebufferOffset();
    total_t total = frameTotal;
    unsigned bin = phaseToBin(effect.phase);

    // New total for this frame
    for (unsigned i = 0; i != chunk.byteCount; i++) {
        uint8_t prev = frame[offset];
        uint8_t next = chunk.data[i];
        frame[offset] = next;
        offset++;

        // Accumulate the square
        total += total_t(next) * total_t(next) - total_t(prev) * total_t(prev);
    }
    frameTotal = total;

    // Accumulate into the current phase bin
    phaseBinNumerators[bin] += total;
    phaseBinDenominators[bin] += sizeof(frame) * 256 * 256;

    // Test the tap delay
    const EffectTap::Frame *tf = tap.get(kExpectedDelay);
    if (tf) {
        ledTapBins[bin] = tf->colors[0][0];
    }
}

inline float LatencyTimer::binToPhase(unsigned bin)
{
    return bin / float(kNumPhaseBins);
}

inline unsigned LatencyTimer::phaseToBin(float p)
{
    return std::min<unsigned>(kNumPhaseBins - 1, p * kNumPhaseBins);
}

inline float LatencyTimer::ledBrightness(unsigned bin)
{
    return effect.brightnessAtPhase(bin / float(kNumPhaseBins));
}

inline float LatencyTimer::cameraBrightness(unsigned bin)
{
    return phaseBinDenominators[bin] ? double(phaseBinNumerators[bin]) / double(phaseBinDenominators[bin]) : 0;
}

inline void LatencyTimer::debug()
{
    fprintf(stderr,
        "\n"
        "=== Latency chart ===\n"
        "bin, milliseconds, led_brightness, camera_brightness, "
        "phase, normalized_led_brightness, normalized_camera_brightness, led_tap_output\n");

    // Capture data
    float camBins[kNumPhaseBins];
    for (unsigned i = 0; i < kNumPhaseBins; i++) {
        camBins[i] = cameraBrightness(i);
    }

    // Data ranges
    float minLed = ledBrightness(0);
    float maxLed = minLed;
    float minCam = camBins[0];
    float maxCam = minCam;
    for (unsigned i = 1; i < kNumPhaseBins; i++) {
        minLed = std::min(minLed, ledBrightness(i));
        maxLed = std::max(maxLed, ledBrightness(i));
        minCam = std::min(minCam, camBins[i]);
        maxCam = std::max(maxCam, camBins[i]);
    }

    for (unsigned i = 0; i < kNumPhaseBins; i++) {
        float phase = binToPhase(i);
        float ms = i * kActualMillisecondsPerBin;
        float led = ledBrightness(i);
        float cam = camBins[i];
        float ledTap = ledTapBins[i];

        fprintf(stderr, "%d, %f, %f, %f, %f, %f, %f, %f\n",
            i, ms, led, cam, phase,
            (led - minLed) / (maxLed - minLed),
            (cam - minCam) / (maxCam - minCam),
            (ledTap - minLed) / (maxLed - minLed));
    }
    fprintf(stderr, "\n");
}
